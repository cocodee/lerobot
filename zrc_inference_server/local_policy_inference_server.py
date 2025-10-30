import time
import yaml
import os
import draccus
import logging
import threading
from typing import Dict, Any, List
from dataclasses import dataclass, asdict,field
from pathlib import Path

# 导入 LeRobot 相关的库
from lerobot.configs import parser
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.utils import build_dataset_frame
from lerobot.policies.factory import make_policy
from lerobot.robots import make_robot_from_config, RobotConfig
from lerobot.utils.control_utils import predict_action
from lerobot.utils.utils import get_safe_torch_device

import traceback
import json 
import zenoh
# 配置日志 (保持不变)
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('PolicyInferenceServer')

ACTION_NAME = "policy_inference"

# 假设导入 LeRobot 中的 EnvConfig 和相关特征类型
try:
    from lerobot.envs.configs import EnvConfig
except ImportError:
    logger.warning("Failed to import LeRobot envs. Please make sure it is installed.")

# 导入你的自定义机器人 (保持不变)
try:
    from lerobot.robots.supre_robot_follower import SupreRobotFollower
    from lerobot.robots.supre_robot_follower import SupreRobotFollowerConfig
except ImportError:
    logger.exception("Failed to import SupreRobotFollower. Please make sure it is installed.")
# ------------------

# 导入 ZRC 库 (保持不变)
from zrc.core import ZRCNode
from zrc.action import ActionServer, ActionHandle, ActionStatus
from zrc.exceptions import ZRCError


# --- 1. 定义配置类 (包含 EnvConfig) ---
@dataclass
class ServerConfig():
    robot: RobotConfig | None = None
    # 显式包含 EnvConfig 字段，允许 CLI/配置文件配置环境特征
    env: EnvConfig | None = None
    policy: PreTrainedConfig | None = None

    zenoh_config: Dict[str, Any] = field(default_factory=dict) 
    node_name: str = field(default="lerobot_inference_server")
    control_freq: int = field(default=30)

    def __post_init__(self):
        policy_path = parser.get_path_arg("policy")
        if policy_path:
            cli_overrides = parser.get_cli_overrides("policy")
            self.policy = PreTrainedConfig.from_pretrained(policy_path, cli_overrides=cli_overrides)
            self.policy.pretrained_path = policy_path
        
        if self.policy is None:
            logger.info("No policy provided at startup. Policy will be loaded via ZRC Action Goal.")

        # 如果 env 配置没有显式提供，则从 robot 配置中推导
        if self.env is None:
            logger.info("EnvConfig not provided.")

    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        return ["policy"]

class PolicyInferenceServer:

    def __init__(self, node: ZRCNode, config: ServerConfig):
        self.node = node
        logger.info("LeRobot Policy Inference Action Server is starting...")
        
        self.config = config
        self.control_freq = config.control_freq
        self.robot_config = config.robot
        self.startup_policy_config = config.policy
        # 直接使用 __post_init__ 确保已加载的 EnvConfig
        self.env_config = config.env 
        
        self.robot = None
        self.policy = None

        logger.info(f"Robot configuration loaded successfully: {self.robot_config}")
        logger.info(f"Environment configuration ready")
        
        # 2. 创建 ZRC Action Server
        self._action_server = ActionServer(
            self.node, 
            ACTION_NAME,
            execute_callback=self.execute_callback,
            data_serializer='json'
        )
        logger.info(f"ZRC Action Server '{ACTION_NAME}' is ready.")

    def execute_callback(self, goal_id: str, goal_data: Dict[str, Any], handle: ActionHandle):
        """
        ZRC Action Server 的执行回调函数。
        使用 self.env_config 来调用 make_policy。
        """
        
        # 解析 Goal 数据
        task_description = goal_data.get('task_description')
        num_inference_steps = goal_data.get('num_inference_steps', 0)

        logger.info(
            f"task='{task_description}', steps={num_inference_steps}"
        )
        
        try:
            # 1. 加载策略配置
            
            policy_config = self.startup_policy_config
            # --- 实例化策略 ---
            
            # 使用 self.env_config 作为 env_cfg
            self.policy = make_policy(policy_config, ds_meta=None, env_cfg=self.env_config)
            self.policy.reset()
            logger.info("Policy instantiated successfully using ServerConfig.env.")
            
            # --------------------

            # 2. 初始化并连接机器人
            logger.info("Initializing and connecting to the robot...")
            self.robot = make_robot_from_config(self.robot_config)
            self.robot.connect()
            logger.info("Robot connected successfully.")

            # 3. 推理循环
            sleep_time = 1.0 / self.control_freq
            
            # 获取特征：使用 policy.config.input_features
            policy_meta_features = self.policy.config.input_features
            if not policy_meta_features:
                 raise RuntimeError("Policy input features are missing after instantiation.")

            for i in range(num_inference_steps):
                
                # 检查取消请求
                if handle.is_cancel_requested():
                    logger.info(f'Goal {goal_id[:8]}... canceled by client request.')
                    result_data = {"success": False, "message": "Goal was canceled."}
                    handle.publish_result(result_data, ActionStatus.PREEMPTED)
                    return

                observation = self.robot.get_observation()
                
                # build_dataset_frame 需要输入特征
                observation_frame = build_dataset_frame(
                    policy_meta_features, observation, prefix="observation"
                )
                
                action_values = predict_action(
                    observation_frame, self.policy,
                    get_safe_torch_device(self.policy.config.device), self.policy.config.use_amp,
                    task=task_description, robot_type=self.robot.robot_type
                )
                
                action = {key: action_values[i].item() for i, key in enumerate(self.robot.action_features)}
                self.robot.send_action(action)
                
                # 发布 Feedback
                feedback_data = {"current_step": i + 1, "status": f"Running step {i+1}/{num_inference_steps}"}
                handle.publish_feedback(feedback_data)
                
                # 等待
                time.sleep(sleep_time)

            # 成功完成
            result_data = {"success": True, "message": "Inference completed successfully."}
            handle.publish_result(result_data, ActionStatus.SUCCEEDED)

        except Exception as e:
            logger.error(f"An error occurred during execution of {goal_id[:8]}...: {e}")
            # 异常中止
            result_data = {"success": False, "message": f"Execution failed: {e}"}
            handle.publish_result(result_data, ActionStatus.ABORTED)
        
        finally:
            if self.robot and self.robot.is_connected:
                logger.info(f"Disconnecting robot for goal {goal_id[:8]}...")
                self.robot.disconnect()
            self.robot = None
            self.policy = None
            logger.info(f"Execution finished for goal {goal_id[:8]}...")


# --- 2. main 函数接收配置对象 (保持不变) ---
@parser.wrap()
def main(config: ServerConfig):
    
    # 1. 初始化 ZRC 节点
    try:
        if not config.zenoh_config:
            # 如果配置是空的，传入 None 激活 Zenoh 默认行为
            zrc_config_to_pass = zenoh.Config()
        else:
            # 如果配置非空，我们必须将其转换为 Zenoh Config 对象
            # 这需要额外导入和逻辑，例如：
            # from zenoh import Config
            # zrc_config_to_pass = Config.from_json(json.dumps(config.zenoh_config))
            
            # 鉴于我们没有看到 Zenoh Config 转换逻辑，为了避免再次出现 Type Error，
            # 我们假设 ZRC 库需要 None 或 Zenoh.Config 对象。
            
            # 如果你确实需要自定义配置，请查阅 ZRC 文档，了解如何从 Python 字典创建
            # zenoh.Config 对象。
            
            # 为简单起见，如果配置非空，我们暂时传入字典，
            # 如果 ZRC 仍然报错，我们只能使用 None。
            config = zenoh.Config()
    # 从配置文件读取 Zenoh 服务器地址
            config.from_json5(json.dumps(config.zenoh_config))
            zrc_config_to_pass = config
        zrc_node = ZRCNode(config.node_name, config=zrc_config_to_pass)
        logger.info(f"ZRC Node '{config.node_name}' initialized.")
    except ZRCError as e:
        logger.fatal(f"Failed to initialize ZRC Node: {e}")
        return

    # 2. 实例化服务器
    try:
        server = PolicyInferenceServer(
            node=zrc_node,
            config=config 
        )
    except Exception as e:
        logger.exception(f"Server instantiation failed: {e}")
        zrc_node.close()
        return

    # 3. 保持主线程运行
    try:
        logger.info(f"Server is running. Press Ctrl+C to stop.")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("Server shutting down.")
    finally:
        zrc_node.close()
        logger.info("ZRC Session closed.")

if __name__ == '__main__':
    main()