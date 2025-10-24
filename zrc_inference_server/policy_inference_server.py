import time
import yaml
import os
import draccus
import logging
import threading
from typing import Dict, Any

# 导入 LeRobot 相关的库
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.utils import build_dataset_frame
from lerobot.policies.factory import make_policy
from lerobot.robots import make_robot_from_config, RobotConfig
from lerobot.utils.control_utils import predict_action
from lerobot.utils.utils import get_safe_torch_device

# 导入你的自定义机器人 (保持不变)
try:
    from lerobot.robots.supre_robot_follower import SupreRobotFollower
    from lerobot.robots.supre_robot_follower import SupreRobotFollowerConfig
except ImportError as e:
    # 仅在实际运行时需要这个导入，这里保持结构不变
    # print(f"\nERROR: Could not import SupreRobotFollower. Error: {e}\n")
    pass
# ------------------

# 导入 ZRC 库
from zrc.core import ZRCNode
from zrc.action import ActionServer, ActionHandle, ActionStatus
from zrc.exceptions import ZRCError

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('PolicyInferenceServer')

ACTION_NAME = "policy_inference"

# --- 1. 定义配置类 ---
class ServerConfig(draccus.Config):
    """
    用于命令行参数的配置类。
    """
    node_name: str = draccus.field(default="lerobot_inference_server", description="Name of the ZRC Node.")
    control_freq: int = draccus.field(default=30, description="Robot control frequency (Hz).")
    robot_config_path: str = draccus.field(description="REQUIRED: Path to the LeRobot robot YAML configuration file.")


class PolicyInferenceServer:

    def __init__(self, node: ZRCNode, robot_config_path: str, control_freq: int = 30):
        
        self.node = node
        logger.info("LeRobot Policy Inference Action Server is starting...")
        
        # 存储配置和频率
        self.control_freq = control_freq
        self.robot = None
        self.policy = None

        # 1. 配置加载 (替换 ROS 2 参数和包查找)
        try:
            self._load_robot_configuration(robot_config_path)
        except Exception as e:
            logger.fatal(f"Failed to load configuration: {e}")
            raise RuntimeError("Configuration Error") from e

        # 2. 创建 ZRC Action Server
        self._action_server = ActionServer(
            self.node, 
            ACTION_NAME,
            execute_callback=self.execute_callback,
            data_serializer='json' # Goal/Feedback/Result using JSON
        )
        logger.info(f"ZRC Action Server '{ACTION_NAME}' is ready.")

    def _load_robot_configuration(self, robot_config_path: str):
        """加载机器人配置，并处理关节配置文件的合并（如果存在）。"""
        if not os.path.exists(robot_config_path):
            raise FileNotFoundError(f"Robot config file not found at: {robot_config_path}")

        logger.info(f"Loading robot config from: {robot_config_path}")
        with open(robot_config_path, 'r') as f:
            robot_config_dict = yaml.safe_load(f)

        # 检查并解析 `joint_config_file`
        if 'joint_config_file' in robot_config_dict.get('robot', {}):
            joint_config_filename = robot_config_dict['robot']['joint_config_file']
            logger.info(f"Found joint config file reference: '{joint_config_filename}'")
            
            # 假设 joint_config_file 与主 robot_config_path 在同一目录下
            config_dir = os.path.dirname(os.path.abspath(robot_config_path))
            joint_config_path = os.path.join(config_dir, joint_config_filename)
            
            if not os.path.exists(joint_config_path):
                logger.error(f"Joint config file not found at resolved path: {joint_config_path}")
                raise FileNotFoundError(f"Joint config file not found at: {joint_config_path}")
            
            # 用解析出的绝对路径更新字典
            robot_config_dict['robot']['joint_config_file'] = joint_config_path
            logger.info(f"Resolved and updated 'joint_config_file' path to: '{joint_config_path}'")

        # 使用最终合并后的字典创建 LeRobot 配置对象
        logger.info(f"Instantiating robot config using draccus...")
        self.robot_config = draccus.decode(RobotConfig, robot_config_dict['robot'])

    def execute_callback(self, goal_id: str, goal_data: Dict[str, Any], handle: ActionHandle):
        """
        ZRC Action Server 的执行回调函数。
        """
        
        # 解析 Goal 数据
        policy_repo_id = goal_data.get('policy_repo_id')
        task_description = goal_data.get('task_description')
        num_inference_steps = goal_data.get('num_inference_steps', 0)

        logger.info(
            f"Executing goal {goal_id[:8]}...: policy='{policy_repo_id}', "
            f"task='{task_description}', steps={num_inference_steps}"
        )
        
        try:
            # 1. 加载策略
            logger.info(f"Loading policy from '{policy_repo_id}'...")
            policy_config = PreTrainedConfig.from_pretrained(policy_repo_id)
            self.policy = make_policy(policy_config, ds_meta=policy_config.dataset_repo_id)
            self.policy.reset()
            logger.info("Policy loaded successfully.")

            # 2. 初始化并连接机器人
            logger.info("Initializing and connecting to the robot...")
            self.robot = make_robot_from_config(self.robot_config)
            self.robot.connect()
            logger.info("Robot connected successfully.")

            # 3. 推理循环
            sleep_time = 1.0 / self.control_freq
            
            for i in range(num_inference_steps):
                
                # 检查取消请求
                if handle.is_cancel_requested():
                    logger.info(f'Goal {goal_id[:8]}... canceled by client request.')
                    result_data = {"success": False, "message": "Goal was canceled."}
                    handle.publish_result(result_data, ActionStatus.PREEMPTED)
                    return # 退出执行

                observation = self.robot.get_observation()
                observation_frame = build_dataset_frame(
                    self.policy.ds_meta['features'], observation, prefix="observation"
                )
                
                # 使用 robot_type 作为 predict_action 的参数
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


# --- 2. main 函数接收配置对象 ---
@draccus.wrap(ServerConfig)
def main(config: ServerConfig):
    """
    使用传入的配置参数启动 ZRC 节点和 Policy Inference 服务器。
    """
    
    # 1. 初始化 ZRC 节点
    try:
        zenoh_config = {} 
        zrc_node = ZRCNode(config.node_name, config=zenoh_config)
        logger.info(f"ZRC Node '{config.node_name}' initialized.")
    except ZRCError as e:
        logger.fatal(f"Failed to initialize ZRC Node: {e}")
        return

    # 2. 实例化服务器
    try:
        server = PolicyInferenceServer(
            node=zrc_node,
            robot_config_path=config.robot_config_path,
            control_freq=config.control_freq
        )
    except Exception as e:
        logger.fatal(f"Server instantiation failed: {e}")
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
    # 注意：我们删除了创建 mock 配置文件的代码块。
    # 运行此脚本时，用户必须通过命令行参数提供 robot_config_path。
    # 示例运行方式 (假设配置文件在 /path/to/my/config.yaml):
    # python your_script_name.py --robot_config_path /path/to/my/config.yaml
    
    main()