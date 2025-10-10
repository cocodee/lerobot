import time
import yaml
import rclpy
import os
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.utils import build_dataset_frame
from lerobot.policies.factory import make_policy
from lerobot.robots import make_robot_from_config, RobotConfig
from lerobot.utils.control_utils import predict_action
from lerobot.utils.utils import get_safe_torch_device

# --- 关键部分 ---
# 导入你的自定义机器人。这会触发 @RobotConfig.register_subclass("supre_robot_follower")
# 让 lerobot 的工厂函数知道这个新机器人类型的存在。
try:
    from lerobot.robots.supre_robot_follower import SupreRobotFollower
    from lerobot.robots.supre_robot_follower import SupreRobotFollowerConfig
except ImportError as e:
    # 打印一个有帮助的错误信息
    print("\nERROR: Could not import SupreRobotFollower. \n"
          "Please ensure that your custom robot class is accessible in the Python path, \n"
          f"for example, by installing your lerobot library with 'pip install -e .'. Error: {e}\n")
    # 如果找不到，程序依然会启动，但在接收到action goal时会因为找不到robot type而失败
    pass
# ------------------

from lerobot_interfaces.action import PolicyInference

class PolicyInferenceServer(Node):

    def __init__(self):
        super().__init__('policy_inference_server')
        self.get_logger().info("LeRobot Policy Inference Action Server is starting...")

        # 声明参数
        self.declare_parameter('control_freq', 30)
        self.declare_parameter('robot_config_file', "robot_config.yaml")

        try:
            # 1. 获取主机器人配置字典 (来自 robot_config.yaml)
            robot_config_filename = self.get_parameter('robot_config_file').get_parameter_value().string_value
            pkg_share = get_package_share_directory('lerobot_ros2')
            robot_config_path = os.path.join(pkg_share, 'config', robot_config_filename)

            if not os.path.exists(robot_config_path):
                raise FileNotFoundError(f"Joint config file not found at: {robot_config_path}")

                # 4. 加载并合并关节配置
            self.get_logger().info(f"Loading robot config from: {robot_config_path}")
            with open(robot_config_path, 'r') as f:
                robot_config_dict = yaml.safe_load(f)

            # 2. 检查是否存在 `joint_config_file` 指令
            if 'joint_config_file' in robot_config_dict:
                # 1. 获取相对文件名，但这次不使用 .pop()，因为我们只想更新它的值
                joint_config_filename = robot_config_dict['joint_config_file']
                self.get_logger().info(f"Found joint config file reference: '{joint_config_filename}'")
            
                # 2. 解析该文件的绝对路径
                # 假设该文件位于本ROS 2包的 'config' 目录下
                pkg_share = get_package_share_directory('lerobot_ros2')
                joint_config_path = os.path.join(pkg_share, 'config', joint_config_filename)
            
                # 3. (推荐) 检查解析出的路径下的文件是否真实存在，以确保配置的有效性
                if not os.path.exists(joint_config_path):
                    self.get_logger().error(f"Joint config file not found at resolved path: {joint_config_path}")
                    # 抛出异常会停止节点启动，这通常是正确的行为，因为配置不完整
                    raise FileNotFoundError(f"Joint config file not found at: {joint_config_path}")
            
                # 4. 用解析出的绝对路径更新字典中 'joint_config_file' 键对应的值
                robot_config_dict['joint_config_file'] = joint_config_path
                
                self.get_logger().info(f"Resolved and updated 'joint_config_file' path to: '{joint_config_path}'")
            # 5. 使用最终合并后的字典创建 LeRobot 配置对象
            # `robot_config_dict` 现在包含了来自两个文件的所有信息
            self.robot_config = SupreRobotFollowerConfig.from_dict(robot_config_dict)
            self.get_logger().info("Final robot configuration created successfully.")
            
        except Exception as e:
            self.get_logger().fatal(f"Failed to load or merge configuration: {e}")
            raise RuntimeError("Configuration Error") from e

        self.control_freq = self.get_parameter('control_freq').get_parameter_value().integer_value
        self.robot = None
        self.policy = None
        
        self._action_server = ActionServer(
            self, PolicyInference, 'policy_inference',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info("Action Server is ready.")

    def goal_callback(self, goal_request):
        """接受或拒绝一个 action goal。"""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """接受或拒绝一个取消请求。"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """执行 action goal。"""
        goal = goal_handle.get_goal()
        self.get_logger().info(
            f"Executing goal: policy='{goal.policy_repo_id}', "
            f"task='{goal.task_description}', steps={goal.num_inference_steps}"
        )

        feedback_msg = PolicyInference.Feedback()
        result = PolicyInference.Result()
        
        try:
            # 1. 加载策略 (无变化)
            self.get_logger().info(f"Loading policy from '{goal.policy_repo_id}'...")
            policy_config = PreTrainedConfig.from_pretrained(goal.policy_repo_id)
            self.policy = make_policy(policy_config, ds_meta=policy_config.dataset_repo_id)
            self.policy.reset()
            self.get_logger().info("Policy loaded successfully.")

            # 2. 初始化并连接机器人 (无变化)
            # make_robot_from_config 会根据 self.robot_config 的 type 字段
            # 自动调用 SupreRobotFollower(self.robot_config)
            self.get_logger().info("Initializing and connecting to the robot...")
            self.robot = make_robot_from_config(self.robot_config)
            self.robot.connect()
            self.get_logger().info("Robot connected successfully.")

            rate = self.create_rate(self.control_freq)

            # 3. 推理循环 (无变化)
            for i in range(goal.num_inference_steps):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    result.success, result.message = False, 'Goal was canceled.'
                    return result

                observation = self.robot.get_observation()
                observation_frame = build_dataset_frame(
                    self.policy.ds_meta['features'], observation, prefix="observation"
                )
                action_values = predict_action(
                    observation_frame, self.policy,
                    get_safe_torch_device(self.policy.config.device), self.policy.config.use_amp,
                    task=goal.task_description, robot_type=self.robot.robot_type
                )
                action = {key: action_values[i].item() for i, key in enumerate(self.robot.action_features)}
                self.robot.send_action(action)
                self.get_logger().debug(f"Step {i+1}: Action sent.")
                feedback_msg.current_step = i + 1
                goal_handle.publish_feedback(feedback_msg)
                await rate.sleep()

            goal_handle.succeed()
            result.success, result.message = True, 'Inference completed successfully.'

        except Exception as e:
            self.get_logger().error(f"An error occurred during execution: {e}")
            goal_handle.abort()
            result.success, result.message = False, f"Execution failed: {e}"
        
        finally:
            if self.robot and self.robot.is_connected:
                self.get_logger().info("Disconnecting robot...")
                self.robot.disconnect()
            self.robot = None
            self.policy = None
            self.get_logger().info("Execution finished.")

        return result

def main(args=None):
    rclpy.init(args=args)
    try:
        policy_inference_server = PolicyInferenceServer()
        executor = MultiThreadedExecutor()
        rclpy.spin(policy_inference_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'policy_inference_server' in locals() and policy_inference_server.robot:
             if policy_inference_server.robot.is_connected:
                policy_inference_server.robot.disconnect()
        policy_inference_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()