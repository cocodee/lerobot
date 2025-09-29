import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.utils import build_dataset_frame
from lerobot.policies.factory import make_policy
from lerobot.robots import make_robot_from_config, RobotConfig
from lerobot.utils.control_utils import predict_action
from lerobot.utils.utils import get_safe_torch_device

# 导入我们自定义的Action
from lerobot_ros2.action import PolicyInference

class PolicyInferenceServer(Node):

    def __init__(self):
        super().__init__('policy_inference_server')
        self.get_logger().info("LeRobot Policy Inference Action Server is starting...")

        # 声明参数
        self.declare_parameter('control_freq', 30)
        self.declare_parameter('robot', rclpy.Parameter.Type.STRUCTURE)

        # 获取机器人配置
        self.control_freq = self.get_parameter('control_freq').get_parameter_value().integer_value
        robot_config_dict = self.get_parameter('robot').get_parameter_value().structure_value
        
        # 将 ROS 2 参数字典转换为 LeRobot 的 RobotConfig 对象
        # 注意: 这部分可能需要根据 RobotConfig 的具体结构进行调整
        self.robot_config = RobotConfig.from_dict(robot_config_dict)
        self.get_logger().info(f"Loaded robot configuration: {self.robot_config}")

        self.robot = None
        self.policy = None
        
        self._action_server = ActionServer(
            self,
            PolicyInference,
            'policy_inference',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info("Action Server is ready.")

    def goal_callback(self, goal_request):
        """接受或拒绝一个 action goal。"""
        self.get_logger().info('Received goal request')
        # 在这里可以添加逻辑，比如检查当前是否已经在执行任务
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
            # 1. 加载策略
            self.get_logger().info(f"Loading policy from '{goal.policy_repo_id}'...")
            policy_config = PreTrainedConfig.from_pretrained(goal.policy_repo_id)
            # ds_meta 是为了让 policy 知道 observation/action 的结构
            # 我们可以从 policy config 中获取
            self.policy = make_policy(policy_config, ds_meta=policy_config.dataset_repo_id)
            self.policy.reset() # 重置 policy 内部状态 (例如 RNN)
            self.get_logger().info("Policy loaded successfully.")

            # 2. 初始化并连接机器人
            self.get_logger().info("Initializing and connecting to the robot...")
            self.robot = make_robot_from_config(self.robot_config)
            self.robot.connect()
            self.get_logger().info("Robot connected successfully.")

            # 创建一个 rate 对象来控制循环频率
            rate = self.create_rate(self.control_freq)

            # 3. 推理循环
            for i in range(goal.num_inference_steps):
                # 检查是否有取消请求
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    result.success = False
                    result.message = 'Goal was canceled.'
                    return result

                # a. 获取观测
                observation = self.robot.get_observation()
                
                # b. 构建策略需要的输入帧
                # policy.ds_meta['features'] 告诉我们 observation/action 的具体格式
                observation_frame = build_dataset_frame(
                    self.policy.ds_meta['features'], observation, prefix="observation"
                )

                # c. 使用策略进行推理
                action_values = predict_action(
                    observation_frame,
                    self.policy,
                    get_safe_torch_device(self.policy.config.device),
                    self.policy.config.use_amp,
                    task=goal.task_description,
                    robot_type=self.robot.robot_type
                )
                
                # 将推理结果转换为机器人可以理解的字典格式
                action = {key: action_values[i].item() for i, key in enumerate(self.robot.action_features)}

                # d. 发送动作到机器人
                self.robot.send_action(action)
                self.get_logger().debug(f"Step {i+1}: Action sent: {action}")

                # e. 发布反馈
                feedback_msg.current_step = i + 1
                goal_handle.publish_feedback(feedback_msg)
                
                # f. 等待下一个周期
                await rate.sleep() # 使用 await for async sleep

            goal_handle.succeed()
            result.success = True
            result.message = 'Inference completed successfully.'

        except Exception as e:
            self.get_logger().error(f"An error occurred during execution: {e}", exc_info=True)
            goal_handle.abort()
            result.success = False
            result.message = f"Execution failed: {e}"
        
        finally:
            # 确保机器人断开连接
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
        # 使用多线程执行器，防止 action 执行阻塞其他回调
        executor = MultiThreadedExecutor()
        rclpy.spin(policy_inference_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        policy_inference_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()