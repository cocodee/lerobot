import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from lerobot_ros2.action import PolicyInference

class PolicyInferenceClient(Node):

    def __init__(self):
        super().__init__('policy_inference_client')
        self._action_client = ActionClient(self, PolicyInference, 'policy_inference')

    def send_goal(self, policy_repo_id, task_description, num_steps):
        goal_msg = PolicyInference.Goal()
        goal_msg.policy_repo_id = policy_repo_id
        goal_msg.task_description = task_description
        goal_msg.num_inference_steps = num_steps

        self._action_client.wait_for_server()
        self.get_logger().info("Sending goal to server...")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.success}, message="{result.message}"')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Step {feedback.current_step}')

def main(args=None):
    rclpy.init(args=args)
    action_client = PolicyInferenceClient()
    action_client.send_goal(
        policy_repo_id="lerobot/aliberts-xarm-pick-and-place-real",
        task_description="pick and place the red block in the green bowl",
        num_steps=300
    )
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()