import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from custom_msgs.action import CameraAction, CameraGoal, CameraResult, CameraFeedback

class CameraActionServer(Node):

    def __init__(self):
        super().__init__('camera_action_server')
        self.server = ActionServer(
            self,
            CameraAction,
            'camera_action',
            execute_callback=self.execute_callback)
        self.timer = self.create_timer(2, self.timer_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received a request for camera action')

        feedback_msg = CameraFeedback()
        feedback_msg.feedback = 'Capturing image...'
        goal_handle.publish_feedback(feedback_msg)

        # Simulate camera capturing image
        import random
        person_detected = random.choice([True, False])

        result = CameraResult()
        if person_detected:
            result.result = 'Person detected!'
        else:
            result.result = 'No person detected.'

        if person_detected:
            self.get_logger().info('Person detected')
            goal_handle.succeed(result)
        else:
            self.get_logger().info('No person detected')
            goal_handle.abort(result)

    def timer_callback(self):
        # Publish a camera status message
        status_msg = String()
        status_msg.data = 'Camera is active'
        self.get_logger().info('Publishing camera status')
        self.camera_status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_action_server = CameraActionServer()
    try:
        rclpy.spin(camera_action_server)
    finally:
        camera_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

