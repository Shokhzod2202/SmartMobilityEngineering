import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_msgs.action import CameraAction, CameraGoal

class TrafficLightActionClient(Node):

    def __init__(self):
        super().__init__('traffic_light_action_client')
        self.client = ActionClient(self, CameraAction, 'camera_action')
        self.traffic_light_publisher = self.create_publisher(String, 'traffic_light_status', 10)

    def send_camera_request(self):
        self.get_logger().info('Sending camera request')
        goal = CameraGoal()
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future)

        if future.result:
            return future.result().result.result

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Camera Feedback: {feedback_msg.feedback}')

    def control_traffic_light(self):
        person_detected = self.send_camera_request()
        if person_detected == 'Person detected!':
            self.get_logger().info('Turning on the traffic light')
            status_msg = String()
            status_msg.data = 'Traffic light is ON'
            self.traffic_light_publisher.publish(status_msg)
        else:
            self.get_logger().info('Turning off the traffic light')
            status_msg = String()
            status_msg.data = 'Traffic light is OFF'
            self.traffic_light_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    traffic_light_action_client = TrafficLightActionClient()
    traffic_light_action_client.control_traffic_light()
    rclpy.spin(traffic_light_action_client)
    traffic_light_action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

