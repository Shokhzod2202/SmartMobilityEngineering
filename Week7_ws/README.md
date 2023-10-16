# Week7

### **Camera Action Server**

**camera_action_server.py** - This Python file sets up a ROS 2 action server for simulating a camera. When called, the camera action server generates a result that indicates whether a person was detected or not.

```python
pythonCopy code
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

```

### **Traffic Light Control Action Client**

**traffic_light_action_client.py** - This Python file creates an action client that sends a request to the camera action server. Based on the camera's response, it either turns the traffic light on or off.

```python
pythonCopy code
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

```

### Interaction diagram:

```

              +---------------------------------------+
              |                                       |
              |    Camera Action Server (Node)        |
              |                                       |
              +---------------------------------------+
                       |                       |
                       |                       |
                       | Action Request        | Result (if person detected)
                       |                       |
                       V                       V
              +---------------------------------------+
              |                                       |
              |             ROS 2 Network             |
              |                                       |
              +---------------------------------------+
                       |                       |
                       |                       |
                       |                       |
                       |                       |
                       |                       |
                       |                       |
                       V                       V
              +---------------------------------------+
              |                                       |
              |    Traffic Light Action Client (Node) |
              |                                       |
              +---------------------------------------+
                       |                       |
                       |                       |
                       |                       |
                       |                       |
                       |                       |
                       V                       V
              +---------------------------------------+
              |                                       |
              |    Traffic Light Hardware             |
              |                                       |
              +---------------------------------------+

```

In the diagram:

- The "Camera Action Server" is a ROS 2 node responsible for simulating a camera. It listens for requests and sends results back based on whether a person is detected or not.
- The "Traffic Light Action Client" is another ROS 2 node that sends a request to the camera action server to check if a person is detected. Depending on the response, it controls the traffic light.
- The "ROS 2 Network" represents the communication infrastructure that facilitates the exchange of messages between nodes.
- The "Traffic Light Hardware" represents the real-world traffic light that can be turned on or off based on the camera's input. In the simulation, this can be a publisher for a "traffic_light_status" topic.
