
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import numpy as np


class RobotControlArchitecture(Node):
    '''
    Node for controlling the robot using a Finite State Machine (FSM) to navigate
    toward a person while avoiding obstacles.
    '''

    def __init__(self):
        '''
        Initialize the robot control node, setting up state machine, subscribers, and publishers.
        '''
        super().__init__('robot_control_architecture')

        # Define FSM states
        self.FORWARD = 0
        self.BACK = 1
        self.TURN = 2
        self.TRACK_PERSON = 3
        self.STOP = 4
        self.state = self.FORWARD
        self.state_ts = self.get_clock().now()

        # Define FSM timers and parameters
        self.TURNING_TIME = 2.0 # 2 seconds
        self.BACKING_TIME = 2.0 # 2 seconds
        self.SCAN_TIMEOUT = 1.0 # 1 second
        self.PERSON_DETECTED_TIMEOUT = 1.0 # 1 second for detecting person
        self.SPEED_LINEAR = 0.3
        self.SPEED_ANGULAR = 0.3
        self.OBSTACLE_DISTANCE = 1.0  # meters

        # Create a CvBridge object for converting ROS images to OpenCV format
        self.br = CvBridge()

        # Variables to store the latest sensor data
        self.last_scan = None
        self.last_rgb_image = None
        self.last_depth_image = None
        self.person_detected = False
        self.person_detection_ts = None

        # Subscribe to Lidar scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Subscribe to RGB camera image
        self.rgb_sub = self.create_subscription(
            Image,
            '/color/image',
            self.rgb_callback,
            10
        )

        # Subscribe to depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/stereo/depth',
            self.depth_callback,
            10
        )

        # Subscribe to MobileNet pedestrian detections (YOLO output)
        self.person_detection_sub = self.create_subscription(
            Detection2DArray,
            '/color/mobilenet_detections',
            self.person_detection_callback,
            10
        )

        # Publisher for robot velocity commands
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for controlling robot behavior based on sensor data
        self.timer = self.create_timer(0.05, self.control_cycle)

    def scan_callback(self, msg):
        '''
        Callback function to handle Lidar scan messages.
        '''
        self.last_scan = msg

    def rgb_callback(self, msg):
        '''
        Callback function to handle RGB image messages.
        '''
        self.last_rgb_image = self.br.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        '''
        Callback function to handle depth image messages.
        '''
        self.last_depth_image = self.br.imgmsg_to_cv2(msg, "32FC1")

    def person_detection_callback(self, msg):
        '''
        Callback function to handle YOLO person detection messages.
        '''
        if len(msg.detections) > 0:
            self.person_detected = True
            self.person_detection_ts = self.get_clock().now()

    def control_cycle(self):
        '''
        Main control loop to handle state transitions and robot movement based on sensor inputs.
        '''
        if self.last_scan is None:
            return  # Wait for the first scan message

        out_vel = Twist()

        if self.state == self.FORWARD:
            out_vel.linear.x = self.SPEED_LINEAR

            if self.check_obstacle_ahead():
                self.go_state(self.BACK)
            elif self.check_person_detected():
                self.go_state(self.TRACK_PERSON)

        elif self.state == self.BACK:
            out_vel.linear.x = -self.SPEED_LINEAR

            if self.check_back_2_turn():
                self.go_state(self.TURN)

        elif self.state == self.TURN:
            out_vel.angular.z = self.SPEED_ANGULAR

            if self.check_turn_2_forward():
                self.go_state(self.FORWARD)

        elif self.state == self.TRACK_PERSON:
            # Track the person, for simplicity we're just moving forward toward them
            out_vel.linear.x = self.SPEED_LINEAR
            if self.check_obstacle_ahead():
                self.go_state(self.BACK)

            # If the person detection times out, return to forward state
            if self.check_person_timeout():
                self.go_state(self.FORWARD)

        elif self.state == self.STOP:
            if self.check_stop_2_forward():
                self.go_state(self.FORWARD)

        self.vel_pub.publish(out_vel)

    def go_state(self, new_state):
        '''
        Transition to a new state and update the timestamp.
        '''
        self.state = new_state
        self.state_ts = self.get_clock().now()

    def check_obstacle_ahead(self):
        '''
        Check if an obstacle is detected ahead of the robot using Lidar data.
        '''
        if self.last_scan is None:
            return False
        # Check the front of the Lidar scan
        front_idx = len(self.last_scan.ranges) // 2
        return self.last_scan.ranges[front_idx] < self.OBSTACLE_DISTANCE

    def check_person_detected(self):
        '''
        Check if a person has been detected using the YOLO detections.
        '''
        return self.person_detected

    def check_person_timeout(self):
        '''
        Check if the person detection has timed out.
        '''
        if self.person_detection_ts is None:
            return False
        elapsed = self.get_clock().now() - self.person_detection_ts
        return elapsed > Duration(seconds=self.PERSON_DETECTED_TIMEOUT)

    def check_back_2_turn(self):
        '''
        Check if it's time to transition from backing up to turning.
        '''
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed > Duration(seconds=self.BACKING_TIME)

    def check_turn_2_forward(self):
        '''
        Check if it's time to transition from turning to moving forward.
        '''
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed > Duration(seconds=self.TURNING_TIME)

    def check_stop_2_forward(self):
        '''
        Check if it's time to transition from stop to moving forward.
        '''
        elapsed = self.get_clock().now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed < Duration(seconds=self.SCAN_TIMEOUT)


def main(args=None):
    rclpy.init(args=args)

    robot_control_architecture_node = RobotControlArchitecture()

    rclpy.spin(robot_control_architecture_node)

    robot_control_architecture_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

