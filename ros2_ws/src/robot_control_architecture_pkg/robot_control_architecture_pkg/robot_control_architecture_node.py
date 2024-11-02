import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from enum import Enum


class State(Enum):
    SEARCH_PERSON = 1
    APPROACH_PERSON = 2
    AVOID_OBSTACLE = 3
    REACHED_PERSON = 4


class ApproachPersonNode(Node):
    """
    A ROS2 node that enables the robot to approach a detected person using OAK-D camera detections,
    while avoiding obstacles using depth data.
    """

    def __init__(self):
        super().__init__("approach_person")

        # Constants
        self.SPEED_LINEAR_MAX = 0.5  # Maximum linear speed (m/s)
        self.SPEED_LINEAR_MIN = 0.05  # Minimum linear speed (m/s)
        self.SPEED_ANGULAR_MAX = 1.0  # Maximum angular speed (rad/s)
        self.IMAGE_WIDTH = 300  # Width of the camera image (pixels)
        self.IMAGE_HEIGHT = 300  # Height of the camera image (pixels)
        self.DISTANCE_TOLERANCE = 20  # Tolerance in bounding box height (pixels)

        self.DESIRED_SIZE_Y = 400  # Desired bounding box height
        self.Kp_linear = 0.001  # Proportional gain for linear speed
        self.Kp_angular = 1.0  # Proportional gain for angular speed

        # Obstacle avoidance parameters
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.5  # meters
        self.DEPTH_IMAGE_WIDTH = 640  # Width of depth image
        self.DEPTH_IMAGE_HEIGHT = 480  # Height of depth image

        # Initialize state
        self.state = State.SEARCH_PERSON

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            "/color/mobilenet_detections",
            self.detection_callback,
            qos_profile_sensor_data,
        )

        self.depth_sub = self.create_subscription(
            Image, "/stereo/depth", self.depth_callback, qos_profile_sensor_data
        )

        self.rgb_sub = self.create_subscription(
            Image, "/color/image", self.rgb_callback, qos_profile_sensor_data
        )

        # Publisher for cmd_vel
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variables to store the latest detection
        self.person_detected = False
        self.person_center_x = 0.0
        self.person_size_y = 0.0

        # Variables for depth image
        self.depth_image = None
        self.bridge = CvBridge()

    def detection_callback(self, msg):
        """
        Callback function for detection messages.
        Processes detections to find the person with the largest bounding box.
        """
        self.person_detected = False  # Reset detection flag
        max_size_y = 0.0

        # Iterate over detections to find persons (class_id == '15')
        for detection in msg.detections:
            for result in detection.results:
                if result.hypothesis.class_id == "15":
                    bbox = detection.bbox
                    size_y = bbox.size_y
                    if size_y > max_size_y:
                        # Update the largest person detected
                        self.person_detected = True
                        self.person_center_x = (
                            bbox.center.position.x
                        )  # Center x of bounding box
                        self.person_size_y = size_y  # Height of bounding box
                        max_size_y = size_y  # Update max_size_y

        if self.person_detected:
            self.get_logger().info(
                f"Person detected at x={self.person_center_x}, size_y={self.person_size_y}"
            )

    def depth_callback(self, msg):
        """
        Callback function for depth image messages.
        Converts the depth image to a numpy array.
        """
        try:
            # Convert the depth image to a numpy array
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            # self.depth_image is now a numpy array with depth values in meters
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")
            self.depth_image = None

    def rgb_callback(self, msg):
        """
        Callback function for RGB image messages.
        For now, we will not process the RGB image.
        """
        pass  # No processing needed currently

    def control_loop(self):
        """
        Main control loop. Adjusts robot motion based on the current state.
        """
        out_vel = Twist()

        if self.state == State.SEARCH_PERSON:
            # Rotate in place to search for person
            out_vel.angular.z = 0.5  # Rotate at a constant speed
            self.get_logger().info("Searching for person...")
            if self.person_detected:
                self.get_logger().info(
                    "Person found, switching to APPROACH_PERSON state."
                )
                self.state = State.APPROACH_PERSON

        elif self.state == State.APPROACH_PERSON:
            if self.person_detected:
                # Check for obstacles
                obstacle_detected = self.check_for_obstacles()

                if obstacle_detected:
                    self.get_logger().info(
                        "Obstacle detected, switching to AVOID_OBSTACLE state."
                    )
                    self.state = State.AVOID_OBSTACLE
                else:
                    # Proceed to approach person
                    # Calculate the horizontal error (normalized between -1 and 1)
                    error_x = (self.person_center_x - self.IMAGE_WIDTH / 2) / (
                        self.IMAGE_WIDTH / 2
                    )

                    # Proportional control for angular speed
                    angular_z = (
                        -self.Kp_angular * error_x
                    )  # Negative because positive angular_z is counter-clockwise

                    # Limit angular speed
                    angular_z = max(
                        min(angular_z, self.SPEED_ANGULAR_MAX), -self.SPEED_ANGULAR_MAX
                    )

                    # Calculate linear speed based on the size of the bounding box
                    # As person_size_y increases, we get closer to the person
                    error_size_y = self.DESIRED_SIZE_Y - self.person_size_y

                    if error_size_y > self.DISTANCE_TOLERANCE:
                        # Person is too far, move forward
                        linear_x = self.Kp_linear * error_size_y
                        # Limit linear speed
                        linear_x = max(
                            min(linear_x, self.SPEED_LINEAR_MAX), self.SPEED_LINEAR_MIN
                        )
                    elif error_size_y < -self.DISTANCE_TOLERANCE:
                        # Person is too close, stop or move backward if desired
                        linear_x = 0.0  # For now, just stop
                        # Optionally, transition to REACHED_PERSON state
                        self.get_logger().info(
                            "Reached person, switching to REACHED_PERSON state."
                        )
                        self.state = State.REACHED_PERSON
                    else:
                        # Within distance tolerance, stop
                        linear_x = 0.0
                        # Optionally, transition to REACHED_PERSON state
                        self.get_logger().info(
                            "Reached person, switching to REACHED_PERSON state."
                        )
                        self.state = State.REACHED_PERSON

                    # Set velocities
                    out_vel.linear.x = linear_x
                    out_vel.angular.z = angular_z

                    self.get_logger().info(
                        f"Approaching person: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}"
                    )

            else:
                # Person lost, switch to SEARCH_PERSON
                self.get_logger().info("Person lost, switching to SEARCH_PERSON state.")
                self.state = State.SEARCH_PERSON

        elif self.state == State.AVOID_OBSTACLE:
            # Implement obstacle avoidance behavior
            # For simplicity, we can turn left or right until the path is clear
            obstacle_detected = self.check_for_obstacles()

            if obstacle_detected:
                # Rotate to avoid obstacle
                out_vel.angular.z = 0.5  # Rotate at a constant speed
                self.get_logger().info("Avoiding obstacle...")
            else:
                # Path is clear, switch back to APPROACH_PERSON
                self.get_logger().info(
                    "Obstacle avoided, switching back to APPROACH_PERSON state."
                )
                self.state = State.APPROACH_PERSON

        elif self.state == State.REACHED_PERSON:
            # Robot has reached the person, stop moving
            out_vel.linear.x = 0.0
            out_vel.angular.z = 0.0
            self.get_logger().info("Reached person, waiting...")
            # Optionally, you can add code to handle what happens when the person moves away
            if not self.person_detected:
                self.get_logger().info("Person lost, switching to SEARCH_PERSON state.")
                self.state = State.SEARCH_PERSON

        # Publish velocities
        self.vel_pub.publish(out_vel)

    def check_for_obstacles(self):
        """
        Processes the depth image to check for obstacles in front of the robot.
        Returns True if obstacle detected within threshold distance.
        """
        if self.depth_image is None:
            return False  # No depth data available, assume no obstacles

        # Define the region of interest (ROI) in the depth image
        # For example, take the central part of the image
        roi_width = int(self.DEPTH_IMAGE_WIDTH * 0.2)
        roi_height = int(self.DEPTH_IMAGE_HEIGHT * 0.2)
        x_offset = int((self.DEPTH_IMAGE_WIDTH - roi_width) / 2)
        y_offset = int((self.DEPTH_IMAGE_HEIGHT - roi_height) / 2)

        roi = self.depth_image[
            y_offset : y_offset + roi_height, x_offset : x_offset + roi_width
        ]

        # Check if any depth values in the ROI are below the obstacle threshold
        # Also, we need to handle NaN or zero values in depth data
        valid_depths = roi[np.isfinite(roi)]
        valid_depths = valid_depths[valid_depths > 0.0]

        if valid_depths.size == 0:
            return False  # No valid depth data in ROI, assume no obstacles

        min_distance = np.min(valid_depths)

        self.get_logger().info(f"Minimum distance in ROI: {min_distance:.2f} meters")

        if min_distance < self.OBSTACLE_DISTANCE_THRESHOLD:
            return True  # Obstacle detected
        else:
            return False  # No obstacle detected


def main(args=None):
    rclpy.init(args=args)

    approach_person_node = ApproachPersonNode()

    rclpy.spin(approach_person_node)

    approach_person_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
