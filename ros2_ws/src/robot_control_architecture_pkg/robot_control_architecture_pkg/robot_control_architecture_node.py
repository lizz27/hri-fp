import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan, Image
import math
import numpy as np


class ApproachPersonAvoidObstaclesNode(Node):
    """
    A ROS2 node that enables the robot to approach a detected person while avoiding obstacles.
    """

    def __init__(self):
        super().__init__("approach_person_avoid_obstacles")

        # FSM States
        self.SEARCHING = 0
        self.APPROACHING = 1
        self.AVOIDING = 2
        self.ARRIVED = 3
        self.state = self.SEARCHING

        # Control Parameters
        self.SPEED_LINEAR_MAX = 0.2  # Maximum linear speed (m/s)
        self.SPEED_LINEAR_MIN = 0.05  # Minimum linear speed (m/s)
        self.SPEED_ANGULAR_MAX = 1.0  # Maximum angular speed (rad/s)
        self.DISTANCE_TOLERANCE = 20  # Tolerance in bounding box height (pixels)

        self.DESIRED_SIZE_Y = 300  # Desired bounding box height (pixels)
        self.Kp_linear = 0.001  # Proportional gain for linear speed
        self.Kp_angular = 1.0  # Proportional gain for angular speed

        self.OBSTACLE_DISTANCE_THRESHOLD = 0.5  # Obstacle detection distance (meters)

        # Image size (will be updated from image topic)
        self.IMAGE_WIDTH = 640  # Default image width (pixels)
        self.IMAGE_HEIGHT = 480  # Default image height (pixels)

        # Variables to store sensor data and state information
        self.person_detected = False
        self.person_center_x = 0.0
        self.person_size_y = 0.0

        self.obstacle_detected = False
        self.min_obstacle_distance = float("inf")
        self.scan_data = None  # Store the full scan data

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            "/color/mobilenet_detections",
            self.detection_callback,
            qos_profile_sensor_data,
        )

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )

        self.image_sub = self.create_subscription(
            Image, "/color/image", self.image_callback, qos_profile_sensor_data
        )

        self.depth_sub = self.create_subscription(
            Image, "/stereo/depth", self.depth_callback, qos_profile_sensor_data
        )

        # Publisher
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

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
        else:
            self.get_logger().info("No person detected")

    def scan_callback(self, msg):
        """
        Callback function for scan messages.
        Processes laser scan data to detect obstacles.
        """
        self.scan_data = msg
        # Consider only valid ranges (ignore NaN or inf)
        valid_ranges = [
            r for r in msg.ranges if not math.isnan(r) and not math.isinf(r)
        ]
        if valid_ranges:
            self.min_obstacle_distance = min(valid_ranges)
        else:
            self.min_obstacle_distance = float("inf")

        # Check if obstacle is within threshold distance in front
        front_angles = range(-10, 10)
        obstacle_in_front = False
        for angle in front_angles:
            index = (angle - msg.angle_min) / msg.angle_increment
            index = int(index) % len(msg.ranges)
            distance = msg.ranges[index]
            if not math.isnan(distance) and distance < self.OBSTACLE_DISTANCE_THRESHOLD:
                obstacle_in_front = True
                break

        self.obstacle_detected = obstacle_in_front
        self.get_logger().info(
            f"Min obstacle distance: {self.min_obstacle_distance:.2f}, Obstacle detected: {self.obstacle_detected}"
        )

    def image_callback(self, msg):
        """
        Callback function for image messages.
        Retrieves image width and height.
        """
        self.IMAGE_WIDTH = msg.width
        self.IMAGE_HEIGHT = msg.height
        self.get_logger().info(
            f"Image size: width={self.IMAGE_WIDTH}, height={self.IMAGE_HEIGHT}"
        )

    def depth_callback(self, msg):
        """
        Callback function for depth images.
        """
        # Process depth data if needed
        pass

    def control_loop(self):
        """
        Main control loop. Adjusts robot motion based on FSM state.
        """
        out_vel = Twist()

        if self.state == self.SEARCHING:
            # State: SEARCHING
            # Behavior: Rotate to look for a person
            self.get_logger().info("State: SEARCHING")

            if self.person_detected:
                self.state = self.APPROACHING
            else:
                # Rotate in place to search for a person
                out_vel.angular.z = (
                    self.SPEED_ANGULAR_MAX / 2
                )  # Rotate at half max speed
                out_vel.linear.x = 0.0
                self.get_logger().info("Searching for person: Rotating")

        elif self.state == self.APPROACHING:
            # State: APPROACHING
            self.get_logger().info("State: APPROACHING")

            if not self.person_detected:
                # Person lost, go back to SEARCHING
                self.state = self.SEARCHING
            elif self.obstacle_detected:
                # Obstacle detected, switch to AVOIDING
                self.state = self.AVOIDING
            else:
                # Approach the person
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
                    # Person is too close, stop
                    linear_x = 0.0
                    # Transition to ARRIVED state
                    self.state = self.ARRIVED
                else:
                    # Within distance tolerance, stop
                    linear_x = 0.0
                    # Transition to ARRIVED state
                    self.state = self.ARRIVED

                # Set velocities
                out_vel.linear.x = linear_x
                out_vel.angular.z = angular_z

                self.get_logger().info(
                    f"Moving towards person: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}"
                )

        elif self.state == self.AVOIDING:
            # State: AVOIDING
            self.get_logger().info("State: AVOIDING")

            if not self.obstacle_detected:
                # Obstacle cleared
                if self.person_detected:
                    self.state = self.APPROACHING
                else:
                    self.state = self.SEARCHING
            else:
                # Avoid the obstacle by finding the clearest path
                if self.scan_data is not None:
                    # Convert scan data to numpy array
                    scan_ranges = np.array(self.scan_data.ranges)
                    scan_angles = np.linspace(
                        self.scan_data.angle_min,
                        self.scan_data.angle_max,
                        len(scan_ranges),
                    )

                    # Filter out invalid readings
                    valid_indices = np.where(np.isfinite(scan_ranges))
                    valid_ranges = scan_ranges[valid_indices]
                    valid_angles = scan_angles[valid_indices]

                    # Find the direction with maximum distance
                    if len(valid_ranges) > 0:
                        max_distance_index = np.argmax(valid_ranges)
                        max_distance_angle = valid_angles[max_distance_index]

                        # Calculate the angular error to the clearest path
                        angular_error = max_distance_angle
                        angular_z = self.Kp_angular * angular_error
                        angular_z = max(
                            min(angular_z, self.SPEED_ANGULAR_MAX),
                            -self.SPEED_ANGULAR_MAX,
                        )

                        # Set a forward speed proportional to the obstacle distance in that direction
                        linear_x = self.SPEED_LINEAR_MAX * (
                            valid_ranges[max_distance_index] / max(valid_ranges)
                        )
                        linear_x = max(
                            min(linear_x, self.SPEED_LINEAR_MAX), self.SPEED_LINEAR_MIN
                        )

                        # Set velocities
                        out_vel.linear.x = linear_x
                        out_vel.angular.z = angular_z

                        self.get_logger().info(
                            f"Avoiding obstacle: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}"
                        )
                    else:
                        # No valid ranges, stop
                        out_vel.linear.x = 0.0
                        out_vel.angular.z = self.SPEED_ANGULAR_MAX
                        self.get_logger().info("No valid LIDAR data, turning in place")
                else:
                    # No scan data, stop
                    out_vel.linear.x = 0.0
                    out_vel.angular.z = self.SPEED_ANGULAR_MAX
                    self.get_logger().info("No scan data, turning in place")

        elif self.state == self.ARRIVED:
            # State: ARRIVED
            self.get_logger().info("State: ARRIVED")

            # Stop the robot
            out_vel.linear.x = 0.0
            out_vel.angular.z = 0.0

            # Optionally, after some time, we can go back to SEARCHING
            # For now, stay in ARRIVED state

        # Publish the velocities
        self.vel_pub.publish(out_vel)


def main(args=None):
    rclpy.init(args=args)

    node = ApproachPersonAvoidObstaclesNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
