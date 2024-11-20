import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import random
import math
import subprocess
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class GreenRedLightGameNode(Node):
    """
    A ROS2 node that implements the "Green Light, Red Light" game.
    """

    def __init__(self):
        super().__init__("green_red_light_game")

        # FSM States
        self.STATE_IDLE = 0
        self.STATE_GREEN_LIGHT = 1
        self.STATE_RED_LIGHT = 2
        self.STATE_END_GAME = 3

        self.state = self.STATE_IDLE
        self.state_start_time = self.get_clock().now()

        # Game Parameters
        self.GREEN_LIGHT_DURATION = 10.0  # seconds
        self.RED_LIGHT_MIN_DURATION = 5.0  # seconds
        self.RED_LIGHT_MAX_DURATION = 10.0  # seconds
        self.MOVEMENT_SPEED = 0.2  # m/s
        self.ROTATION_SPEED = 0.5  # rad/s
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.5  # meters
        self.BOUNDARY_DISTANCE_THRESHOLD = 0.3  # meters

        # Direction Variables
        self.current_direction_angle = 0.0  # radians

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )

        # Publisher for cmd_vel
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variables to store sensor data
        self.last_scan = None

        # Players remaining (this would ideally be managed by a human judge)
        self.players_remaining = 5  # Start with 5 players

        self.get_logger().info("GreenRedLightGameNode has been started.")

    def scan_callback(self, msg):
        """
        Callback function for scan messages.
        Stores the latest scan data.
        """
        self.last_scan = msg

    def control_loop(self):
        """
        Main control loop. Adjusts robot motion based on FSM.
        """
        out_vel = Twist()
        current_time = self.get_clock().now()

        if self.state == self.STATE_IDLE:
            # Wait for game start command
            # For this example, we'll start the game automatically after 5 seconds
            elapsed_time = (current_time - self.state_start_time).nanoseconds / 1e9
            if elapsed_time > 5.0:
                self.state = self.STATE_GREEN_LIGHT
                self.state_start_time = self.get_clock().now()
                self.play_audio("green_light.wav")
                self.get_logger().info("Game started. Switching to GREEN_LIGHT state.")

        elif self.state == self.STATE_GREEN_LIGHT:
            # Robot remains stationary
            out_vel.linear.x = 0.0
            out_vel.angular.z = 0.0
            elapsed_time = (current_time - self.state_start_time).nanoseconds / 1e9
            if elapsed_time > self.GREEN_LIGHT_DURATION:
                # Transition to Red Light
                self.state = self.STATE_RED_LIGHT
                self.state_start_time = self.get_clock().now()
                self.play_audio("red_light.wav")
                self.random_direction()
                self.get_logger().info("Switching to RED_LIGHT state.")

        elif self.state == self.STATE_RED_LIGHT:
            # Move in the selected random direction
            out_vel.linear.x = self.MOVEMENT_SPEED
            out_vel.angular.z = (
                0.0  # Assume robot is already facing the correct direction
            )

            # Check for collision with player or boundary
            collision = self.detect_collision()
            boundary_reached = self.detect_boundary()

            if collision:
                # Stop movement
                out_vel.linear.x = 0.0
                # Signal player is out
                self.play_audio("player_out.wav")
                self.players_remaining -= 1
                self.get_logger().info(
                    f"Player hit! Players remaining: {self.players_remaining}"
                )
                if self.players_remaining > 1:
                    # Transition to Green Light
                    self.state = self.STATE_GREEN_LIGHT
                    self.state_start_time = self.get_clock().now()
                    self.play_audio("green_light.wav")
                    self.get_logger().info("Switching to GREEN_LIGHT state.")
                else:
                    # Transition to End Game
                    self.state = self.STATE_END_GAME
                    self.state_start_time = self.get_clock().now()
                    self.get_logger().info(
                        "Only one player remaining. Switching to END_GAME state."
                    )
            elif boundary_reached:
                # Stop movement
                out_vel.linear.x = 0.0
                # Change direction
                self.random_direction()
                # Transition to Green Light
                self.state = self.STATE_GREEN_LIGHT
                self.state_start_time = self.get_clock().now()
                self.play_audio("green_light.wav")
                self.get_logger().info(
                    "Boundary reached. Switching to GREEN_LIGHT state."
                )
            else:
                # Continue moving
                pass

        elif self.state == self.STATE_END_GAME:
            # Stop the robot
            out_vel.linear.x = 0.0
            out_vel.angular.z = 0.0
            # Announce winner
            self.play_audio("winner.wav")
            self.get_logger().info("Game ended. Announcing winner.")
            # Reset the game
            self.state = self.STATE_IDLE
            self.state_start_time = self.get_clock().now()
            self.players_remaining = 5  # Reset player count
            self.get_logger().info("Resetting game. Switching to IDLE state.")

        else:
            # Default to stop
            out_vel.linear.x = 0.0
            out_vel.angular.z = 0.0
            self.get_logger().warn("Unknown state. Stopping the robot.")

        # Publish velocities
        self.vel_pub.publish(out_vel)

    def play_audio(self, file_name):
        """
        Play an audio file using an external command.
        """
        audio_file_path = f"/home/your_username/audio_files/{file_name}"
        subprocess.Popen(["aplay", audio_file_path])
        self.get_logger().info(f"Playing audio: {file_name}")

    def random_direction(self):
        """
        Randomly select a direction for the robot to face.
        """
        # Random angle between 0 and 2*pi radians
        self.current_direction_angle = random.uniform(0, 2 * math.pi)
        # Rotate the robot to face this direction
        self.rotate_to_angle(self.current_direction_angle)
        self.get_logger().info(
            f"Random direction selected: {math.degrees(self.current_direction_angle):.2f} degrees"
        )

    def rotate_to_angle(self, target_angle):
        """
        Rotate the robot to face the target angle.
        """
        # For simplicity, we'll assume instantaneous rotation
        # In practice, you would implement rotation over time
        # Here we'll simulate rotation by publishing angular velocity until the desired angle is reached

        # Get current orientation (assuming we have access to odometry or IMU data)
        # For this example, we'll assume the robot can rotate to the target angle instantly
        pass  # Implement rotation logic if necessary

    def detect_collision(self):
        """
        Detect if a player is in front of the robot.
        """
        if self.last_scan is None:
            return False

        # Get scan parameters
        angle_min = self.last_scan.angle_min
        angle_increment = self.last_scan.angle_increment
        ranges = self.last_scan.ranges
        num_ranges = len(ranges)

        # Index corresponding to 0 degrees (front)
        index_center = int((-angle_min) / angle_increment)

        # Indices for +/- 15 degrees
        angle_range = 15 * (math.pi / 180)  # 15 degrees in radians
        index_range = int(angle_range / angle_increment)

        index_min = max(0, index_center - index_range)
        index_max = min(num_ranges - 1, index_center + index_range)

        # Get the ranges in front
        front_ranges = ranges[index_min : index_max + 1]

        # Check if any of the ranges are less than collision distance threshold
        for distance in front_ranges:
            if 0 < distance < self.OBSTACLE_DISTANCE_THRESHOLD:
                return True
        return False

    def detect_boundary(self):
        """
        Detect if the robot has reached a boundary.
        """
        if self.last_scan is None:
            return False

        # Assuming boundaries are detected when obstacles are close on all sides
        # Check left, right, and front

        ranges = self.last_scan.ranges
        num_ranges = len(ranges)

        # Indices for front, left, and right
        angle_min = self.last_scan.angle_min
        angle_increment = self.last_scan.angle_increment

        index_center = int((-angle_min) / angle_increment)
        angle_90 = 90 * (math.pi / 180)
        index_left = int((angle_90 - angle_min) / angle_increment)
        index_right = int((-angle_90 - angle_min) / angle_increment)

        # Get distances
        distance_front = ranges[index_center]
        distance_left = ranges[index_left]
        distance_right = ranges[index_right]

        # Check if any distances are less than boundary threshold
        if (
            0 < distance_front < self.BOUNDARY_DISTANCE_THRESHOLD
            or 0 < distance_left < self.BOUNDARY_DISTANCE_THRESHOLD
            or 0 < distance_right < self.BOUNDARY_DISTANCE_THRESHOLD
        ):
            return True
        else:
            return False


def main(args=None):
    rclpy.init(args=args)

    game_node = GreenRedLightGameNode()

    rclpy.spin(game_node)

    game_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
