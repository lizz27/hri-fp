import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
import random
import math

class SquidGameNode(Node):
    """
    A ROS2 node that implements the 'Green Light, Red Light' game using OAK-D camera detections.
    """

    def __init__(self):
        super().__init__('squid_game')

        # Game Parameters
        self.state = 'INIT'
        self.time_limit = 120.0  # Total time limit in seconds
        self.elapsed_time = 0.0
        self.game_start_time = None

        self.green_light_duration = 0.0
        self.red_light_duration = 0.0
        self.green_light_end_time = None
        self.red_light_end_time = None

        self.movement_threshold = 10.0  # Threshold for detecting movement in pixels

        self.size_y_finish_line = 400.0  # Size_y indicating finish line
        self.player_reached_finish_line = False
        self.player_moved = False

        self.previous_detection = None
        self.current_detection = None

        self.random_interval_min = 2.0  # Minimum duration for green/red light
        self.random_interval_max = 5.0  # Maximum duration for green/red light

        # Subscriber to the person detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/color/mobilenet_detections',
            self.detection_callback,
            qos_profile_sensor_data)

        # Publisher for game state (optional)
        self.state_pub = self.create_publisher(String, 'game_state', 10)

        # Publisher for cmd_vel
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile_sensor_data)

        self.current_yaw = 0.0
        self.rotating = False
        self.rotation_target_yaw = 0.0
        self.rotation_direction = 1.0  # 1.0 for CCW, -1.0 for CW
        self.rotation_speed = 0.5  # rad/s

        # Main loop timer
        self.timer = self.create_timer(0.1, self.main_loop)

        self.get_logger().info('Squid Game Node Initialized.')

    def main_loop(self):
        if self.rotating:
            self.handle_rotation()
            return
        if self.state == 'INIT':
            self.init_state()
        elif self.state == 'GREEN_LIGHT':
            self.green_light_state()
        elif self.state == 'RED_LIGHT':
            self.red_light_state()
        elif self.state == 'GAME_OVER':
            self.game_over_state()
        else:
            self.get_logger().error(f'Unknown state: {self.state}')

    def init_state(self):
        self.get_logger().info('Game Starting. Transitioning to GREEN_LIGHT.')
        self.state = 'GREEN_LIGHT'
        self.game_start_time = self.get_clock().now()
        self.elapsed_time = 0.0
        # Assume robot is already facing away from the player
        self.start_green_light()

    def start_green_light(self):
        # Random duration for green light
        self.green_light_duration = random.uniform(self.random_interval_min, self.random_interval_max)
        duration = Duration(seconds=self.green_light_duration)
        self.green_light_end_time = self.get_clock().now() + duration
        self.get_logger().info(f'GREEN_LIGHT START. Duration: {self.green_light_duration:.2f} seconds.')
        # Announce 'Green Light'
        self.publish_state('GREEN_LIGHT')
        # Rotate robot to turn back to player
        self.start_rotation(math.pi)

    def green_light_state(self):
        # Update elapsed time
        now = self.get_clock().now()
        self.elapsed_time = (now - self.game_start_time).nanoseconds / 1e9

        # Check if time limit reached
        if self.elapsed_time >= self.time_limit:
            self.get_logger().info('Time limit reached. Player loses.')
            self.state = 'GAME_OVER'
            self.game_result = 'LOSE'
            return

        # Check if player reached finish line
        if self.player_reached_finish_line:
            self.get_logger().info('Player reached finish line. Player wins!')
            self.state = 'GAME_OVER'
            self.game_result = 'WIN'
            return

        # Check if green light duration expired
        if now >= self.green_light_end_time:
            self.get_logger().info('GREEN_LIGHT END.')
            self.state = 'RED_LIGHT'
            self.start_red_light()

    def start_red_light(self):
        # Random duration for red light
        self.red_light_duration = random.uniform(self.random_interval_min, self.random_interval_max)
        duration = Duration(seconds=self.red_light_duration)
        self.red_light_end_time = self.get_clock().now() + duration
        self.get_logger().info(f'RED_LIGHT START. Duration: {self.red_light_duration:.2f} seconds.')
        # Reset movement detection
        self.previous_detection = None
        self.player_moved = False
        # Announce 'Red Light'
        self.publish_state('RED_LIGHT')
        # Rotate robot to face the player
        self.start_rotation(math.pi)

    def red_light_state(self):
        # Update elapsed time
        now = self.get_clock().now()
        self.elapsed_time = (now - self.game_start_time).nanoseconds / 1e9

        # Check if time limit reached
        if self.elapsed_time >= self.time_limit:
            self.get_logger().info('Time limit reached. Player loses.')
            self.state = 'GAME_OVER'
            self.game_result = 'LOSE'
            return

        # Check if player moved
        if self.player_moved:
            self.get_logger().info('Player moved during RED_LIGHT. Player loses.')
            self.state = 'GAME_OVER'
            self.game_result = 'LOSE'
            return

        # Check if red light duration expired
        if now >= self.red_light_end_time:
            self.get_logger().info('RED_LIGHT END.')
            self.state = 'GREEN_LIGHT'
            self.start_green_light()

    def game_over_state(self):
        # Announce game over
        if self.game_result == 'WIN':
            self.get_logger().info('Game Over: Player Wins!')
        else:
            self.get_logger().info('Game Over: Player Loses.')
        # Publish game over state
        self.publish_state('GAME_OVER')
        # Stop the node or reset game
        self.timer.cancel()

    def detection_callback(self, msg):
        # Process detections to find the person
        person_detected = False
        max_size_y = 0.0
        detection = None

        # Iterate over detections to find persons (class_id == '15')
        for det in msg.detections:
            for result in det.results:
                if result.hypothesis.class_id == '15':
                    bbox = det.bbox
                    size_y = bbox.size_y
                    if size_y > max_size_y:
                        # Update the largest person detected
                        person_detected = True
                        detection = det
                        max_size_y = size_y

        if person_detected:
            self.current_detection = detection
            # Check if player reached finish line
            if self.current_detection.bbox.size_y >= self.size_y_finish_line:
                self.player_reached_finish_line = True

            # In RED_LIGHT state, check for movement
            if self.state == 'RED_LIGHT':
                if self.previous_detection is not None:
                    moved = self.detect_movement(self.previous_detection, self.current_detection)
                    if moved:
                        self.player_moved = True

                # Update previous detection
                self.previous_detection = self.current_detection

    def detect_movement(self, prev_det, curr_det):
        # Calculate differences in bounding box centers and sizes
        prev_bbox = prev_det.bbox
        curr_bbox = curr_det.bbox

        delta_x = abs(curr_bbox.center.position.x - prev_bbox.center.position.x)
        delta_y = abs(curr_bbox.center.position.y - prev_bbox.center.position.y)
        delta_size_x = abs(curr_bbox.size_x - prev_bbox.size_x)
        delta_size_y = abs(curr_bbox.size_y - prev_bbox.size_y)

        # Check if any movement exceeds the threshold
        if (delta_x > self.movement_threshold or
            delta_y > self.movement_threshold or
            delta_size_x > self.movement_threshold or
            delta_size_y > self.movement_threshold):
            self.get_logger().info(f'Movement detected: delta_x={delta_x}, delta_y={delta_y}, delta_size_x={delta_size_x}, delta_size_y={delta_size_y}')
            return True
        else:
            return False

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def start_rotation(self, angle_rad):
        # Record starting yaw
        self.rotation_start_yaw = self.current_yaw
        # Compute target yaw
        self.rotation_target_yaw = self.normalize_angle(self.rotation_start_yaw + angle_rad)
        # Set rotation direction
        if angle_rad >= 0:
            self.rotation_direction = 1.0
        else:
            self.rotation_direction = -1.0
        self.rotating = True
        self.get_logger().info(f'Starting rotation by {angle_rad:.2f} radians from yaw {self.rotation_start_yaw:.2f} to {self.rotation_target_yaw:.2f}.')

    def handle_rotation(self):
        yaw_error = self.normalize_angle(self.rotation_target_yaw - self.current_yaw)
        if abs(yaw_error) < 0.05:
            # Rotation complete
            self.rotating = False
            vel_msg = Twist()
            vel_msg.angular.z = 0.0
            self.vel_pub.publish(vel_msg)
            self.get_logger().info('Rotation complete.')
        else:
            # Continue rotating
            vel_msg = Twist()
            vel_msg.angular.z = self.rotation_direction * self.rotation_speed
            self.vel_pub.publish(vel_msg)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        x = orientation_q.x
        y = orientation_q.y
        z = orientation_q.z
        w = orientation_q.w
        _, _, yaw = self.euler_from_quaternion(x, y, z, w)
        self.current_yaw = yaw

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert quaternion to Euler angles.
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)

    squid_game_node = SquidGameNode()

    rclpy.spin(squid_game_node)

    squid_game_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
