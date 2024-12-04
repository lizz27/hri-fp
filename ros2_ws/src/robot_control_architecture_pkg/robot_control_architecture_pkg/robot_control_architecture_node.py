import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import time
import os
import threading

class SquidGameNode(Node):
    """
    A ROS2 node that alternates between 'Green Light' (rotating) and 'Red Light' (stopping).
    """

    def __init__(self):
        super().__init__('squid_game')

        # Publisher for cmd_vel
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for main loop
        self.timer = self.create_timer(0.1, self.main_loop)

        # State variables
        self.state = 'INIT'
        self.green_light_duration = 0.0

        self.get_logger().info('Squid Game Node Initialized.')
        
        # Music Control
        #self.music_thread = None # Thread for playing the music
        #self.music_stop_flag = threading.Event()  # Flag to stop music

    def main_loop(self):
        if self.state == 'INIT':
            self.start_green_light()
        elif self.state == 'GREEN_LIGHT':
            self.green_light()
        elif self.state == 'RED_LIGHT':
            self.red_light()

    def start_green_light(self):
        # Start Green Light state
        self.green_light_duration = random.uniform(3, 5)  # Random duration between 3-5 seconds
        self.green_light_end_time = time.time() + self.green_light_duration
        os.system("mpg123 green_light.mp3")
        self.state = 'GREEN_LIGHT'
        self.get_logger().info(f'GREEN_LIGHT START. Rotating for {self.green_light_duration:.2f} seconds.')
        # self.start_music("squid_games_remix.mp3")
        self.start_rotation()

    def green_light(self):
        # Continue rotating until the duration ends
        if time.time() >= self.green_light_end_time:
            self.stop_rotation()
            # self.stop_music()
            self.start_red_light()

    def start_red_light(self):
        # Start Red Light state
        self.get_logger().info('RED_LIGHT START. Waiting for 3 seconds.')
        os.system("mpg123 red_light.mp3")
        self.state = 'RED_LIGHT'
        self.stop_rotation()
        time.sleep(3)  # Wait for 3 seconds
        self.start_green_light()

    def red_light(self):
        # Transition back to Green Light after Red Light
        self.start_green_light()

    def start_rotation(self):
        # Continuously send rotation commands until the duration ends
        start_time = time.time()
        while time.time() < start_time + self.green_light_duration:
            vel_msg = Twist()
            vel_msg.angular.z = 1.0  # Set a more dramatic rotational speed
            self.vel_pub.publish(vel_msg)
            time.sleep(0.1)  # Send commands at regular intervals
        self.get_logger().info('Robot rotating.')

    def stop_rotation(self):
        # Command the robot to stop
        vel_msg = Twist()
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)
        self.get_logger().info('Robot stopped.')
        # self.stop_music()    

    def start_music(self, file_path):
        # Start playing music in a separate thread
        self.music_stop_flag.clear()
        self.music_thread = threading.Thread(target=self.play_music, args=(file_path,))
        self.music_thread.start()

    def stop_music(self):
        # Stop the music playback
        self.music_stop_flag.set()
        if self.music_thread:
            self.music_thread.join()

    def play_music(self, file_path):
        while not self.music_stop_flag.is_set():
            os.system(f"mpg123 {file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = SquidGameNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_music()  # Ensure music stops when node shuts down
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
