# HRI Lab 3

## How-To

### Enable the object detection topic:

Install DepthAI-ROS:

```bash
sudo apt install ros-humble-depthai-ros
```

Run the YOLO object detection node:

```bash
ros2 launch depthai_examples mobile_publisher.launch.py
ros2 launch depthai_examples rgb_stereo_node.launch.py
```

Open a new terminal, check if the topic exists:

```bash
ros2 topic list | grep mobile
```

- This should give:
  ```bash
  /colors/mobilenet_detections
  ```

Check if the topic yeilds any messages:
```bash
ros2 topic echo /colors/mobilenet_detections
```

- This should return something like this if there is no object detected:

  ```bash
  ---
  header:
    stamp:
      sec: 1730486765
      nanosec: 639241689
    frame_id: oak_rgb_camera_optical_frame
  detections: []
  ```

- If there is a person (class_id = 15) being detected:
  ```bash
  ---
  header:
    stamp:
      sec: 1730485920
      nanosec: 232146637
    frame_id: oak_rgb_camera_optical_frame
  detections:
  - header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    results:
    - hypothesis:
        class_id: '15'
        score: 0.95361328125
      pose:
        pose:
          position:
            x: 0.0
            y: 0.0
            z: 0.0
          orientation:
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
        covariance:
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
    bbox:
      center:
        position:
          x: 222.0
          y: 151.5
        theta: 0.0
      size_x: 156.0
      size_y: 299.0
    id: '15'
  ```

### Scan data from the LiDAR sensor

Check if the topic exists:

```bash
ros2 topic list | grep scan
```

Inspect the topic `/scan`:

```bash
ros2 topic echo /scan
```
- This should start to yeild some messages. If not, try rebooting the system.

### Run the package

Navigate to the ROS workspace and run the package:

```bash
rosdep install --from-path src -yi
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run robot_control_architecture_pkg robot_control_architecture_node
```
