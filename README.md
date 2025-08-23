## Terminal 1: Start `roscore`

This starts the ROS Master which all nodes communicate through.

```bash
# Terminal 1
roscore
```

Leave this running. Do NOT close this terminal.

---

## Terminal 2: Source workspace and play `.bag`

Open a new terminal and run:

```bash
# Terminal 2

# 1. Source your catkin workspace (so your Python node is recognized)
source ~/Downloads/ros_projects/devel/setup.bash

# 2. Optionally: source ROS if not in ~/.bashrc
source /opt/ros/noetic/setup.bash

# 3. Play your bag file (this will publish topics like /imu0 or /fcu/imu)
rosbag play ~/Downloads/V1_02_medium.bag
```

---

## Terminal 3: Echo IMU data

If you want to inspect the raw messages being published:

```bash
# Terminal 3
source /opt/ros/noetic/setup.bash
rostopic list                  # see available topics
rostopic echo /imu0            # or /fcu/imu or another IMU topic
```

---

## Terminal 4: Run your custom IMU listener node

```bash
# Terminal 4 
source ~/Downloads/ros_projects/devel/setup.bash
rosrun imu_listener_pkg imu_listener.py
```

---

## Recap of Topics in Your Bag

From your earlier `rosbag info`, these are the IMU-related topics available:

* `/imu0` → `sensor_msgs/Imu`
* `/fcu/imu` → `sensor_msgs/Imu`

---

## Quick Reference Table

| Item             | What it does                                | Real-world analogy                     |
| ---------------- | ------------------------------------------- | -------------------------------------- |
| `roscore`        | Starts the master and enables communication | WhatsApp server (lets devices chat)    |
| `rostopic echo`  | See what's being published to a topic       | Listening to a live radio channel      |
| Folder structure | Organizes code for ROS to compile and run   | Clean kitchen layout                    |

## Sample IMU message format from V1_02_medium.bag 
```
header:
  stamp:
    secs: 1403715533
    nsecs: 142143000
  frame_id: "imu4"

orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0

orientation_covariance: [99999.9, 0, 0, 0, 99999.9, 0, 0, 0, 99999.9]

angular_velocity:
  x: -0.49
  y: 0.26
  z: 0.12

angular_velocity_covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0]

linear_acceleration:
  x: 9.9
  y: 0.98
  z: -3.3

linear_acceleration_covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0]
```