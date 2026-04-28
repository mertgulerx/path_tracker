# path_tracker

`path_tracker` is a standalone ROS 2 Jazzy package that records a robot trajectory from TF and publishes it as a latched `nav_msgs/Path`.

<img src="https://raw.githubusercontent.com/mertgulerx/readme-assets/main/frontier-exploration/mertgulerx-frontier-exploration-mrtsp.gif" alt="Frontier exploration demo with Greedy MRTSP, map optimization, and preemption on a TurtleBot3 Waffle Pi" width="75%" />

## Features

- Samples the robot pose from TF at a fixed rate
- Publishes a single traversed path topic for the robot
- Resets the tracked path with either a topic or a service
- Publishes the first valid pose as an initial pose
- Uses transient local QoS so RViz can receive the latest path immediately

## Package Contents

- `path_tracker/path_tracker_node.py`: main ROS 2 node
- `config/path_tracker.yaml`: default parameters
- `launch/path_tracker.launch.py`: launch file that loads the packaged config

## Build

From your ROS 2 workspace root:

```bash
colcon build --packages-select path_tracker
source install/setup.bash
```

## Run

Using the packaged launch file:

```bash
ros2 launch path_tracker path_tracker.launch.py
```

Or directly with the packaged parameter file:

```bash
ros2 run path_tracker path_tracker_node --ros-args --params-file $(ros2 pkg prefix path_tracker)/share/path_tracker/config/path_tracker.yaml
```

## Runtime Interfaces

- Subscribes: `/path_tracker/reset` (`std_msgs/msg/Empty`)
- Publishes: `/path_tracker/path` (`nav_msgs/msg/Path`)
- Publishes: `/path_tracker/initial_pose` (`geometry_msgs/msg/PoseStamped`)
- Service: `/path_tracker/reset_path` (`std_srvs/srv/Empty`)

## Default Parameters

The default configuration assumes:

- global frame: `map`
- robot base frame: `base_footprint`
- path topic: `/path_tracker/path`
- reset topic: `/path_tracker/reset`
- initial pose topic: `/path_tracker/initial_pose`
- update rate: `5.0` Hz
- minimum translation delta: `0.05` m

You can override any parameter with your own YAML file or `--ros-args -p ...`.

## License

Apache License 2.0

## Maintainer

Maintainer: `mertgulerx`  
Support Email: `support.mertgulerx@gmail.com`
