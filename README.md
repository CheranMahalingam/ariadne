# ariadne
A platform for building maps of indoor environments using RGB-D visual odometry.

## Development
To ensure reproducible builds all development occurs in containers built using Docker. To setup a
dev environment run,
```
./scripts/setup-dev-env.sh
```
Then to build and launch the visual SLAM ROS node within the container run,
```
colcon build
bass source install/setup.bash
ros2 launch ariadne_launch ariadne.launch.py
```

