Usage:
source install/setup.bash
cd src
cd cev-ackermann-sim
python3 sim.py

ROS RViz:
ros2 run rviz2 rviz2

Then, add the topics on RViz that our digital car publishes.

Install: 
Make a new directory to be used as your ros workspace
Make a src directory in this workspace
From `src`, clone dependencies with the following:
- cev-ackermann-sim (this repo)
- cev_planner_ros2
- cev_trajectory_follower_ros2
- cev_msgs
- https://github.com/ros-drivers/ackermann_msgs
Then from `src`, run:

`colcon build && source install/setup.bash`

To run:
`ros2 run cev_planner_ros2 planner_node`

`ros2 launch trajectory_follower launch.py`

`python3 cev-ackermann-sim/sim.py`
