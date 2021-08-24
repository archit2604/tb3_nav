# tb3_nav
Point to point navigation of a tb3 using a PID controller

## Installing turtlebot3 in ROS noetic

https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/

## Navigation Stack Contents:
There are two ROS nodes in this stack (/src):
- planner.py- Asks the user for the goal it on the `/path` topic.
- controller.py-Controller node which subscribes to the path and publishes velocities to cmd_vel.The controller used for the motion of the omnibase is a PID controller.

This stack also has a roslaunch file `nav.launch`. This launch file runs both the above mentioned nodes.It also launches a seperate terminal window to take inputs from the user.

## Running the navigation code
### Please follow these instructions to reproduce the results:
Clone stack:
```bash
cd ~/catkin_ws/src
git clone https://github.com/archit2604/tb3_nav.git
cd ~/catkin_ws
catkin build
source devel/setup.bash
```
Launch the turtlebot in an empty environment in gazebo:
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
A Gazebo window opens up with the turtlebot in an empty world.

Run the stack:
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch tb3_nav nav_fixed.launch
```
Another terminal window opens up showing the status of the stack.It asks the user for the goal. It calculates the wheel velocities required to reach the goal and then guides the bot towards the goal.

Use Ctrl+C to stop the node.
