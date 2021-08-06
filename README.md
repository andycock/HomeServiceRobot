# HomeServiceRobot

This  is Udacity Robotics Software Nanodegree.
Task of this course is robot simulation in gazebo using ROS packages.

In this project was used following ROS packages:

1. Turtlebot 
This is package simulate two wheels robot and can send odometry data for other packages. Turtlebot is two wheels robot which can move forward, backward and rotate. Turtelebot was simulated in gazebo world which was created in previous project. Also we use map in pgm format for robot navigtaion
 

2. AMCL
This is package was used for navigation. AMCL is probalistic algorithm which uses particales to localise robot. Amcl takes laser-based map, laser scans and outputs pose estimates. Also it takes map to retrieve the map used for laser-based localization. ROS Navigation stack was used based on Uniform Cost Search algorithm to plan robot trajectory

3. TurtleBot rviz launcher
This package was used for displaying navigation information. In RVIZ we see robot, map which was generated before , laser-based environmentand robot trajectory.

4. Add markers
This package display object which robot will pick up in one place and kickoff in another.First we spawn marker in pickup area, wait until robot reached marker then hide marker and wait until robot reached kickoff aread. Then spawn marker in kickoff area. This is simulate object delivery by robot.

5. Pick objects
This package will navigate robot from pickup place to kick off place.
First we set goal for robot to go to pick up area and then to kickoff area.