# Plane fitting and visualization in Rviz

RViz does not have a Plane marker so does not directly let you visualize planes. 
So, this is some sample code for (least squares) fitting a plane given a set of points. 

To use, clone to your `catkin_workspace/src` and `catkin_make`:
1. `roscore &`
2. `rosrun using_markers basic_shapes`
3. `rosrun rviz rviz`
4. Add 2 markers and subscribe to `visualization_marker` and `point_marker_list` 


Pre-requisites: ROS-Kinetic and [eigen_catkin](https://github.com/ethz-asl/eigen_catkin)
