# ConveyorBelt_Analysis
UTS Summer Scholarship Project with NextOre - developing a simulated environment of RealSense cameras analysing the load on a conveyor belt


Using conveyor belt from IFRA Cranfield (Plug-in) - https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt
and RealSense cameras (unknown model yet) - https://github.com/IntelRealSense/realsense-ros

Package Setup/Installation/Usage

Required plugin packages to clone/install into ros2_ws (Gazebo Simulation):
- IFRA's Conveyor belt package (https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt)
- realsense_gazebo_plugin (https://gitioc.upc.edu/ros2tutorials/realsense_gazebo_plugin/-/tree/main?ref_type=heads)

Models (sdf and urdf files) are included in the package and should already include the plugins in their description files:
- realsense_camera: camera on a mock up tripod to view conveyor belt from side-one position
- new_conveyor_belt: modified from IFRA's model to visually align with real world belts used in the industry

Project world is also included with conveyor belt and camera already set up in position (can be changed)
- gazebo ~/ros_ws/src/ConveyorBelt_Analysis/conveyor_belt/worlds/project_world.world

To control Conveyor Belt speed, run the following command (-- value between 0-100):
- ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: --}"

Only need to run the launch file to start all nodes (rviz, spawn, despawn. load_profile) and service call for power as well:
- ros2 launch conveyor_belt project_world.launch.py

Launch RViz manually if launch file doesn't work (it should work without depth image topic)
- rviz2 rviz2 -d ~/ros2_ws/src/ConveyorBelt_Analysis/conveyor_belt/rviz/rviz.rviz

Need to do:

- Working on load profile generation (filter.cpp)
- tf2::doTransform doesn't work (seg fault)
- pcl library has major issues, I believe it has something to do with the include headers, but not sure
