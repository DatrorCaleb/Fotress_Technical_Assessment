The file contains the instructions for installing depedencies, running the simulation, starting the control node and calling the service.
A. Installing Dependencies
I. System Packages
Ensure your core system tools and package manager are up to date:
sudo apt update && sudo apt install
 sudo apt install python3-transforms3d

 B. Running the simulation
 I. Building the controller
 cd ~/colcon_ws
 rm -rf build/ install/ log/
 colcon build --packages-select controller
  source install/setup.bash
  ros2 run controller pose_controller_node.py
  cd ~/colcon_ws
  colcon build --packages-select controller
  
 II. Launching the sim environment
 cd ~/colcon_ws
 source install/setup.bash
 export TURTLEBOT3_MODEL=burger
 ros2 launch turtlebot3_gazebo empty_world.launch.py

 C. Launching the control node
 source install/setup.bash
ros2 run controller pose_controller_node.py  #launches the control node of the  pose controller

D. Calling the service
 source install/setup.bash
ros2 service call /go_to_pose controller/srv/GoToPose "{x: 1.0, y: 0.5, yaw: 45.0}" #calls the service with a sample target coordinates.
