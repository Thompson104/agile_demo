# DESCRIPTION
This is a proof-of-concept demo using agile_grasp, trac_ik and ros_control for a UR5 robot from Universal Robots.
The driver used for the UR5 is the ur_modern_driver.

This demo is meant to give a rough idea of how to combine these packages to make a UR5 find grasps on an object (seen as a pointcloud),
 calculate the joint configuration to reach them using trac-ik and finally move towards them using ros_control.
Note however that this demo is far from perfect and not meant for real usage, as no sanity checks are done. Therefore, the user is encouraged to use this code only in simulation.

# INSTALLATION
This demo was written and tested using ROS Kinetic.
If you are using Ubuntu, you can install ROS Kinetic by following the instructions here:
http://wiki.ros.org/kinetic/Installation/Ubuntu

Furthermore, this demo makes use of the Universal Robot simulator version 3.3.3, which can be downloaded here:
https://www.universal-robots.com/download/?option=25381#section16632

This package should be cloned within a catkin_workspace, together with the following dependencies if they are not already available on your system:

# Robot control
ros_control
git clone git@github.com:ros-controls/ros_control.git
branch kinetic-devel

ros_controllers
git clone git@github.com:ros-controls/ros_controllers.git
branch kinetic-devel

control_toolbox
git clone git@github.com:ros-controls/control_toolbox.git
branch kinetic-devel

realtime_tools
git clone git@github.com:ros-controls/realtime_tools.git
branch kinetic-devel

ur_msgs  # Subfolder of universal_robot repository.
git clone git@github.com:ros-industrial/universal_robot.git
branch indigo-devel # Also works for kinetic.

ur_description  # Subfolder of universal_robot repository.
git clone git@github.com:ros-industrial/universal_robot.git
branch indigo-devel # Also works for kinetic.

ur_modern_driver
git clone github.com:jettan/ur_modern_driver.git
branch master

# Inverse kinematics
trac_ik
git clone https://bitbucket.org/traclabs/trac_ik.git
branch master

# Grasp planning
agile_grasp
git clone git@github.com:wilson-ko/agile_grasp.git # If using OpenCV3
git clone git@github.com:atenpas/agile_grasp.git   # Original version using OpenCV2
branch master

# Miscellaneous.
dr_base
git clone git@github.com:delftrobotics/dr_base.git
branch master

dr_eigen
git clone git@github.com:delftrobotics/dr_eigen.git
branch master

dr_param
git clone git@github.com:delftrobotics/dr_param.git
branch master

Note that the original repository for agile_grasp compiles against OpenCV2.
If you system uses OpenCV3, this will not compile. Take the OpenCV3 version instead if this happens.

# RUNNING THE SYSTEM
First make sure that the UR simulator is running by following the instructions on the
 download page of Universal Robots. Once the controller is running, launch the system using:

roslaunch agile_demo_bringup agile_demo.launch

The system will then start and spawn the robot and a pointcloud of an object to grasp.
Agile grasp will immediately find grasp poses and store them within a vector.
To make the robot move to a grasp pose, execute the following command:

rosservice call /coordinator/command

This will make the system pop a grasp pose from the vector and move towards it.
Call the service again to make it move to the next grasp pose.
This can be done until all found poses are exhausted, which depends on how many agile grasp finds.
