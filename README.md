**Simulate 6 DoF Robot Arm in ROS2, Gazebo And Moveit2**

*Introduction*

The Robot Operating System 2 (ROS 2) has emerged as a leading platform for developing robotic systems, offering a flexible and modular framework. In this report, I detail the development of a six DoF robot within ROS 2, focusing on simulation, motion planning, and programming aspects. The integration of Gazebo for simulation and visualization, MoveIt2 for motion planning, and Python for programming provides a comprehensive environment for robotic development.


*Outcomes*

 Modelling a robot in a CAD software(Autodesk Fusion)
 Simulating the robot in Gazebo
 Motion planning in MoveIt2
 Adding a python script to control the joints
 
*Prerequisites*

 Ubuntu 22.04
 ROS2 Humble
 Gazebo
 Moveit2

 
*System Architecture*

The six DoF robot employs a robotic arm configuration, featuring six joints for articulation.
Within the ROS 2 architecture, multiple nodes are utilized to control different aspects of the system, including joint controllers, sensor interfaces, and motion planning modules. 
Gazebo serves as the simulation environment, providing realistic physics and sensor feedback.
Simulation in Gazebo
Using, Autodesk Fusion I came up with the Robot model and exported it as a URDF using the URDF exporter for ROS2. I imported it into Gazebo for simulation. I added the world link to pin it to the ground of the world.

The robot lacks control in the simulated world and thusly, I added control using ros2_control. Defining the joints, the command and state interfaces and adding controller launchers in the gazebo.launch.py to spawn the model and initiate the controllers of the robot.

*Motion Planning with MoveIt2*

MoveIt2 provides a powerful framework for motion planning and manipulation tasks. The robot's kinematic model is interfaced with MoveIt2, enabling path planning in configuration space. Various motion planning algorithms, including Rapidly-exploring Random Trees (RRT), are implemented to generate collision-free trajectories.

*Programming in Python*

Python scripts are utilized to control the robot's behavior within the ROS 2 environment. Functions for trajectory generation, inverse kinematics calculations, and feedback control are implemented in Python. This allows for flexibility in programming and easy integration with ROS 2 nodes.
The script takes requires 8 arguments 6 for the joint position for each joint and 2 arguments for duration.
The file setup in vscode is as follows
the control, description and moveit configs are each their own packages in the welder_ws

*Results and Evaluation*

Simulation experiments demonstrate the effectiveness of the developed system. The robot successfully performs tasks such as pick-and-place operations or obstacle avoidance maneuvers. Performance metrics such as execution time and trajectory accuracy are evaluated to assess the system's capabilities.
