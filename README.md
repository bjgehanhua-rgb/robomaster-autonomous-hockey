\# RoboMaster Autonomous Hockey



An autonomous robotic hockey system developed for the University of Waterloo ECE 486 Control Systems course project.



The project uses a DJI RoboMaster EP mobile manipulator to autonomously navigate to a hockey stick, grasp it, navigate toward a puck, and shoot the puck toward a goal.



The system combines approximate linearization control, ROS2, geometric planning, and finite-state control. The controller was evaluated in simulation and tested on a physical RoboMaster EP robot.



\## Project Overview



The objective of this project is to design an autonomous controller that enables a RoboMaster EP robot to complete a robotic hockey task.



The complete task consists of:



1\. Navigate from the initial position to the hockey stick.

2\. Align the robot with the stick.

3\. Lower the robotic arm and grasp the stick using the gripper.

4\. Lift the stick.

5\. Navigate toward the puck while carrying the stick.

6\. Position the robot for the shot.

7\. Execute a shooting motion to send the puck toward the goal.



The complete autonomous sequence is coordinated using a finite-state control architecture.



\## Control Method



The RoboMaster mobile base is modeled using unicycle kinematics with:



\- Position: (x, y)

\- Heading angle: theta

\- Linear velocity: v

\- Angular velocity: omega



The navigation controller is based on approximate linearization.



Instead of directly controlling the center of the robot, a virtual point located a distance epsilon in front of the robot is defined as:



p = \[x + epsilon\*cos(theta), y + epsilon\*sin(theta)]



The controller computes the velocity required for this virtual point to track a desired trajectory and converts it into linear and angular velocity commands for the RoboMaster.



This approach allows the robot to follow reference trajectories while respecting the nonholonomic motion constraints of the mobile platform.



\## Autonomous Task Pipeline



The overall autonomous behavior follows a finite-state sequence:



Start

&#x20; |

&#x20; v

Navigate to Hockey Stick

&#x20; |

&#x20; v

Align with Stick

&#x20; |

&#x20; v

Lower Arm

&#x20; |

&#x20; v

Close Gripper

&#x20; |

&#x20; v

Lift Stick

&#x20; |

&#x20; v

Navigate to Puck

&#x20; |

&#x20; v

Align for Shot

&#x20; |

&#x20; v

Execute Shooting Motion

&#x20; |

&#x20; v

Stop



Separate control logic is used for navigation, stick acquisition, and shooting.



\## Simulation



The controller was first developed and evaluated in simulation before being deployed on the physical RoboMaster EP.



The repository contains simulation and testing code used during development, including RoboMaster simulation and TurtleSim-based experiments.



\### Robot Trajectory



!\[Robot Trajectory](code/Trajectory.png)



\### State Response



!\[State Response](code/State.png)



\### Control Input



!\[Control Input](code/Input.png)



Additional experimental results are available in:



\- code/Trajectory2.png

\- code/State2.png

\- code/Input2.png



\## Physical Robot Experiment



The final controller was tested on a DJI RoboMaster EP equipped with a robotic arm and gripper.



Robot localization was provided using a Vicon motion-capture system. The measured robot pose was used by the controller to generate velocity commands in real time.



The physical experiment tested the complete autonomous task:



Navigate -> Grab Stick -> Navigate -> Approach Puck -> Shoot



The system was able to autonomously execute the full task sequence without manual intervention.



\## Technologies



\- Python

\- ROS2

\- MATLAB

\- DJI RoboMaster EP

\- RoboMaster robotic arm and gripper

\- Vicon motion capture

\- Approximate linearization control

\- Finite-state machine control

\- Robot simulation



\## Repository Structure



robomaster-autonomous-hockey/

|

|-- README.md

|-- Project Report - Hanhua Ge.pdf

|

`-- code/

&#x20;   |-- controller\_final.py

&#x20;   |-- controller.py

&#x20;   |-- controller2.py

&#x20;   |-- nevigate.py

&#x20;   |-- shoot.py

&#x20;   |-- shoot2.py

&#x20;   |-- simulator.m

&#x20;   |

&#x20;   |-- simulator\_tank/

&#x20;   |   `-- multi\_robomaster\_ros\_sim-main/

&#x20;   |

&#x20;   |-- simulator\_turtle/

&#x20;   |

&#x20;   |-- Trajectory.png

&#x20;   |-- Trajectory2.png

&#x20;   |-- State.png

&#x20;   |-- State2.png

&#x20;   |-- Input.png

&#x20;   |-- Input2.png

&#x20;   `-- Model.png



\## Main Controller



The primary controller implementation is located at:



code/controller\_final.py



Additional controller files in the repository represent intermediate development and testing versions.



\## Project Report



A detailed description of the system modeling, controller design, autonomous task logic, simulation results, and physical experiments is available in:



Project Report - Hanhua Ge.pdf



\## Author



Hanhua Ge



ECE 486 - Control Systems

University of Waterloo

