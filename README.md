# OpenVisionMimicArm-VIMA-
Welcome to the OpenVIMA GitHub Project!
<img width="423" alt="VIMA_LOGO" src="https://github.com/paycheckpacheck/OpenVisionMimicArm-VIMA-/assets/140981896/6f5ba1ed-0854-47c5-a3fd-7fa22ead83cb">

OpenVisionMimicArm (VIMA) is a fun GUI project for interactive robot control and hand tracking of a URDF 6 DoF robot. It integrates IK and FK solvers, computer vision for hand tracking and finding user limb angle info, to  write to a robot plot and/or to a serial interfaced robot.
<img width="1117" alt="image" src="https://github.com/paycheckpacheck/OpenVisionMimicArm-VIMA-/assets/140981896/768dbc10-c721-4fd5-9fac-634ee53235de">

OpenVisionMimicArm (VIMA) is a cutting-edge Graphical User Interface (GUI) project designed to provide seamless control and hand tracking capabilities for a URDF 6 degrees-of-freedom robot. Through the power of Inverse Kinematics (IK) and Forward Kinematics (FK) solvers, combined with intuitive sliders, users can effortlessly manipulate the robot's movements and visualize its position and orientation.

**Key Features:
**1. _Kinematics Solvers:_ VIMA incorporates robust Inverse Kinematics (IK) and Forward Kinematics (FK) solvers, empowering users to precisely control the robot's motions with ease and accuracy.

2. _Real Robot Integration:_ VIMA is equipped with an advanced serial connection interface, allowing seamless interaction with a physical robot. The implementation of handshakes and a secure TCP-like protocol ensures reliable data transmission between VIMA and the robot.

3. _Innovative Computer Vision:_ Leveraging the latest computer vision technologies, VIMA offers two exciting functionalities. The hand-tracking module detects (x, y, z) coordinates, directly applied to the Inverse Kinematics solver, enabling real-time hand tracking control of the robot. Additionally, the project computes angles between a user's limbs, skillfully manipulating the robot based on the user's gestures.

4._ Trainer Module:_ VIMA's trainer feature allows users to record and replay robotic movements, creating custom sequences using a queue. This unique capability facilitates seamless teaching and testing of the robot's capabilities.

**Important Notes:
**As an evolving project, the codebase may require some debugging and optimization. Contributors are welcome to join in and help improve the codebase further.
VIMA is built with a beginner-friendly approach, focusing on functionality first. As such, some areas may require code cleanup and optimization in the future.
To enhance performance and eliminate current speed limitations, the project roadmap includes the implementation of multithreading with mutex locking for the computer vision process.
With the name "OpenVisionMimicArm (VIMA)", we embrace an open and inclusive development environment. The project welcomes enthusiasts, beginners, and experts alike to collaborate, share ideas, and contribute to making VIMA a leading-edge tool for robotic control and hand tracking.

Join us on this exciting journey as we explore the possibilities of VIMA, turning vision into reality, and unlocking the full potential of interactive robot control and hand tracking. Happy coding and robotics exploration!
