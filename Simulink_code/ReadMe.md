   
# KINEMATIC MODEL OF C-ARM AND TABLE (10DOF)



1. Install Matlab and Simulink[1] with the Robotics System Toolbox[2] and the Simscape Multibody Toolbox[3] 

2. Open the Cios_Alpha_model_FK.m file

3. This is the forward kinematics of a 10 DOF model of a C-arm and Table combined. One can input the values of the joints in the constant block.

Joint 1 : table vertical

  Joint 2 : table Tilt

  Joint 3 : table Lateral

  Joint 4 : table Longitudinal

  Joint 5 : C-arm Lateral

  Joint 6 : C-arm Vertical

  Joint 7 : C-arm WigWag

  Joint 8 : C-arm Horizontal
  
  Joint 9 : C-arm Tilt
  
  Joint 10 : C-arm Orbital

The 3D model of the Carm and table are in the first block. The code to compute the forward kinematics is in the second block.

The two subsystems in the first block are the C-arm and Table model connected to the same 3D world frame, the MATLAB solver, and the mechanical configuration block.

4. When the visual code is run the translation and rotation of the End-effector C-arm will output. The 3D model can be visualised in the Matlab Mechanics Explorers with different views.



[1]: https://www.mathworks.com/products/matlab.html
[2]: https://www.mathworks.com/help/robotics/
[3]: https://www.mathworks.com/products/simmechanics.html
