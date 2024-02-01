		KINEMATIC MODEL OF C-ARM AND TABLE (10DOF)

1. Install Matlab[1] and the Robotics System Toolbox[2] 

2. Open the Carm_table_10DOF.m file

3. Carm_table_10_dof.m file is divide into 3 sections, the first section is the model of the C-Arm and table
represented as a rigid body tree model. The C-Arm model is based on Siemens Cios Alpha mobile C-Arm.
The table is modeled on the Siemens Artis OR Table. More info on rigid body tree model at https://www.mathworks.com/help/robotics/ug/rigid-body-tree-robot-model.html

4. The second section is the forward kinematics of the model. The inputs are the 10 parameters of the configuration for the 10 DOF. The output would be fktform as the homogeneous transformation matrix between the CArm end-effector and the Table end-effector wich is the common origne frame in our model. fk_Rotation_ZYX is the vector row of the Euler angles (Z,Y,X) in degrees. And the translation in x,y, and z is also available.

5. The third section is the inverse kinematics of the model. The inputs are the desired translation and rotation in meters and radians. The output is a struct array named configSoln with the 10 joint values. The homogeneous transformation matrix and Euler angles can also be output to confirm and describe the solution of the solver. The algorithm used for solving inverse kinematics is BFGSGradientProjection but LevenbergMarquardt can also be used

6. All variable can be visualised in the Matlab Workspace

[1]: https://www.mathworks.com/products/matlab.html
[2]: https://www.mathworks.com/help/robotics/
