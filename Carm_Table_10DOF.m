% Kinematic model of a C-arm(6DOF) and Table(4DOF) for a total of 10DOF

% Create 5 rigid bodies for the 4 DOF and the end effector of the table
TableVertical = rigidBody('TableVertical');
TableTilt = rigidBody('TableTilt');
TableLateral = rigidBody('TableLateral');
TableLongitudinal = rigidBody('TableLongitudinal');
TableEndEffector = rigidBody('TableEndEffector');

% Create 7 rigid bodies for the 6 DOF and the end effector of the CArm
CArmLateral = rigidBody('CArmLateral');
CArmVertical = rigidBody('CArmVertical');
CArmWigwag = rigidBody('CArmWigwag');
CArmHorizontal = rigidBody('CArmHorizontal');
CArmTilt = rigidBody('CArmTilt');
CArmOrbital = rigidBody('CArmOrbital');
CArmEndEffector = rigidBody('CArmEndEffector');

% Create the 4 joint of the table, one for each DOF
jntTve = rigidBodyJoint('jntTve','prismatic'); % Vertical movement of the table Z-axe
jntTti = rigidBodyJoint('jntTti','revolute');  % Tilt movement of the table Y-axe
jntTla = rigidBodyJoint('jntTla','prismatic'); % Lateral movement of the table Y-axe
jntTlo = rigidBodyJoint('jntTlo','prismatic'); % Longitudinal movement of the table X-axe

% Create the 6 joint of the CArm, one for each DOF
jntCla = rigidBodyJoint('jntCla','prismatic');  % Lateral movement of the table X-axe
jntCve = rigidBodyJoint('jntCve','prismatic');  % Vertical movement of the table Z-axe
jntCwi = rigidBodyJoint('jntCwi','revolute');   % Wigwag movement of the table Z-axe
jntCho = rigidBodyJoint('jntCho','prismatic');  % Horizontal movement of the table Y-axe
jntCti = rigidBodyJoint('jntCti','revolute');   % Tilt movement of the table Y-axe
jntCor = rigidBodyJoint('jntCor','revolute');   % Orbital movement of the table X-axe

% Define the joint position limits of the table
jntTve.PositionLimits = [-0.225 0.1];   % Meter [min max]
jntTti.PositionLimits = [-pi/12 pi/12]; % Radian [min max]
jntTla.PositionLimits = [-0.175 0.175]; % Meter [min max]
jntTlo.PositionLimits = [-0.5 0.75];    % Meter [min max]

% Define the joint position limits of the CArm
jntCla.PositionLimits = [-0.25 0.55];   % Meter [min max]
jntCve.PositionLimits = [0 0.45];       % Meter [min max]
jntCwi.PositionLimits = [-pi/15 pi/15]; % Radian [min max]
jntCho.PositionLimits = [-0.10 0.10];   % Meter [min max]
jntCti.PositionLimits = [deg2rad(-225) deg2rad(225)];   % degrees [min max]
jntCor.PositionLimits = [deg2rad(-51.5) deg2rad(96.5)]; % degrees [min max]

% using the trvec2tform function and eul2tform function to convert a
% translation vector and a set of Euler angles into a homogeneous
% transformation, Specify the transformation from one body to the next
% by setting the fixed transformation on each joint
tform1 = trvec2tform([-0.875, 0, -0.775])*eul2tform([pi, 0, 0])  % Set initial position and rotation of the vertical joint - Table
tform2 = trvec2tform([0, 0, 0.375])*eul2tform([0, 0, -pi/2]);     % Set initial position and rotation of the tilt joint  - Table
tform3 = trvec2tform([0, -0.25, 0])*eul2tform([pi/2, pi, 0]);     % Set initial position and rotation of the lateral joint  - Table
tform4 = trvec2tform([0, 0, 0])*eul2tform([0, 0, -pi/2]);         % Set initial position and rotation of the longitudinal joint  - Table
tform5 = trvec2tform([0.15, 0, 0.875])*eul2tform([0, -pi/2, pi]); % Set initial position and rotation of the end effector  - Table

%CArm
tform6 = trvec2tform([0, -1.25, -1])*eul2tform([-pi/2, 0, -pi/2]);% initial position and rotation of the lateral joint - CArm
tform7 = trvec2tform([0, -0.75, 0])*eul2tform([0, pi, pi/2]);     % initial position and rotation of the vertical joint - CArm
tform8 = trvec2tform([0, 0, 0.25])*eul2tform([-pi/2, 0, 0]);      % initial position and rotation of the Wigwag joint - CArm
tform9 = trvec2tform([0, 0, 0])*eul2tform([0, 0, -pi/2]);         % initial position and rotation of the Horizontal joint - CArm
tform10 = trvec2tform([0, 0, 0]);                                 % initial position and rotation of the Tilt joint - CArm
tform11 = trvec2tform([0, 0, 1.25])*eul2tform([-pi/2, 0, -pi/2]); % initial position and rotation of the Orbital joint - CArm
tform12 = trvec2tform([0, 0, 0])*eul2tform([0, -pi/2, pi]);       % initial position and rotation of the End effector - CArm

% Set the homogeneous transformation to the right joint
setFixedTransform(jntTve,tform1);
setFixedTransform(jntTti,tform2);
setFixedTransform(jntTla,tform3);
setFixedTransform(jntTlo,tform4);
setFixedTransform(TableEndEffector.Joint,tform5);
%CArm
setFixedTransform(jntCla,tform6);
setFixedTransform(jntCve,tform7);
setFixedTransform(jntCwi,tform8);
setFixedTransform(jntCho,tform9);
setFixedTransform(jntCti,tform10);
setFixedTransform(jntCor,tform11);
setFixedTransform(CArmEndEffector.Joint,tform12);

% Set the appropriate body to the joint
%Table
TableLateral.Joint = jntTla;
TableVertical.Joint = jntTve;
TableTilt.Joint = jntTti;
TableLongitudinal.Joint = jntTlo;

%CArm
CArmLateral.Joint = jntCla;
CArmVertical.Joint = jntCve;
CArmWigwag.Joint = jntCwi;
CArmHorizontal.Joint = jntCho;
CArmTilt.Joint = jntCti;
CArmOrbital.Joint = jntCor;

% Create a tree-structured robot. This tree is initialized with a base
% coordinate frame to attach the bodies of the Table.
CArm = rigidBodyTree('MaxNumBodies',12);

% Add the bodies together to form the table
addBody(CArm,TableVertical,'base');                 % Add TableLateral to base
addBody(CArm,TableTilt,'TableVertical');            % Add TableVertical to TableLateral
addBody(CArm,TableLateral,'TableTilt');             % Add TableTilt to TableVertical
addBody(CArm,TableLongitudinal,'TableLateral');     % Add TableLongitudinal to TableTilt
addBody(CArm,TableEndEffector,'TableLongitudinal'); % Add TableEndEffector to TableLongitudinal

% Add the bodies together to form the CArm
addBody(CArm,CArmLateral,'base');                 % Add CArmLateral to base
addBody(CArm,CArmVertical,'CArmLateral');         % Add CArmVertical to CArmLateral
addBody(CArm,CArmWigwag,'CArmVertical');          % Add CArmWigwag to CArmVertical
addBody(CArm,CArmHorizontal,'CArmWigwag');        % Add CArmHorizontal to CArmWigwag
addBody(CArm,CArmTilt,'CArmHorizontal');          % Add CArmTilt to CArmHorizontal
addBody(CArm,CArmOrbital,'CArmTilt');             % Add CArmOrbital to CArmTilt
addBody(CArm,CArmEndEffector,'CArmOrbital');      % Add CArmendeffector to CArmOrbital

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Forward kinematics

% Comfiguration of Table
config = homeConfiguration(CArm);
config(1).JointPosition = -0.5;    % Verical movement of the table in meter
config(2).JointPosition = 0;    % Tilt movement of the table in radians
config(3).JointPosition = 0;    % Lateral movement of the table in radians
config(4).JointPosition = -0.5;    % Longitudinal movement of the table in meter
% Comfiguration of CArm
config(5).JointPosition = 0.5;    % Lateral movement of the CArm in meter
config(6).JointPosition = 0;    % Vertical movement of the CArm in meter
config(7).JointPosition = 0;    % Wigwag movement of the CArm in radians
config(8).JointPosition = 0;    % Horizontal movement of the CArm in meter
config(9).JointPosition = 0;    % Tilt movement of the CArm in radians
config(10).JointPosition = 0;   % Orbital movement of the CArm in radian

% Get transform between body frames
fktform = getTransform(CArm,config,'CArmEndEffector','TableEndEffector')

% Convert the rotation matrix include in transform matrix between CArm end
% effector and Table end effector into Euler angles (Z,Y,X)
rotm = fktform(1:3,1:3);
fk_Rotation_ZYX = rotm2eul(rotm)*(180/pi) % in degrees

% Get coordinates of the end-effector
X = fktform(1,4);
Y = fktform(2,4);
Z = fktform(3,4);

% Show the CArm and Table joint model with the config configuration
show(CArm,config);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Inverse Kinematics

%Create an inverse kinematics solver
gik = generalizedInverseKinematics;
gik.RigidBodyTree = CArm;           % Set the CArm model above to the solver
gik.ConstraintInputs = {'pose'};    % Set the contrainte for the solver
%Set the algorithm for solving inverse kinematics 'BFGSGradientProjection' or 'LevenbergMarquardt'
gik.SolverAlgorithm  = ('BFGSGradientProjection') 
initialguess = homeConfiguration(CArm); % Set the initial guess as zero for all joints

% Set the value of the movement between the CArm and Table
tx = 0;     % translation in x - meter
ty = 0;     % translation in y - meter
tz = 0;     % translation in z - meter
rx = 0;     % Rotation in x - radian
ry = 0;     % Rotation in y - radian
rz = 0;     % Rotation in z - radian

% Final desired transformation matrix between CArm and Table
Finaltform = trvec2tform([tx, ty, tz])*eul2tform([rz, ry, rx]);

% Set constraintPoseTarget object properties
poseConst = constraintPoseTarget('CArmEndEffector');
poseConst.TargetTransform = Finaltform;
poseConst.ReferenceBody = ('TableEndEffector');

% Solve for the configuration that satisfies the input constraints using the gik solver
% See the configuration "configSoln" solution in the Workspace
[configSoln,solnInfo] = gik(initialguess,poseConst)

% Get the transformation matrix between the CArm and the table
iktform = getTransform(CArm,configSoln,'CArmEndEffector','TableEndEffector')

% Convert the rotation matrix include in transform matrix between CArm end
% effector and Table end effector in to Euler angles (Z,Y,X)
ikrotm = iktform(1:3,1:3);
CArm_Rotation_ZYX = rotm2eul(ikrotm)*(180/pi) % in degrees

% Get coordinates of the endeffector
X = iktform(1,4);
Y = iktform(2,4);
Z = iktform(3,4);

% Show the CArm and Table joint model with the configSoln configuration

