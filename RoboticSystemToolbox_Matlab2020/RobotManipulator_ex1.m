%% Initialize
clear all;
close all;
clc;

% Show - Constant bool
show_homeconfig = 0;
show_inverseKin = 0;
show_randomconfig = 0;
DefineRobot = 0;

% Inverse kinematic constant
Constant.xdes = -0.3 ;
Constant.ydes = -0.3 ;
Constant.zdes = 0.1 ;

%% Robot Definition
if DefineRobot
    robot = defineRobot();
else
    load('robot.mat');
end

%% Show the robot in home config
if show_randomconfig
    random_config = randomConfiguration(robot);
    fig = figure('Name','Robot 3D view','NumberTitle','off');
    show(robot,robot.homeConfiguration);
    showdetails(robot);
    fig.CurrentAxes.ZLim(1) = 0;
end

%% Show the robot in random config
if show_homeconfig
    home_config = homeConfiguration(robot);
    fig = figure('Name','Robot 3D view - HOME CONFIGURATION','NumberTitle','off');
    show(robot,robot.homeConfiguration);
    showdetails(robot);
    fig.CurrentAxes.ZLim(1) = 0;
end

%% InverseKinematics

if show_inverseKin
    % Define the InverseKinematic solver
    IK = inverseKinematics('RigidBodyTree',robot);

    % Parameter to pass to the solver
    Goal_Pose = HomTranf(eye(3),Constant.xdes,Constant.ydes,Constant.zdes);
    disp('The desired transformation matrix is:')
    disp(Goal_Pose)
    weights = [0 0 0 1 1 1];
    initialguess = robot.homeConfiguration;

    % Find the joint angles
    [configSoln,solnInfo] = IK( char(robot.BodyNames(robot.NumBodies)),Goal_Pose,weights,initialguess);

    fig = figure('Name','Robot 3D view','NumberTitle','off');
    show(robot,configSoln);
    Obtained_pose = getTransform(robot,configSoln,'EndEffector','base');
    fig.CurrentAxes.ZLim(1) = 0;
    
    Contant_Point_Transf = getTransform(robot,configSoln,char(robot.BodyNames(robot.NumBodies)),char(robot.BodyNames(1)));
    disp('The resulting transformation matrix is:')
    disp(Contant_Point_Transf)
end

%% Trajectory definition
t = (0:0.2:10);
count = length(t);
center = [0; 0; 0.2];
radius = 0.15;
theta = t * (2*pi/t(end));
points = center + radius * [cos(theta); sin(theta); ones(1,length(theta))*0.2];
points = points';

% Pre-allocate configuration solutions as a matrix qs.
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

% Define the InverseKinematic solver
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0, 0, 0, 1, 1, 1];

% Iterate the IK solver
qInitial = q0;

for i = 1:count
    % Solve for the configuration satisfying the desired end effector position
    point = points(i,:);
    qSol = ik('ContactPoint',trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

% Animate the solution
%figure
show(robot,qs(1,:)')
%view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
%axis([-0.1 0.7 -0.3 0.5])

% Set up a rateControl object to display the robot trajectory at a fixed rate
% of 15 frames per second. Show the robot in each configuration from the 
% inverse kinematic solver. Watch as the arm traces the circular trajectory
% shown.

framesPerSecond = 30;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%
%%% FUNCTION DEFINITION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

function HomTransf = HomTranf(RotMat,x,y,z)
HomTransf = trvec2tform([x, y, z]);
HomTransf(1:3,1:3) = RotMat;
end

function robot = defineRobot()
    robot = rigidBodyTree('DataFormat','column','MaxNumBodies',7);
    % Definition of the base
    Body0 = rigidBody('Body0');
    % Let's define the joint Joint0_1
    Jointw_0 = rigidBodyJoint('Jointw_0','fixed');
    %t_0 = [1, 0, 0, -0.085; 0, 0, -1, 0.125; 0, 1, 0, -0.06; 0, 0, 0, 1];
    t_0 = HomTranf(rotx(90),-0.085,0.125,-0.06);
    addVisual(Body0,'Mesh','Body0.stl',t_0);
    % Define the Homogenous transform and set to Body1
    tform = trvec2tform([0, 0, 0]); % User defined
    setFixedTransform(Jointw_0,tform);
    Body0.Joint = Jointw_0;
    % Add the first body to the robot
    addBody(robot,Body0,'base')

    
    % Definition of body one
    Body1 = rigidBody('Body1');
    Joint0_1 = rigidBodyJoint('Joint0_1','revolute');
    %t_1 = [1, 0, 0, ; 0, 0, 1, ; 0, -1, 0, ; 0, 0, 0, 1];
    t_1 = HomTranf(rotz(90)*rotx(-90),0.075,-0.0713,0.09);
    addVisual(Body1,'Mesh','Body1.stl',t_1)
    Joint0_1.HomePosition = 0; 
    tform2 = trvec2tform([0, 0, 0]);
    setFixedTransform(Joint0_1,tform2);
    Body1.Joint = Joint0_1;
    addBody(robot,Body1,'Body0'); 

    
    % Definition of body two
    Body2 = rigidBody('Body2');
    Joint1_2 = rigidBodyJoint('Joint1_2','revolute');
    t_2 = HomTranf(rotx(1)*rotz(90)*rotx(90)*rotz(45),-0.1052,0.0138,-0.061);
    addVisual(Body2,'Mesh','Body2.stl',t_2)
    Joint1_2.PositionLimits = [-pi/2, pi/2];
    %addVisual(Body2,'Mesh','Center_arm.stl')
    Joint1_2.JointAxis = [1, 0, 0];
    Joint1_2.HomePosition = 0; 
    tform2 = trvec2tform([0, 0, 0.13]);
    setFixedTransform(Joint1_2,tform2);
    Body2.Joint = Joint1_2;
    addBody(robot,Body2,'Body1'); 

    
    % Definition of body three
    Body3 = rigidBody('Body3');
    Joint2_3 = rigidBodyJoint('Joint2_3','revolute');
    t_3 = HomTranf(rotz(180),0.017,0.027,-0.029);
    addVisual(Body3,'Mesh','Body3.stl',t_3)
    Joint2_3.PositionLimits = [-pi/2, pi/2];
    Joint2_3.JointAxis = [1, 0, 0];
    Joint2_3.HomePosition = 0; 
    tform3 = trvec2tform([0, 0, 0.18]);
    setFixedTransform(Joint2_3,tform3);
    Body3.Joint = Joint2_3;
    addBody(robot,Body3,'Body2'); 

    
    % Definition of body four
    Body4 = rigidBody('Body4');
    Joint3_4 = rigidBodyJoint('Joint3_4','revolute');
    t_4 = HomTranf(rotx(5)*rotz(180)*roty(-90),-0.05,0.05,-0.02);
    addVisual(Body4,'Mesh','Body4.stl',t_4)
    Joint3_4.PositionLimits = [-pi/2, pi/2];
    Joint3_4.JointAxis = [1, 0, 0];
    Joint3_4.HomePosition = 0; 
    tform4 = trvec2tform([0, 0, 0.115]);
    setFixedTransform(Joint3_4,tform4);
    Body4.Joint = Joint3_4;
    addBody(robot,Body4,'Body3'); 

    
    % Definition of the endEffector
    EndEffector = rigidBody('EndEffector');
    Joint4_E = rigidBodyJoint('Joint4_E','revolute');
    t_5 = HomTranf(rotx(180),-0.055,0.06,0.12);
    addVisual(EndEffector,'Mesh','Body5.stl',t_5)
    Joint4_E.PositionLimits = [-pi/2, pi/2];
    Joint4_E.JointAxis = [0, 0, 1];
    Joint4_E.HomePosition = 0; 
    tformE = trvec2tform([0, 0, 0.09]);
    setFixedTransform(Joint4_E,tformE);
    EndEffector.Joint = Joint4_E;
    addBody(robot,EndEffector,'Body4'); 
    
    % Definition of the contact point
    ContactPoint = rigidBody('ContactPoint');
    JointE_C = rigidBodyJoint('JointE_C','fixed');
    tformC = trvec2tform([0, 0, 0.1]);
    setFixedTransform(JointE_C,tformC);
    ContactPoint.Joint = JointE_C;
    addBody(robot,ContactPoint,'EndEffector');
    
    % save the new robot
    save('robot','robot')
end