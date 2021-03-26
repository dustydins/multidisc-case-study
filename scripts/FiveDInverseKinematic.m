% mdhparams = [0   	0	    0.065   0;
            % 0.076	pi/2    0       0
            % 0.2	    0	    0   	0;
            % 0.2   	-pi/2	0   	0;
            % 0       -pi/2	0.010   0;
            % 0       0       0       0];

% robot = rigidBodyTree;

% body1 = rigidBody('body1');
% jnt1 = rigidBodyJoint('jnt1','revolute');

% setFixedTransform(jnt1,mdhparams(1,:),'mdh');
% body1.Joint = jnt1;


% addBody(robot,body1,'base')

% body2 = rigidBody('body2');
% jnt2 = rigidBodyJoint('jnt2','revolute');
% body3 = rigidBody('body3');
% jnt3 = rigidBodyJoint('jnt3','revolute');
% body4 = rigidBody('body4');
% jnt4 = rigidBodyJoint('jnt4','revolute');
% body5 = rigidBody('body5');
% jnt5 = rigidBodyJoint('jnt5','revolute');
% body6 = rigidBody('body6');
% jnt6 = rigidBodyJoint('jnt6','revolute');

% setFixedTransform(jnt2,mdhparams(2,:),'mdh');
% setFixedTransform(jnt3,mdhparams(3,:),'mdh');
% setFixedTransform(jnt4,mdhparams(4,:),'mdh');
% setFixedTransform(jnt5,mdhparams(5,:),'mdh');
% setFixedTransform(jnt6,mdhparams(6,:),'mdh');

% body2.Joint = jnt2;
% body3.Joint = jnt3;
% body4.Joint = jnt4;
% body5.Joint = jnt5;
% body6.Joint = jnt6;

% addBody(robot,body2,'body1')
% addBody(robot,body3,'body2')
% addBody(robot,body4,'body3')
% addBody(robot,body5,'body4')
% addBody(robot,body6,'body5')

%% Import the manipulator as a rigidBodyTree Object
robot = importrobot('../model/urdf_export/urdf_export/urdf/urdf_export.urdf');
robot.DataFormat = 'column';
% Define end-effector body name
eeName = 'End_effector';
% Define the number of joints in the manipulator
numJoints = 5;

show(robot);
axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])
%axis off

%%
% Show details of the robot to validate the input properties. The robot
% should have two non-fixed joints for the rigid bodies and a fixed body
% for the end-effector.
showdetails(robot)

%% Define The Trajectory
% Define a circle to be traced over the course of 10 seconds. This circle
% is in the _xy_ plane with a radius of 0.15.
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.3 0 0];
radius = 0.10;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta)  sin(theta) zeros(size(theta)) ];

%% Inverse Kinematics Solution
% Use an |inverseKinematics| object to find a solution of robotic 
% configurations that achieve the given end-effector positions along the 
% trajectory. 

%%
% Pre-allocate configuration solutions as a matrix |qs|.
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = cell(count,1);
%%
% Create the inverse kinematics solver. 
ik = inverseKinematics('RigidBodyTree', robot);
weights =  [0.25 0.25 0.25 1 1 1];
endEffector = 'End_effector';

%%
% Loop through the trajectory of points to trace the circle. Call the |ik|
% object for each point to generate the joint configuration that achieves
% the end-effector position. Store the configurations to use later.
% 
qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    i
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    [configSoln,solnInfo]  = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs{i} = configSoln;
    % Start from prior solution
    qInitial = configSoln;

end

%% Animate The Solution
% Plot the robot for each frame of the solution using that specific robot 
% configuration. Also, plot the desired trajectory.

%%
% Show the robot in the first configuration of the trajectory. Adjust the 
% plot to show the 2-D plane that circle is drawn on. Plot the desired 
% trajectory.
figure
show(robot,q0);
%view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot3(points(:,1),points(:,2),points(:,3),'k')
axis([-0.5 0.5 -0.5 0.5 -0.5 0.5])

%%
% Set up a <docid:robotics_ref.mw_9b7bd9b2-cebc-4848-a38a-2eb93d51da03 Rate> object to display the robot 
% trajectory at a fixed rate of 15 frames per second. Show the robot in
% each configuration from the inverse kinematic solver. Watch as the arm
% traces the circular trajectory shown.
framesPerSecond = 15;
r = rateControl(framesPerSecond);

for i = 1:count

    show(robot,qs{i},'PreservePlot',false);
    drawnow
    waitfor(r);
end


%% 
% Copyright 2012 The MathWorks, Inc.
