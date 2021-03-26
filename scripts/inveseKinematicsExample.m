
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Import the manipulator as a rigidBodyTree Object
robot = importrobot('../model/urdf_export/urdf_export/urdf/urdf_export.urdf');
robot.DataFormat = 'column';
% Define end-effector body name
eeName = 'End_effector';
% Define the number of joints in the manipulator
numJoints = 5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Define The Trajectory
% Define a circle to be traced over the course of 10 seconds. This circle
% is in the _xy_ plane with a radius of 0.15.
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.3 0 0];
radius = 0.10;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta)  sin(theta) zeros(size(theta)) ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% define a waypoint
pos = [0.0, 0.0, 0.5];
orientation = [0, 0, 0];
waypt0 = [pos, orientation];

% Define a [1x6] vector of relative weights on the orientation and
% position error for the inverse kinematics solver.
weights = ones(1,6);

% Transform the first waypoint to a Homogenous Transform Matrix for initialization
initTargetPose = eul2tform(waypt0(4:6));
initTargetPose(1:3,end) = waypt0(1:3)';

% Solve for q0 such that the manipulator begins at the first waypoint
ik = inverseKinematics('RigidBodyTree',robot);
[q0,solInfo] = ik(eeName,initTargetPose,weights,q0)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Loop through the trajectory of points to trace the circle. Call the |ik|
% object for each point to generate the joint configuration that achieves
% the end-effector position. Store the configurations to use later.
% 
% Initialize size of q0, the robot joint configuration at t=0. This will
% later be replaced by the first waypoint.
q0 = zeros(numJoints,1);
qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    [configSoln,solnInfo]  = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs{i} = configSoln;
    % Start from prior solution
    qInitial = configSoln;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

