clear all
clc

robot = importrobot('./urdf_assembly/urdf/urdf_assembly.urdf');
robot.DataFormat = 'column';
% Define end-effector body name
eeName = 'End_Effector_Ring';
% Define the number of joints in the manipulator
numJoints = 5;

init_config = zeros(numJoints,1); % initial position of end effector

% px = 0.2;
% py = -0.2;
% pz = 0.4;
% ax = pi;
% ay = 0;
% az = 0;

px = 0.2;
py = 0.2;
pz = 0.4;
ax = 0.0;
ay = 0.0;
az = pi;

target_pose = [px, py, pz, az, ax, ay];
weights = [0.25, 0.25, 0.25, 1, 1, 1];

[config, sol_info] = inverse_k(robot, eeName, init_config, target_pose, weights);

show(robot, config, 'visuals', 'on', 'collisions', 'off');
axis([-0.4,0.4,-0.4,0.4,0.0,0.5]);
