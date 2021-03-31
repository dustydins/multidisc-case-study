clear
clc

%% import robot
robot = importrobot('urdf_assembly/urdf/urdf_assembly.urdf')
robot.Gravity = [0 0 -9.8];

%% define config variables
px = 0.2;
py = 0.2;
pz = 0.4;
ax = 0;
ay = 0;
az = pi;

out.config = [[[0;0;0;0;0]]]
