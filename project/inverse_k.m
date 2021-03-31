function [config, sol_info] = inverse_k(robot, eeName, init_config, target_pose, weights)

    config = init_config; % initial position of end effector
    
    % Define a [1x6] vector of relative weights on the orientation and
    % position error for the inverse kinematics solver.
    weights = [0.25, 0.25, 0.25, 1, 1, 1];

    % transform target pose to homogenous transform matrix
    tf = eul2tform(target_pose(4:6));
    tf(1:3, end) = target_pose(1:3);
    
    % solve for init
    ik = inverseKinematics('RigidBodyTree', robot);
    [config, sol_info] = ik(eeName, tf, weights, config);

end

