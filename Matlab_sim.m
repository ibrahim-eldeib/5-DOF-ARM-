% STEP 1: Build a 6-DoF robot with a spherical base joint

% Clear workspace
clc; clear; close all;

% Load Robotics Toolbox if needed
% startup_rvc  % <-- Uncomment if using for the first time in a session

% Define DH parameters
% Link(theta, d, a, alpha)
L1 = Link('revolute', 'd', 0.0,  'a', 0,   'alpha', pi/2); % Base Yaw
L2 = Link('revolute', 'd', 0.0,  'a', 0,   'alpha', pi/2); % Base Pitch
L3 = Link('revolute', 'd', 0.3,  'a', 0,   'alpha', pi/2); % Base Roll

L4 = Link('revolute', 'd', 0.0,  'a', 0.3, 'alpha', 0);    % Elbow
L5 = Link('revolute', 'd', 0.0,  'a', 0.2, 'alpha', pi/2); % Wrist Pitch
L6 = Link('revolute', 'd', 0.1,  'a', 0.0, 'alpha', 0);    % Wrist Roll

% Create the SerialLink object
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '6DOF_Arm');

% Show DH table
% disp(robot);

% Plot robot in zero configuration
figure;
robot.plot(zeros(1,6));
title('6-DoF Robot Arm with Spherical Base Joint');
%% Step 2 move it. 
robot.teach()

%%  Forward Kinematics 
% Define a sample joint configuration (in degrees for readability)
q_deg = [30, -45, 60, 45, 20, 10];      % Yaw, Pitch, Roll, Elbow, WristPitch, WristRoll
q_rad = deg2rad(q_deg);                % Convert to radians

% Compute forward kinematics
T = robot.fkine(q_rad);                % Homogeneous transformation matrix

% Display the result
disp("=== Forward Kinematics ===");
disp("Joint Angles (deg):");
disp(q_deg);
disp("End-Effector Pose T (4x4 homogeneous transform):");
disp(T);
% Extract position
pos = T.t;  % same as T(1:3,4)
disp("End-Effector Position (x, y, z):");
disp(pos');

% Extract rotation matrix (optional)
R = T.R;
disp("Rotation Matrix:");
disp(R);

% Plot the robot in this configuration
figure;
robot.plot(q_rad);
title('Forward Kinematics Pose');
%%  Inverse Kinematics 
% Define target position (you can change this)
target_xyz = [0.3, 0.2, 0.4];

% Create a homogeneous transform with position only, identity rotation
T_target = transl(target_xyz);  % Same as SE3(x,y,z)

% Solve IK with position only using a mask: [1 1 1 0 0 0]
% Initial guess: all zeros
q_guess = zeros(1, 6);

% Compute IK
q_solution = robot.ikine(T_target, q_guess, 'mask', [1 1 1 0 0 0]);
disp("IK Joint Angles (degrees):");
disp(rad2deg(q_solution));
%% Trajectory Between Two Poses

% Define start and end end-effector positions (you can change these)
p_start = [0.25, 0.15, 0.3];
p_end   = [0.15, -0.3, 0.45];

% Build corresponding homogeneous transforms
T_start = transl(p_start);
T_end   = transl(p_end);

% Solve IK to get joint angles for each pose
q0 = robot.ikine(T_start, zeros(1,6), 'mask', [1 1 1 0 0 0]);
qf = robot.ikine(T_end,   zeros(1,6), 'mask', [1 1 1 0 0 0]);

% Check validity
if any(isnan(q0)) || any(isnan(qf))
    error('IK solution failed for one of the poses.');
end

% Generate trajectory: 100 steps
steps = 100;
[q_traj, qd, qdd] = jtraj(q0, qf, steps);

% Animate the trajectory
figure;
robot.plot(q0);  % Show start pose
hold on;
title('Trajectory Execution');

for i = 1:steps
    robot.plot(q_traj(i, :));
    pause(0.01); % Adjust for animation speed
end