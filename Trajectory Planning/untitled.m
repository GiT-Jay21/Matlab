robot = importrobot('mybot.urdf')
robot.DataFormat = 'row'
x1 = input('X coordinate of desired point')
y1 = input('Y coordinate of desired point')
z1 = input('Z coordinate of desired point')
homeConfig = homeConfiguration(robot);
show(robot,Visuals="on",Collisions="off");
axis([-1 1 -1 1 0 2])
gik = generalizedInverseKinematics('RigidBodyTree', robot, 'SolverAlgorithm', 'BFGSGradientProjection');

% Constraint for the first end effector
constraint1 = constraintPoseTarget('right_gripper_finger');
constraint1.TargetTransform = trvec2tform([x1 y1 z1]) * eul2tform([0 0.7854 0]);

% Constraint for the second end effector
constraint2 = constraintPoseTarget('left_gripper_finger');
constraint2.TargetTransform = trvec2tform([x1 y1 z1]) * eul2tform([0 0.7854 0]);
gik.ConstraintInputs = {'pose', 'pose'};
initialGuess = homeConfiguration(robot);
[configSolution, solutionInfo] = gik(initialGuess, constraint1, constraint2)
configSolution

%joint_values = homeConfig.JointPosition;
% Define waypoints for each joint
%waypoints = [homeConfig(1).JointPosition, configSolution(1);
%             homeConfig(2).JointPosition, configSolution(2);
 %            homeConfig(3).JointPosition, configSolution(3);
  %           homeConfig(4).JointPosition, configSolution(4);
   %          homeConfig(5).JointPosition, configSolution(5);
    %         homeConfig(6).JointPosition, configSolution(6)];

% Get joint values from home configuration 
% Extract joint positions from configurations
% First determine how many joints your robot has
numJoints = length(homeConfig)

% Extract joint positions (assuming these are already vectors or can be treated as such)
homeJointValues = homeConfig;
solutionJointValues = configSolution;


waypoints = zeros(numJoints, 2);
waypoints(:, 1) = homeJointValues;
waypoints(:, 2) = solutionJointValues;

t_f = 2;
numSamples = 4;
% Time points corresponding to each waypoint
timePoints = [0, t_f];
% Time samples for evaluation
tSamples = linspace(0, t_f, numSamples);
% Compute the cubic polynomial trajectory
[q, qd, qdd] = cubicpolytraj(waypoints, timePoints, tSamples)


% Create a figure for the animation
figure;
axis([-1 1 -1 1 0 2]); % Adjust axis limits as per your robot's workspace
grid on;
hold on;
view(0,0);
% Display the initial configuration
show(robot, q(:,1)', 'PreservePlot', false);
title('Robot Trajectory Animation');

% Animate the robot motion
for i = 1:size(q, 2)
    % Update the robot's configuration
    show(robot, q(:,i)', 'PreservePlot', false);
    drawnow;
    pause(0.1); % Adjust pause duration to control animation speed
end


% Extract the final joint positions from the initial trajectory
finalPositions = q(:, end)';

% Define waypoints: from the current position back to home
waypointsReturn = [finalPositions; homeJointValues]';
% Define the total duration for the return trajectory
t_f_return = 2; % seconds

% Number of samples for the return trajectory
numSamplesReturn = 4;

% Time points corresponding to each waypoint
timePointsReturn = [0, t_f_return];

% Generate time samples for evaluation
tSamplesReturn = linspace(0, t_f_return, numSamplesReturn);

% Compute the cubic polynomial trajectory for the return path
[qReturn, qdReturn, qddReturn] = cubicpolytraj(waypointsReturn, timePointsReturn, tSamplesReturn);
% Animate the return trajectory
figure;
axis([-1 1 -1 1 0 2]); % Adjust axis limits as per your robot's workspace
grid on;
hold on;
title('Robot Returning to Home Configuration');

% Set the desired view perspective (e.g., front view)
view(0, 0);

% Display the initial configuration at the start of the return trajectory
show(robot, qReturn(:,1)', 'PreservePlot', false);

% Loop through each time step to animate the return motion
for i = 1:size(qReturn, 2)
    % Update the robot's configuration
    show(robot, qReturn(:,i)', 'PreservePlot', false);
    drawnow;
    pause(0.1); % Adjust pause duration to control animation speed
end
