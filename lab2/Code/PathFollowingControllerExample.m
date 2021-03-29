%% Path Following for a Differential Drive Robot
%% Introduction
% This example demonstrates how to control a robot to follow a desired path
% using a Robot Simulator. The example uses the Pure Pursuit path following
% controller to drive a simulated robot along a predetermined path. A desired path is a
% set of waypoints defined explicitly or computed using a path planner (refer to
% <docid:robotics_examples.example-PathPlanningExample>). The Pure Pursuit
% path following controller for a simulated differential drive robot is created and
% computes the control commands to follow a given path. The computed control commands are
% used to drive the simulated robot along the desired trajectory to
% follow the desired path based on the Pure Pursuit controller.
%
% Note: Starting in R2016b, instead of using the step method to perform the
% operation defined by the System object, you can call the object with
% arguments, as if it were a function. For example, |y = step(obj,x)| and
% |y = obj(x)| perform equivalent operations.

% Copyright 2014-2016 The MathWorks, Inc.
%% Define waypoints
% Define a set of waypoints for the desired path for the robot

% path = [2.00    1.00;
%     1.25    1.75;
%     5.25    8.25;
%     7.25    8.75;
%     11.75   10.75;
%     12.00   10.00];

% Reference trajectory specification
figure(1)
clf
lims = [0,13,0,13];
axis(lims)
hold on
%%% starting of the trajectory generation reference stuff
disp('use the mouse to input via points for the reference trajectory - right button for the last point');
button = 1;
k = 1;
path = [];
while button==1,
    [x(k),y(k),button] = ginput(1);
    path = [path; x(k),y(k)];
    plot(x(k),y(k),'r+')
    k = k + 1;
end
disp([ num2str(k-1), ' points to interpolate from '])
close(figure(1));

% Set the current location and the goal location of the robot as defined by the path
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);

%%
% Assume an initial robot orientation (the robot orientation is the angle
% between the robot heading and the positive X-axis, measured
% counterclockwise).
initialOrientation = 0;

%%
% Define the current pose for the robot [x y theta]
robotCurrentPose = [robotCurrentLocation initialOrientation];

%% Initialize the robot simulator
% A simple robot simulator is used in this example that updates and
% returns the pose of the differential drive robot for given control inputs. An external
% simulator or a physical robot will require a localization mechanism to
% provide an updated pose of the robot.

%%
% Initialize the robot simulator and assign an initial pose. The simulated
% robot has kinematic equations for the motion of a two-wheeled differential drive robot.
% The inputs to this simulated robot are linear and angular velocities. It also has plotting
% capabilities to display the robot's current location and draw the
% trajectory of the robot.

robotRadius = 0.4;
robot = ExampleHelperRobotSimulator('emptyMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);

%%
% Visualize the desired path
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])

%%
% <<path_robot_simulator_part1.png>>
%

%% Define the path following controller
% Based on the path defined above and a robot motion model, you need a path
% following controller to drive the robot along the path. Create the path
% following controller using the  |<docid:robotics_ref.buoofp1-1 robotics.PurePursuit>|  object.
controller = robotics.PurePursuit

%%
% Use the path defined above to set the desired waypoints for the
% controller
controller.Waypoints = path;

%%
% Set the path following controller parameters. The desired linear
% velocity is set to 0.3 meters/second for this example.
controller.DesiredLinearVelocity = 0.5;

%%
% The maximum angular velocity acts as a saturation limit for rotational velocity, which is
% set at 2 radians/second for this example.
controller.MaxAngularVelocity = 2;

%%
% As a general rule, the lookahead distance should be larger than the desired
% linear velocity for a smooth path. The robot might cut corners when the
% lookahead distance is large. In contrast, a small lookahead distance can
% result in an unstable path following behavior. A value of 0.5 m was chosen
% for this example.
controller.LookaheadDistance = 0.5;

%% Using the path following controller, drive the robot over the desired waypoints
% The path following controller provides input control signals for the
% robot, which the robot uses to drive itself along the desired path.
%
% Define a goal radius, which is the desired distance threshold
% between the robot's final location and the goal location. Once the robot is
% within this distance from the goal, it will stop. Also, you compute the current
% distance between the robot location and
% the goal location. This distance is continuously checked against the goal
% radius and the robot stops when this distance is less than the goal radius.
%
% Note that too small value of the goal radius may cause the robot to miss
% the goal, which may result in an unexpected behavior near the goal.
goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);

%%
% The |<docid:robotics_ref.buoofp1-1 controller>| object computes control commands for the robot.
% Drive the robot using these control commands until it reaches within the
% goal radius. If you are using an external simulator or a physical robot,
% then the controller outputs should be applied to the robot and a localization
% system may be required to update the pose of the robot. The controller runs at 10 Hz.
controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robot.getRobotPose);
    
    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);
    
    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentPose = robot.getRobotPose;
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    
    waitfor(controlRate);
    
end

%%
% <<path_completed_path_part1.png>>
%

%%
% The simulated robot has reached the goal location using the path following
% controller along the desired path. Close simulation.
delete(robot)
displayEndOfDemoMessage(mfilename)
