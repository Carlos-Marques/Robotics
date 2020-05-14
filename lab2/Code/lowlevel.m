%% Robotics - Lab 2 - Low level controller
%
% 2nd Semester 2019/2020
%
% Carlos Marques, 81323
% Gon?alo Pereira, 81602
% Ivan Andrushka, 86291
%
%% Clear all variables and close figures
clear;
close all;

%% Simulation parameters

hi = 0.01;  %this controls the time interval of the ref traj

% Car specs
car_length = 3.332; %m
car_width = 1.508; %m
L = 2.2; %m
Lf = (car_length - L)/2; %m
Lb = Lf; %m
weight = 810; %Kg
w_radius = 0.256; %m

% Car polygon constructed counter-clock wise
car_polygon = [ -Lb, -car_width/2;
                L + Lf, -car_width/2;
                L + Lf, car_width/2;
                -Lb, car_width/2 ];
            
% Reference trajectory specification
figure(1)
clf
axis([-10,10,-10,10])
hold on
%%% starting of the trajectory generation reference stuff
disp('use the mouse to input via points for the reference trajectory - right button for the last point');
button = 1;
k = 1;
while button==1,
    [x(k),y(k),button] = ginput(1);
    plot(x(k),y(k),'r+')
    k = k + 1;
end
drawnow;
disp([ num2str(k-1), ' points to interpolate from '])

% Trajectory information
npt = length(x); %number of via points, including initial and final
nvia = [0:1:npt-1];
csinterp_x = csapi(nvia,x);
csinterp_y = csapi(nvia,y);
time = [0:hi:npt-1];
xx = fnval(csinterp_x, time);
yy = fnval(csinterp_y, time);
plot(xx,yy, 'r')

% Compute angle of the tangent to the reference trajectory
for k=1:length(xx)-1,
    theta_tr(k) = atan2(yy(k+1)-yy(k), xx(k+1)-xx(k));
end
theta_tr(k+1) = theta_tr(k);

% Input the initial position and orientation for the car
disp('input the position and orientation of the car - 2 points')
[car_x, car_y] = ginput(2);
plot(car_x, car_y,'b');
plot(car_x(1),car_y(1),'bo');
drawnow
car_t = atan2(car_y(2)-car_y(1), car_x(2)-car_x(1)); %initial orientation of the car

% Initialization parameters
h = 0.01; % time step for each iteration (s)
k = 1; % sampling instant
initialPose = [car_x(1), car_y(1), car_t, 0]; % initial configuration for the car
carPose(1,:) = initialPose;
ref = [xx', yy', theta_tr'];   % reference trajectory 
npt_ref = length(ref(:,1)); % number of points in the trajectory
look_ahead_idx = 10;  % number of points ahead of the point in the reference traj
                        % that is closest to the car used to "look ahead"
                        
% Limitations
maxPhi = pi/4; %Steering angle maximum value (absolute)

% Goal condition
goalRadius = 2;
distanceToGoal = norm(carPose(1,1:2)-ref(end,1:2));
nsimul = 30000;  % max number of iterations of the simulation
turn_back_flag = 0;

% Controller parameters
Kl = 5;
Ks = 25;
Kv = 2;
rate = 10; %Control rate at 10Hz

while(distanceToGoal > goalRadius && k < nsimul && turn_back_flag ==0)
    
    % Get the point of the reference trajectory that is closest to the car
    err = ref(:,1:2) - carPose(k,1:2);
    for i=1:length(err(:,1)),
        norm_err(i) = norm(err(i,:));
    end
    [err_min, min_idx] = min(norm_err); % Minimize eucleadean norm
    closestPoint = ref(min_idx,:); % Closest point in the reference trajectory
    
    % Look ahead
    if min_idx + look_ahead_idx > npt_ref
        lookAheadPose = ref(end,:);
    else
        lookAheadPose = ref(min_idx + look_ahead_idx,:);
    end
    %plot(q_look_ahead(1),q_look_ahead(2), 'g+');
    err_dist_la= norm(lookAheadPose(1:2) - carPose(k,1:2));
    err_phi_la = lookAheadPose(3) - carPose(k,3);
    
    % Reduction of the angle error to the smallest interval
    if err_phi_la > pi
        err_phi_la = err_phi_la - 2*pi;
    elseif err_phi_la < -pi
        err_phi_la = err_phi_la + 2*pi;
    end
    % checks that the car is not trying to go back - if that's the case stop
    % the simulation as something is likely to be going wrong
    turn_back_flag = (abs(err_phi_la)>pi/2);
    % checks which side, relative to the ref trajectory, the car is
    cp = cross([err(min_idx,1:2),0],[cos(lookAheadPose(3)),sin(lookAheadPose(3)),0]);
    % if cp(3)>0 the car is on the left side
    % otherwise it's on the right side
    
    % Compute controller outputs, then feed them to the robot
    % v(k) = 0.1; % alternative 1 - a bit slow
    v(k) = 1 - tanh(Kv*err_dist_la); % alternative 2 - faster - needs additional tweaking
    ws(k) = -Kl*sign(cp(3))*err_dist_la + Ks*err_phi_la;
    
     % limitation on the steering velocity - as it happens in a real car
     %if abs(ws(k))>pi/8
     %   ws(k) = sign(ws(k))*pi/8;
     %end
    
    
    % Simulates the car using the model of the theory classes
    carPose(k+1,1) = carPose(k,1) + h*cos(carPose(k,3))*v(k);
    carPose(k+1,2) = carPose(k,2) + h*sin(carPose(k,3))*v(k);
    carPose(k+1,3) = carPose(k,3) + h*tan(carPose(k,4))/L*v(k);
    carPose(k+1,4) = carPose(k,4) + h*ws(k);
    
    %Get current pose of the robot
    currPose = carPose(k+1,:);
    
    % steering angle limitation - as it happens in a real car
    if currPose(4) > pi/4
        currPose(4) = pi/4;
    elseif currPose(4) < -pi/4
        currPose(4) = -pi/4;
    end
    
    k = k + 1;
    if mod(k,1000)==0
        disp(['k= ',num2str(k), ' look ahead idx ', num2str(min_idx), ' omega_s ', num2str(ws(k-1))]);
        plot(carPose(k,1),carPose(k,2),'b+')
        drawnow
    end
    
end


figure(1)
plot(carPose(:,1),carPose(:,2),'b')
title('trajectories (car - blue, reference - red)')
xlabel('x');
ylabel('y');

figure(2)
plot(carPose(:,3))
title('car orientation')
xlabel(['time x ', num2str(h)]);
ylabel('\theta (rad)');

figure(3)
plot(ws)
hold on
plot(v);
hold off
title('velocities (steering - blue, linear - red)')
xlabel(['time x ', num2str(h)]);
ylabel('\omega_s (rad/s), v (m/s)');

if turn_back_flag
    disp('simulation stopped - car was trying to go back')
    beep; beep
end

