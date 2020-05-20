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
 
% SAVE PATHS OF VARIOUS TYPES!!! TO AVOID REPETITION
            
% Reference trajectory specification
figure(1)
clf
lims = [-10,10,-10,10];
hi = 0.01;
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
drawnow;
disp([ num2str(k-1), ' points to interpolate from '])

% Get trajectory 
[time, path] = genTrajectory(path,hi);
xx = path(:,1);  
yy = path(:,2); 
plot(xx,yy, 'r')

%Initial position and orientation of the car
car_x = normrnd(xx(1), 0.1);
car_y = normrnd(yy(1), 0.1);
car_x(2) = xx(30);
car_y(2) = yy(30);
plot(car_x, car_y,'b');
plot(car_x(1),car_y(1),'bo');
drawnow
car_t = atan2(car_y(2)-car_y(1), car_x(2)-car_x(1)); %initial orientation of the car

%Initialization parameters
k = 1; % sampling instant
t(1) = 0;
initialPose = [car_x(1), car_y(1), car_t, 0]; % initial configuration for the car
carPose(1,:) = initialPose;
distanceToGoal = norm(carPose(1,1:2)-path(end,1:2));
TBFlag = 0;

% Limitations
maxPhi = pi/4; %Steering angle maximum value (absolute)
maxVelSteer = pi/8; %Maximum steering velocity

% Controller parameters - Tune if neeeded
%Baseline
%Kl = 10;
%Ks = 50;
%Kv = 3;
Kl = 0.8;
Ks = 500;
Kv = 0.5;

%Energy control
P0 = 5; % Change this value

h = 0.01;
NSim = 30000;
goalRadius = 0.2;
lookAhead = 10;

E_budget = 5000;
E_remaining = E_budget;

while(distanceToGoal > goalRadius && k < NSim && TBFlag == 0 && E_remaining > 0)
    
    t(k) = k*h;
    
    % Get the point of the patherence trajectory that is closest to the car
    err = path(:,1:2) - carPose(k,1:2);
    for i=1:length(err(:,1)),
        norm_err(i) = norm(err(i,:));
    end
    [err_min, min_idx] = min(norm_err); % Minimize eucleadean norm
    closestPoint = path(min_idx,:); % Closest point in the patherence trajectory
    
    % Look ahead
    if min_idx + lookAhead > length(path)
        lookAheadPose = path(end,:);
    else
        lookAheadPose = path(min_idx + lookAhead,:);
    end
    %plot(q_look_ahead(1),q_look_ahead(2), 'g+');
    err_dist_la= norm(lookAheadPose(1:2) - carPose(k,1:2));
    err_theta_la = lookAheadPose(3) - carPose(k,3);
    e_dis(k) = err_dist_la;
    e_ang(k) = err_theta_la;
    
    % Reduction of the angle error to the smallest interval
    if err_theta_la > pi
        err_theta_la = err_theta_la - 2*pi;
    elseif err_theta_la < -pi
        err_theta_la = err_theta_la + 2*pi;
    end
    % checks that the car is not trying to go back - if that's the case stop
    % the simulation as something is likely to be going wrong
    TBFlag = (abs(err_theta_la)>pi/2);
    % checks which side, relative to the path trajectory, the car is
    cp = cross([err(min_idx,1:2),0],[cos(lookAheadPose(3)),sin(lookAheadPose(3)),0]);
    % if cp(3)>0 the car is on the left side
    % otherwise it's on the right side
    
    % Compute controller outputs, then feed them to the robot
    %v(k) = 0.1; % alternative 1 - a bit slow 
    v(k) = 1 - tanh(Kv*err_dist_la); % alternative 2 - faster - needs additional tweaking
    %v(k) = Kv*err_dist_la;
    ws(k) = -Kl*sign(cp(3))*err_dist_la + Ks*err_theta_la;
    
    %Energy consumption 
    if(k==1)
        dv(k) = v(k)/h; %acceleration
        dE(k) = (weight*dv(k)+P0)*v(k)*h; %energy spent on the movement
        E(k) = dE(k); % Energy spent so far
    else
        if(abs(v(k)) > abs(maxVelocity(k-1)))
            disp('You exceeded the maximum velocity!');
            v(k) = sign(v(k))*abs(maxVelocity(k-1)); %saturate velocity
        end
        dv(k) = (v(k)-v(k-1))/h; %acceleration
        dE(k) = abs((weight*dv(k)+P0)*v(k)*h); %energy spent on the movement
        E(k) = E(k-1)+dE(k); % Energy spent so far
    end
    
    %Remaining energy for the rest of the trajectory
    dE_budget(k) = E_budget-E(k);
    E_remaining = dE_budget(k);
    %Remaining energy per step
    E_step_rem(k) = dE_budget(k)/(NSim-k);
    %Maximum velocity 
    maxVelocity(k) = (P0/h)*E_step_rem(k);
    
    % limitation on the steering velocity - as it happens in a real car
    if abs(ws(k))>maxVelSteer
       ws(k) = sign(ws(k))*maxVelSteer;
    end
    
    % Simulates the car using the model of the theory classes
    carPose(k+1,1) = carPose(k,1) + h*cos(carPose(k,3))*v(k);
    carPose(k+1,2) = carPose(k,2) + h*sin(carPose(k,3))*v(k);
    carPose(k+1,3) = carPose(k,3) + h*tan(carPose(k,4))/L*v(k);
    carPose(k+1,4) = carPose(k,4) + h*ws(k);
    
    %Get current pose of the robot
    currPose = carPose(k+1,:);
    
    % steering angle limitation - as it happens in a real car
    if abs(currPose(4)) > maxPhi
        carPose(k+1,4) = sign(currPose(4))*maxPhi;
    end
    
    % Re-compute distance to target
    distanceToGoal = norm(currPose(1:2)-path(end,1:2));
    
    if mod(k,1000)==0
        %disp(['k= ',num2str(k), ' look ahead idx ', num2str(min_idx), ' omega_s ', num2str(ws(k-1))]);
        disp(['Energy remaining: ', num2str(dE_budget(k)), ' Maximum velocity: ', num2str(maxVelocity(k)),' Current velocity: ', num2str(v(k))]);
        plot(carPose(k,1),carPose(k,2),'b+')
        drawnow
    end
    
    k = k + 1;
    
end

figure(1)
plot(currPose(1),currPose(2),'go')
viscircles(path(end,1:2),goalRadius,'Color','k');
plot(carPose(:,1),carPose(:,2),'b')
title('Trajectories')
xlabel('x (m)');
ylabel('y (m)');


figure(2)
plot(carPose(:,3))
title('Car orientation')
xlabel('Time instant (Ts = 0.01s)');
ylabel('\theta (rad)');

figure(3)
yyaxis right
plot(t,ws);
ylabel('\omega_s (rad/s)');
hold on
yyaxis left
plot(t,v);
hold off
title('Controller outputs');
xlabel('Time (s)');
ylabel('v (m/s)');

figure(4)
yyaxis left
plot(t,e_dis);
ylabel('Error distance (m)');
hold on
yyaxis right
plot(t,e_ang);
title('Look ahead error');
xlabel('Time (s)');
ylabel('Error orientation (rad)');

figure(5)
yyaxis left
plot(t,E);
ylabel('Energy spent (J)');
hold on
yyaxis right
plot(t,dE_budget);
title('Energy consumption');
xlabel('Time (s)');
ylabel('Energy remaining (J)');


if(E_remaining <= 0)
    disp('You ran out of energy!!!')
end

if TBFlag
    disp('simulation stopped - car was trying to go back')
    beep; beep
end

