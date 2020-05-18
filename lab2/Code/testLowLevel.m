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
close;

% Get trajectory 
[time, ref] = genTrajectory(lims,path,hi);

h = 0.01;
nsim = 30000;
goalRadius = 0.5;
lookAhead = 10;

% Low level controller
[fig, flag, t, carPose, u, e] = lowlevelController(h,nsim,goalRadius,lookAhead,lims,path,ref);

v = u(1,:);
ws = u(2,:);

e_dis = e(1,:);
e_ang = e(2,:);

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


if flag
    disp('simulation stopped - car was trying to go back')
    beep; beep
end

