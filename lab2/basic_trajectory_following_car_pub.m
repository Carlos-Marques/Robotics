%
% *********** trajectory following car ************
% 
% doesn't work if the car starts too far behing the initial point of the
% reference trajectory - the structure of the steering control is not
% adequate
%
% test with cte linear velocity
% test with linear velocity dependent on the lateral error
%

clear all
hi = 0.01;   % this controls the time interval of the ref traj


% dimensions of the car - mostly used for the plotting of the car rectangle
% - if useful
%
car_length = 3.332;
car_width = 1.508;
L = 2.2;
car_wheelbase = L;
car_length_out_wheelbase = car_length - car_wheelbase;
% assume that the length out-of-wheelbase is identical at front and back of
% the car
a1 = car_length_out_wheelbase / 2;
%  car polygon constructed ccw
car_polygon = [ -a1, -car_width/2;
                car_wheelbase + a1, -car_width/2;
                car_wheelbase + a1, car_width/2;
                -1, car_width/2 ];


% reference trajectory specification
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


npt = length(x);        % number of via points, including initial and final
nvia = [0:1:npt-1];
csinterp_x = csapi(nvia,x);
csinterp_y = csapi(nvia,y);
time = [0:hi:npt-1];
xx = fnval(csinterp_x, time);
yy = fnval(csinterp_y, time);
plot(xx,yy, 'r')

% % draw the car at each 10 points of the trajectory
% tp = length(xx);
% sp = round(tp / 10);
% for k=1:sp:tp,
%     if k+1<=tp
%         theta_car = atan2(yy(k+1)-yy(k), xx(k+1)-xx(k));
%     else
%         theta_car = atan2(yy(k)-yy(k-1), xx(k)-xx(k-1));
%     end
%     for p=1:4,
%         rot_car_polygon(p,:) = ([cos(theta_car), -sin(theta_car); sin(theta_car), cos(theta_car)]*car_polygon(p,:)')';
%     end
%     plot(rot_car_polygon(:,1)/x_scale+xx(k), rot_car_polygon(:,2)/y_scale+yy(k))
% end

% the angle of the tangent to the reference trajectory
for k=1:length(xx)-1,
    tt(k) = atan2(yy(k+1)-yy(k), xx(k+1)-xx(k));
end
tt(k+1) = tt(k);

%%% now the control

%%% input the initial position and orientation for the car
disp('input the position and orientation of the car - 2 points')
[car_x, car_y] = ginput(2);
plot(car_x, car_y,'b');
plot(car_x(1),car_y(1),'bo');
drawnow

car_t = atan2(car_y(2)-car_y(1), car_x(2)-car_x(1));

% some initialization stuff
h = 0.01;                   % time step for each iteration (s)
k=1;
nsimul = 30000;             % max number of iterations of the simulation
q(1,:) = [car_x(1), car_y(1), car_t, 0];    % initial configuration for the car
q_ref = [xx', yy', tt'];    % reference trajectory 
look_ahead_idx = 10;        % number of points ahead of the point in the reference traj
                            % that is closest to the car used to "look ahead"
turn_back_flag = 0;
Kl = 5;
Ks = 25;
Kv = 2;

% main loop
while norm(q(k,1:2)-q_ref(end,1:2))>2 && ...
        k<nsimul && ...
        turn_back_flag==0,
    
    % get the point of the reference trajectory that is closest to the car
    err = q_ref(:,1:2) - q(k,1:2);
    for k1=1:length(err(:,1)),
        norm_err(k1) = norm(err(k1,:));
    end
    [norm_err_min, norm_err_min_idx] = min(norm_err);
    
    q_closest = q_ref(norm_err_min_idx,:);
    if norm_err_min_idx + look_ahead_idx>length(q_ref(:,1))
        q_look_ahead = q_ref(end,:);
    else
        q_look_ahead = q_ref(norm_err_min_idx+look_ahead_idx,:);
    end
%     plot(q_look_ahead(1),q_look_ahead(2), 'g+');
    l_look_ahead = norm(q_look_ahead(1:2) - q(k,1:2));
    error_t_look_ahead = q_look_ahead(3) - q(k,3);
    
    % reduction of the angle error to the smallest interval
    if error_t_look_ahead>pi
        error_t_look_ahead = error_t_look_ahead - 2*pi;
    elseif error_t_look_ahead<-pi
        error_t_look_ahead = error_t_look_ahead + 2*pi;
    end
    
    % checks that the car is not trying to go back - if that's the case stop
    % the simulation as something is likely to be going wrong
    turn_back_flag = (abs(error_t_look_ahead)>pi/2);
    
    % checks which side, relative to the ref trajectory, the car is
    cp = cross([err(norm_err_min_idx,1:2),0],[cos(q_look_ahead(3)),sin(q_look_ahead(3)),0]);
    % if cp(3)>0 the car is on the left side
    % otherwise it's on the right side
    
    % compute the controls
%     v(k) = 0.1;         % alternative 1 - a bit slow
    v(k) = 1 - tanh(Kv*l_look_ahead);      % alternative 2 - faster - needs additional tweaking

    ws(k) = -Kl*sign(cp(3))*l_look_ahead + Ks*error_t_look_ahead;

    
    % limitation on the steering velocity - as it happens in a real car
%     if abs(ws(k))>pi/8
%         ws(k) = sign(ws(k))*pi/8;
%     end
    
    % simulates the car using the model of the theory classes
    q(k+1,1) = q(k,1) + h*cos(q(k,3))*v(k);
    q(k+1,2) = q(k,2) + h*sin(q(k,3))*v(k);
    q(k+1,3) = q(k,3) + h*tan(q(k,4))/L*v(k);
    q(k+1,4) = q(k,4) + h*ws(k);
    
    % steering angle limitation - as it happens in a real car
    if q(k+1,4)>pi/4
        q(k+1,4) = pi/4;
    elseif q(k+1,4)<-pi/4
        q(k+1,4) = -pi/4;
    end
  
    k = k + 1;
    if mod(k,1000)==0
        disp(['k= ',num2str(k), ' look ahead idx ', num2str(norm_err_min_idx), ' omega_s ', num2str(ws(k-1))]);
        plot(q(k,1),q(k,2),'b+')
        drawnow
    end
end

figure(1)
plot(q(:,1),q(:,2),'b')
title('trajectories (car - blue, reference - red)')
xlabel('x');
ylabel('y');

figure(2)
plot(q(:,3))
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