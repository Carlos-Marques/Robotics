%
% trajectory generation test using cubic splines and plot the vehicle 
% 
%

clear all
h = 0.01;

%%%%%%%%%%%%%%%%%%%%%%%%
% dimensions of the car
%
car_length = 3.332;
car_width = 1.508;
car_wheelbase = 2.2;
car_length_out_wheelbase = car_length - car_wheelbase;
% assume that the length out-of-wheelbase is identical at front and back of
% the car
a1 = car_length_out_wheelbase / 2;
%  car polygon constructed ccw
car_polygon = [ -a1, -car_width/2;
                car_wheelbase + a1, -car_width/2;
                car_wheelbase + a1, car_width/2;
                -1, car_width/2 ];

%%%%%%%%%%%%%%%%%%%%%%%%
% define the reference trajectory
figure(1)
% clf

imshow(imread('ist_gmaps.png'));
hold on

x_scale = 0.18107;
disp(['xx scale factor ', num2str(x_scale), ' meters/pixel']);


y_scale = 0.21394;
disp(['yy scale factor ', num2str(y_scale), ' meters/pixel']);

disp('use the mouse to input the world frame origin');
% [xx_org, yy_org, button] = ginput(1);
xx_org = 235;
yy_org = 258;
disp(['world frame origin at image coordinates ', num2str(xx_org), ' ', num2str(yy_org)]);

% draw the car at 0,0,0,0 configuration
plot(car_polygon(:,1)/x_scale+xx_org, car_polygon(:,2)/y_scale+yy_org)


%%% starting of the trajectory generation reference stuff

disp('use the mouse to input via points for the reference trajectory');
disp('--button 3-- to end the input');
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
time = [0:h:npt-1];
xx = fnval(csinterp_x, time);
yy = fnval(csinterp_y, time);
plot(xx,yy)

% draw the car at every 10 points of the trajectory
tp = length(xx);
sp = round(tp / 10);
for k=1:sp:tp,
    if k+1<=tp
        theta_car = atan2(yy(k+1)-yy(k), xx(k+1)-xx(k));
    else
        theta_car = atan2(yy(k)-yy(k-1), xx(k)-xx(k-1));
    end
    for p=1:4,
        rot_car_polygon(p,:) = ([cos(theta_car), -sin(theta_car); sin(theta_car), cos(theta_car)]*car_polygon(p,:)')';
    end
    plot(rot_car_polygon(:,1)/x_scale+xx(k), rot_car_polygon(:,2)/y_scale+yy(k))
end

