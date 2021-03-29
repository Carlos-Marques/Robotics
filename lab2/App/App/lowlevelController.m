function [TBFlag,t,carPose,u,e,E_remaining,distanceCovered] = lowlevelController(h,NSim,goalRadius,lookAhead,path,time,car_polygon,E_budget, ...
          energy_field,axis,time_field,velocity_field)
%INPUT
%
%
%
%
%
%OUTPUT
L = 2.2; %m
weight = 810; %Kg

xx = path(:,1);  
yy = path(:,2); 
%Initial position and orientation of the car
car_x = normrnd(xx(1), 0.1);
car_y = normrnd(yy(1), 0.1);
car_x(2) = xx(30);
car_y(2) = yy(30);
h1=plot(car_y, car_x,'b','Parent',axis);
%h2=plot(car_y(1),car_x(1),'bo');
drawnow
car_t = atan2(car_y(2)-car_y(1), car_x(2)-car_x(1)); %initial orientation of the car

%Initialization parameters
k = 1; % sampling instant
t = time;
initialPose = [car_x(1), car_y(1), car_t, 0]; % initial configuration for the car
carPose(1,:) = initialPose;
distanceToGoal = norm(carPose(1,1:2)-path(end,1:2));
TBFlag = 0;
distanceCovered = 0;

% Limitations
maxPhi = pi/4; %Steering angle maximum value (absolute)
maxVelSteer = pi/8; %Maximum steering velocity

% Controller parameters - Tune if neeeded
%Baseline
%Kl = 10;
%Ks = 50;
%Kv = 3;
Kl = 5;
Ks = 250;
Kv = 0.03;

P0 = 5; %Change this value, random
E_remaining = E_budget;

while(distanceToGoal > goalRadius && k < NSim && TBFlag == 0 && E_remaining > 0)
    
    t = time + k*h;
    
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
            display(v(k))
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
    
    %Calculate distance covered so far
    distanceCovered = distanceCovered + norm(currPose(1:2)-carPose(k,1:2));
    
    % steering angle limitation - as it happens in a real car
    if abs(currPose(4)) > maxPhi
        carPose(k+1,4) = sign(currPose(4))*maxPhi;
    end
    
    % Re-compute distance to target
    distanceToGoal = norm(currPose(1:2)-path(end,1:2));
    
    if mod(k,1000)==0
        delete(h1);
        disp(['k= ',num2str(k), ' look ahead idx ', num2str(min_idx), ' omega_s ', num2str(ws(k))])
        %h1=plot(carPose(k,2),carPose(k,1),'b+','Parent',axis);
        CarPoly = drawCar(carPose);
        h1=plot(CarPoly(:,2)+carPose(k,2),CarPoly(:,1)+carPose(k,1),'Parent',axis,'LineWidth',1.5);
        %car_polygon = rotate(car_polygon, rad2deg(carPose(k,3)));
        %car_polygon = translate((car_polygon), carPose(k,2),carPose(k,1));
        %h1 = plot(translate((car_polygon), carPose(k,2),carPose(k,1)),'Parent',axis);
        energy_field.Value=dE_budget(k);
        time_field.Value = t;
        velocity_field.Value = v(k)*50; 
        drawnow
    end
    
    k = k + 1;
    
end

%Controller outputs
u = [v;ws];

%Look ahead errors
e = [e_dis;e_ang];

end

