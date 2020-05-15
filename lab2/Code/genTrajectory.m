function [time,T] = genTrajectory(lims,path,hi)
 %input: Limits of the graph [xmin xmax ymin ymax]
 %       Set of points: path = [x(1) y(1); x(2) y(2); ... x(k) y(k)]
 %       Time interval: hi - this controls the time interval of the ref traj
 %output: npt - number of points in the trajectory. including initial and
 %final
 %        time - array containing the time instants of the trajectory
 %        T - array containing the desired pose of the robot in each instant
 
%  Grid limits
 xmin = lims(1);
 xmax = lims(2);
 ymin = lims(3);
 ymax = lims(4);
 
 x = path(:,1);
 y = path(:,2);
 
% Trajectory information
npt = length(x); %number of via points, including initial and final
nvia = [0:1:npt-1];
csinterp_x = csapi(nvia,x);
csinterp_y = csapi(nvia,y);
time = [0:hi:npt-1];
xx = fnval(csinterp_x, time);
yy = fnval(csinterp_y, time);

if(min(xx) < xmin) 
    error('Error.Points exceed grid limits - xmin');
elseif(max(xx) > xmax)
    error('Error.Points exceed grid limits - xmax');
elseif(min(yy) < ymin)
    error('Error.Points exceed grid limits - ymin');
elseif(max(yy) > ymax)
    error('Error.Points exceed grid limits - ymax');
end
 
% Compute angle of the tangent to the reference trajectory
for k=1:length(xx)-1,
    theta_tr(k) = atan2(yy(k+1)-yy(k), xx(k+1)-xx(k));
end
theta_tr(k+1) = theta_tr(k);

% Return the trajectory
T = [xx', yy', theta_tr'];   % reference trajectory 

end

