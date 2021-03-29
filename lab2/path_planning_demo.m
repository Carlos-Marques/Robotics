clear;

% Load map and create visibility graph
map = load('scuffed_map.mat');
map = map.img_gray;
se = strel('rectangle',[5 5]);
map_eroded = imerode(map, se);
[G,x_graph,y_graph,T] = generate_graph(map_eroded);

T = imresize(map, [512, 512]);
imshow(T);
hold on;


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

% Choose the path
disp('use the mouse to input via points for the reference trajectory - right button for the last point');
button = 1;
k = 1;
while button==1,
    [x(k), y(k), button] = ginput(1);
    plot(x(k), y(k),'r+')
    k = k + 1;
end
drawnow;

x_aux = x_graph;
x_graph = y_graph;
y_graph = x_aux;
% Get the nodes closest to the desired path
for i=1:length(x)
    error_graph = (x_graph-x(i)).^2 + (y_graph-y(i)).^2;
    [min_error_graph, min_error_graph_idx] = min(error_graph);
    x_graph_min(i) = x_graph(min_error_graph_idx);
    y_graph_min(i) = y_graph(min_error_graph_idx);
    plot(x_graph_min(i), y_graph_min(i),'o')
end

%Simulation parameters
hi = 0.01;
NSim = 30000;
goalRadius = 2;
lookAhead = 10;
E_budget = 1000000000;


for n=1:length(y_graph_min)-1
    node1 = strcat(num2str(y_graph_min(n)), '_',num2str(x_graph_min(n)));
    node2 = strcat(num2str(y_graph_min(n+1)), '_',num2str(x_graph_min(n+1)));

    [path, d] = G.shortestpath(node1, node2);

    x_path=[];
    y_path=[];
    for i = 1:size(path, 2)
        name = cell2mat(path(i));
        sp = split(name, '_');
        splited = str2num(char(sp));
        x_path = [x_path; splited(1)];
        y_path = [y_path; splited(2)];
        %name = cell2mat(path(i));
        %splited = cellfun(@str2num, split(name, '_'));
        %x_path = [x_path; splited(1)];
        %y_path = [y_path; splited(2)];
    end

    path_coords = [x_path y_path];

    [time,trajectory] = genTrajectory(path_coords,hi);

    plot(trajectory(:,2),trajectory(:,1));
    
    [TBFlag,t,carPose,u,e,E_remaining] = lowlevelController(hi,NSim,goalRadius,lookAhead,trajectory,E_budget);
    
    %Update energy budget
    E_budget = E_remaining;
    
    if E_remaining <= 0
        disp('You ran out of energy!');
        break;
    end
    
end
