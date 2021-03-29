function [G,x_graph,y_graph,u]=simulation(map_original,map_size,objs,e1,e2,e3,e4,axis,time_field,energy_field,G,x_graph,y_graph)

    % Dilate objects in the map. This way the genenrated path never touches
    % the objects.
    map = generate_bw_map(map_size,objs);
    se = strel('rectangle',[10 10]);
    map_eroded = imerode(map, se);
    if isempty(G)
        [G,x_graph,y_graph,~] = generate_graph(map_eroded);
    end

    % show chosen objects overlayed on map
    T = map_original;
    for i=1:length(objs)
        object_array = [int16(objs{i}.x)' int16(objs{i}.y)'];
        object_array = reshape(object_array.', [1 2*length(object_array)]);
        T = insertShape(T, 'FilledPolygon', object_array, 'Color', 'black');
    end
    imshow(T);
    hold on;
    
    % draw events on map
    for i=1:length(e1)
        d = imread('stop_sign_image.png');
        image(d, 'XData', [e1{i}.x-10 e1{i}.x+10], 'YData', [e1{i}.y-10 e1{i}.y+10]);
    end
    
    for i=1:length(e2)
        d = imread('speed_limit_image.png');
        image(d, 'XData', [e2{i}.x-10 e2{i}.x+10], 'YData', [e2{i}.y-10 e2{i}.y+10]);
    end
    
    for i=1:length(e3)
        d = imread('pedestrian_crossing_image.png');
        image(d, 'XData', [e3{i}.x-10 e3{i}.x+10], 'YData', [e3{i}.y-10 e3{i}.y+10]);
    end
    
    for i=1:length(e4)
        d = imread('person_image.png');
        image(d, 'XData', [e4{i}.x-10 e4{i}.x+10], 'YData', [e4{i}.y-10 e4{i}.y+10]);
    end
    
    % choose the path
    button = 1;
    k = 1;
    while button==1
        [x(k), y(k), button] = ginput(1);
        plot(x(k), y(k),'r+');
        plot(x(k), y(k),'r+','Parent',axis);
        k = k + 1;
    end
    drawnow;
    
    % Close the image after choosing the path
    figHandles = findobj('type', 'figure', '-not', 'figure', 'figure1');
    close(figHandles);    
    % switch coordinates because graph is in xy, while image is in yx
    x_aux = x_graph;
    x_graph = y_graph;
    y_graph = x_aux;
    
    % Get the nodes closest to the desired path
    for i=1:length(x)
        error_graph = (x_graph-x(i)).^2 + (y_graph-y(i)).^2;
        [~, min_error_graph_idx] = min(error_graph);
        x_graph_min(i) = x_graph(min_error_graph_idx);
        y_graph_min(i) = y_graph(min_error_graph_idx);
    end
    
    % Car specs
    car_length = 3.332; %m
    car_width = 1.508; %m
    L = 2.2; %m
    Lf = (car_length - L)/2; %m
    Lb = Lf; %m
    weight = 810; %Kg
    w_radius = 0.256; %m
    
    car_polygon = [ -Lb, -car_width/2;
                    L + Lf, -car_width/2;
                    L + Lf, car_width/2;
                    -Lb, car_width/2 ];
    car_polygon = polyshape(car_polygon(:,1),car_polygon(:,2));
    
    % Main loop for simulation
    hi = 0.01;
    NSim = 300000000;
    goalRadius = 1.5;
    lookAhead = 10;
    E_budget = 100000000;
    
    timer = 0;
    e1_done = 0;
    e2_done = 0;
    e3_done = 0;
    e4_done = 0;
    
    for n=1:length(y_graph_min)-1
        
        flag_e1 = 0;
        flag_e2 = 0;
        flag_e3 = 0;
        flag_e4 = 0;
        
        % generate path between current and next nodes
        node1 = strcat(num2str(y_graph_min(n)), '_',num2str(x_graph_min(n)));
        node2 = strcat(num2str(y_graph_min(n+1)), '_',num2str(x_graph_min(n+1)));
        
        [path, ~] = G.shortestpath(node1, node2);
        
        % get coordinates associated with path
        x_path=[];
        y_path=[];
        for i = 1:size(path, 2)
            name = cell2mat(path(i));
            splited = cellfun(@str2num, split(name, '_'));
            x_path = [x_path; splited(1)];
            y_path = [y_path; splited(2)];
        end
        
        path_coords = [x_path y_path];
        
        for i=1:length(path_coords)
            % check if close to stop sign event
            for j=1:length(e1)
                if sqrt((path_coords(i,2)-e1{j}.x)^2+(path_coords(i,1)-e1{j}.y)^2)<10 && flag_e1==0 && e1_done~=j
                    flag_e1 = i;
                    e1_done = j; 
                end
            end
            % check if close to speed limit event
            for j=1:length(e2)
                if sqrt((path_coords(i,2)-e2{j}.x)^2+(path_coords(i,1)-e2{j}.y)^2)<10 && flag_e2==0 && e2_done~=j
                    flag_e2 = i;
                    e2_done = j;
                end
            end
            % check if close to pedestrian crossing sign event
            for j=1:length(e3)
                if sqrt((path_coords(i,2)-e3{j}.x)^2+(path_coords(i,1)-e3{j}.y)^2)<10 && flag_e3==0 && e3_done~=j
                    flag_e3 = i;
                    e3_done = j;
                end
            end
            % check if close and time for pedestrian crossing
            for j=1:length(e4)
                if sqrt((path_coords(i,2)-e4{j}.x)^2+(path_coords(i,1)-e4{j}.y)^2)<10 && abs(timer-e4{j}.start_time)<200 && flag_e4==0 && e4_done~=j 
                    flag_e4 = i;
                    e4_done = j;
                end
            end
        end
        
        [flags, flags_idx] = sort([flag_e1 flag_e2 flag_e3 flag_e4]);
        
        display(path_coords)
        display(flags)
        
        flags = [flags length(path_coords)];
        flags_idx = [flags_idx 420];
        
        % Move to first non zero point in the path
        if min(nonzeros(flags))~=1
            sub_path = path_coords(1:min(nonzeros(flags)),:);
            [~,trajectory] = genTrajectory(sub_path,hi);
            plot(trajectory(:,2),trajectory(:,1),'Parent',axis);
            [TBFlag,t,carPose,u,e,E_remaining] = lowlevelController(hi,NSim,goalRadius,lookAhead,trajectory,timer,car_polygon,E_budget,energy_field,axis,time_field);
            timer = timer+t;
        end
        
        for i=1:4
            if flags(i)==0
                continue;
            end
            % stop sign event
            if flags_idx(i)==1
                for tt = 1:20
                    time_field.Value = timer+tt;
                    pause(0.1);
                end
                timer = timer+tt;
                if flags(i)~=flags(i+1)
                    sub_path = path_coords(flags(i):flags(i+1),:);
                    [~,trajectory] = genTrajectory(sub_path,hi);
                    plot(trajectory(:,2),trajectory(:,1),'Parent',axis);
                    [TBFlag,t,carPose,u,e,E_remaining] = lowlevelController(hi,NSim,goalRadius,lookAhead,trajectory,timer,car_polygon,E_budget,energy_field,axis,time_field);
                    timer = timer+t;
                end
            end
            % speed limit event
            if flags_idx(i)==2
                if flags(i)~=flags(i+1)
                    sub_path = path_coords(flags(i):flags(i+1),:);
                    [~,trajectory] = genTrajectory(sub_path,hi);
                    plot(trajectory(:,2),trajectory(:,1),'Parent',axis);
                    [TBFlag,t,carPose,u,e,E_remaining] = lowlevelController(hi,NSim,goalRadius,lookAhead,trajectory,timer,car_polygon,E_budget,energy_field,axis,time_field);
                    timer = timer+t;
                end
            end 
            % Pedestrian crossing sign event
            if flags_idx(i)==3
                if flags(i)~=flags(i+1)
                    sub_path = path_coords(flags(i):flags(i+1),:);
                    [~,trajectory] = genTrajectory(sub_path,hi);
                    plot(trajectory(:,2),trajectory(:,1),'Parent',axis);
                    [TBFlag,t,carPose,u,e,E_remaining] = lowlevelController(hi,NSim,goalRadius,lookAhead,trajectory,timer,car_polygon,E_budget,energy_field,axis,time_field);
                    timer = timer+t;
                end
            end
            if flags_idx(i)==4
                for tt = 1:e4{j}.duration*10
                    time_field.Value = timer+tt;
                    pause(0.1);
                end
                timer = timer+tt;
                if flags(i)~=flags(i+1)
                    sub_path = path_coords(flags(i):flags(i+1),:);
                    [~,trajectory] = genTrajectory(sub_path,hi);
                    plot(trajectory(:,2),trajectory(:,1),'Parent',axis);
                    [TBFlag,t,carPose,u,e,E_remaining] = lowlevelController(hi,NSim,goalRadius,lookAhead,trajectory,timer,car_polygon,E_budget,energy_field,axis,time_field);
                    timer = timer+t;
                end
            end
        end
        
        E_budget = E_remaining;
    
        if E_remaining <= 0
            msgbox('You ran out of energy!');
            break;
        end
        
        % generate the trajectory for the path (aka sample the path)
        %[~,trajectory] = genTrajectory(path_coords,hi);
       
        
        % plot the trajectory yx for consistency with image
        %plot(trajectory(:,2),trajectory(:,1));
        %plot(trajectory(:,2),trajectory(:,1),'Parent',axis);
        %[TBFlag,t,carPose,u,e] = lowlevelController(hi,NSim,goalRadius,lookAhead,trajectory,timer,car_polygon,axis,time_field);
        %timer = timer+t;
    end
    
    x_g = x_graph;
    y_g = y_graph;
    save('map_obstacles.mat','objs','G','x_g','y_g'); 
    
end