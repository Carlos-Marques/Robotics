function simulation(map_original,map_size,objs,e1,e2,e3,e4)

    map = generate_bw_map(map_size,objs);
    se = strel('rectangle',[5 5]);
    map_eroded = imerode(map, se);
    [G,x_graph,y_graph,~] = generate_graph(map_eroded);

    T = map_original;
    for i=1:length(objs)
        object_array = [int16(objs{i}.x)' int16(objs{i}.y)'];
        object_array = reshape(object_array.', [1 2*length(object_array)]);
        T = insertShape(T, 'FilledPolygon', object_array, 'Color', 'black');
    end
    imshow(T);
    hold on;
    
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
    
    button = 1;
    k = 1;
    while button==1
        [x(k), y(k), button] = ginput(1);
        plot(x(k), y(k),'r+');
        k = k + 1;
    end
    drawnow;
    
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
    
    hi = 0.01;
    NSim = 30000;
    goalRadius = 1;
    lookAhead = 10;
    for n=1:length(y_graph_min)-1
        node1 = strcat(num2str(y_graph_min(n)), '_',num2str(x_graph_min(n)));
        node2 = strcat(num2str(y_graph_min(n+1)), '_',num2str(x_graph_min(n+1)));
        
        [path, ~] = G.shortestpath(node1, node2);
        
        x_path=[];
        y_path=[];
        for i = 1:size(path, 2)
            name = cell2mat(path(i));
            splited = cellfun(@str2num, split(name, '_'));
            x_path = [x_path; splited(1)];
            y_path = [y_path; splited(2)];
        end
        
        path_coords = [x_path y_path];
        
        [time,trajectory] = genTrajectory(path_coords,hi);
        
        plot(trajectory(:,2),trajectory(:,1));
        
        [TBFlag,t,carPose,u,e] = lowlevelController(hi,NSim,goalRadius,lookAhead,trajectory);
    end

end