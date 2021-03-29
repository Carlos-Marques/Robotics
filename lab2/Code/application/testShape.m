    
    clear;
    close all
    
    load map_obstacles.mat
    
    map_original = imresize(imread('ist_map_detail.png'), [512 512]);
    map_size = [512 512];

    % Dilate objects in the map. This way the genenrated path never touches
    % the objects.
    map = generate_bw_map(map_size,objs);
    se = strel('rectangle',[5 5]);
    map_eroded = imerode(map, se);
    [G,x_graph,y_graph,~] = generate_graph(map_eroded);

    % show chosen objects overlayed on map
    T = map_original;
    
    for i=1:length(objs)
        object_array = [int16(objs{i}.x)' int16(objs{i}.y)'];
        object_array = reshape(object_array.', [1 2*length(object_array)]);
        T = insertShape(T, 'FilledPolygon', object_array, 'Color', 'black');
    end
    imshow(T);
    hold on;
    
    %Make conversion of the car polygon
    %Base on the reference axis drawn on the map   
    %yaxis
    pt1 = [200 57];
    pt2 = [232 207];
    distance_im_y = norm(pt2-pt1);
    distance_meters_y = 40; %meters
    scale_factor_y = distance_im_y/distance_meters_y;
    
    %xaxis
    pt3 = [228 205];
    pt4 = [321 201];
    distance_im_x = norm(pt4-pt3);
    distance_meters_x = 10; %meters
    scale_factor_x = distance_im_x/distance_meters_x;
    
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
    
    %scale car_polygon and rotate
    theta = pi/6;
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    %Rotation before scale
    car_polygon_rot = (R*car_polygon')';
    car_polygon_scaled = car_polygon_rot*[scale_factor_x 0;0 scale_factor_y];
    %Scale before rotation
    car_polygon_scaled_1 = car_polygon*[scale_factor_x 0;0 scale_factor_y];
    car_polygon_rot_1 = (R*car_polygon_scaled_1')';
    plot(car_polygon_scaled(:,1)+130,car_polygon_scaled(:,2)+250,'LineWidth',1.5);
    plot(car_polygon_rot_1(:,1)+130,car_polygon_rot_1(:,2)+250,'LineWidth',1.5);
    %rectangle('Position',[130 250 car_width*scale_factor_x (car_length)*scale_factor_y],'LineWidth',1.5)
    