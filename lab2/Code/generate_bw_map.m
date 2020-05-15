load('map_obstacles.mat');

img = uint8(ones(572, 276))*255;

for i=1:length(objs)
    object_array = [int16(objs{i}.x)' int16(objs{i}.y)'];
    object_array = reshape(object_array.', [1 2*length(object_array)]);
    img = insertShape(img, 'FilledPolygon', object_array, 'Color', 'black');
end

img_gray = rgb2gray(img);
imshow(img_gray);
