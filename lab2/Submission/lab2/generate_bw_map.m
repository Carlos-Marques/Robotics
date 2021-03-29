function [gray_map] = generate_bw_map(map_size,objs)

img = uint8(ones(map_size(1), map_size(2)))*255;

for i=1:length(objs)
    object_array = [int16(objs{i}.x)' int16(objs{i}.y)'];
    object_array = reshape(object_array.', [1 2*length(object_array)]);
    img = insertShape(img, 'FilledPolygon', object_array, 'Color', 'black');
end

gray_map = rgb2gray(img);

end
