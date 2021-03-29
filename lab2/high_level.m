% image = imread('scuffed_map.png');
% image_gs = rgb2gray(image);
% image_gs_r = imresize(image_gs, [512 512]);
% se = strel('rectangle',[15 15]);
% map_eroded = imerode(image_gs_r, se);
% imshow(image_gs_r);
% camroll(180);
% hold on;

load("scuffed_map.mat");
img_gray_r = imresize(img_gray, [512, 512]);
imshow(img_gray_r);
camroll(180);
hold on;
se = strel('rectangle',[15 15]);
map_eroded = imerode(img_gray_r, se);
T =map_eroded;       

S = qtdecomp(T, 0, [4 8]);

[i, j, v] = find(S);

% B = [];
% local_B = [];
% nodes = [];
% for x = 1:size(i)
%     if T(i(x), j(x)) == 255
%         fprintf("i = %i j = %i v = %i\n", i(x), j(x), v(x))
%         n = v(x)/8;
%         for row = 0:n-1
%             for column = 0:n-1
%                 center = ceil(8/2)-1;
%                 A = [i(x)+(row*8) j(x)+(column*8) 8 center i(x)+(row*8)+center j(x)+(column*8)+center];
%                 local_B = [local_B; A];
%                 disp(n);
%             end
%         end
%         for counter = 1:size(local_B,1)
%             weight = local_B(counter, 3)/2 + local_B(counter, 3)/2;
%             node = [counter+size(B, 1) counter+size(B,1) weight];
%             nodes = [nodes;node];
%         end
%         B = [B; local_B];
%         local_B = [];
%     end
% end

disp("HERE 0");
B = [];
for x = 1:size(i)
    if T(i(x), j(x)) == 255
        center = ceil(v(x)/2)-1;
        A = [i(x) j(x) v(x) center i(x)+center j(x)+center];
        B = [B; A];
    end
end

disp("HERE 1");
nodes = [];
for x = 1:size(B, 1)
    border_right = B(x, 2)+B(x, 3);
    border_down = B(x, 1)+B(x, 3);
    for y = x:size(B,1)
        border_right_c = B(y, 2)+B(y, 3);
        border_down_c = B(y, 1)+B(y, 3);
        %disp([border_right B(y, 2)])
        if border_right==B(y, 2) && border_down > B(y,1) && B(x,1) < border_down_c
            %fprintf("border right: %i %i\n", B(x, 3), B(y, 3))
            %fprintf("connect: %i %i to %i %i\n", B(x, 5), B(x, 6), B(y, 5), B(y, 6))
            weight = B(x, 3)/2 + B(y, 3)/2;
            node = [x y weight];
            nodes = [nodes;node];
        elseif border_down==B(y, 1) && border_right > B(y,2) && B(x,2) < border_right_c
            %fprintf("border down: %i %i\n", B(x, 3), B(y, 3))
            %fprintf("connect: %i %i to %i %i\n", B(x, 5), B(x, 6), B(y, 5), B(y, 6))
            weight = B(x, 3)/2 + B(y, 3)/2;
            node = [x y weight];
            nodes = [nodes;node];
        end
    end
end

disp("HERE 3");
node1_names = {};
node2_names = {};
for x = 1:size(nodes, 1)
    node1_names{x} = strcat(num2str(B(nodes(x, 1), 5)), '_', num2str(B(nodes(x, 1), 6)));
    node2_names{x} = strcat(num2str(B(nodes(x, 2), 5)), '_', num2str(B(nodes(x, 2), 6)));
end

G = graph(node1_names, node2_names, nodes(:, 3));

disp("HERE 4");
x_array=[];
y_array=[];
for x = 1:size(G.Nodes, 1)
    name = cell2mat(G.Nodes(x, 1).Name);
    splited = cellfun(@str2num, split(name, '_'));
    x_array = [x_array; splited(1)];
    y_array = [y_array; splited(2)];
end

disp("HERE 5");
p = plot(G,'XData',y_array,'YData',x_array,'EdgeLabel',G.Edges.Weight)

% [path, d] = G.shortestpath('20_492', '388_156');
% 
% highlight(p, path, 'EdgeColor','r');
% 
% x_array=[];
% y_array=[];
% for x = 1:size(path, 2)
%     name = cell2mat(path(x));
%     splited = cellfun(@str2num, split(name, '_'));
%     x_array = [x_array; splited(1)];
%     y_array = [y_array; splited(2)];
% end
% 
% h = 0.01;
% npt = length(x_array);        % number of via points, including initial and final
% nvia = [0:1:npt-1];
% csinterp_x = csapi(nvia,x_array);
% csinterp_y = csapi(nvia,y_array);
% time = [0:h:npt-1];
% xx = fnval(csinterp_x, time);
% yy = fnval(csinterp_y, time);
% plot(yy,xx)