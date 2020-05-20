function [G, x_array, y_array,T] = generate_graph(map)
% GENERATE_GRAPH  Computes the graph for a given map
%   [G, x_array, y_array] = generate_graph(map)
%   
%   Inputs: 
%           map - image of a map in array form (map=imread(.))
%   Returns: 
%           G - graph with nodes and edges
%           x_array - x coordinates associated with the nodes of G
%           y_array - y coordinates associated with the nodes of G
%           T - resized map from which the graph was built

T = imresize(map, [512, 512]);

S = qtdecomp(T, 0, [1 16]);

[i, j, v] = find(S);

B = [];
for x = 1:size(i)
    if T(i(x), j(x)) == 255 && v(x) > 4
        center = ceil(v(x)/2)-1;
        A = [i(x) j(x) v(x) center i(x)+center j(x)+center];
        B = [B; A];
    end
end

nodes = [];
for x = 1:size(B, 1)
    border_right = B(x, 2)+B(x, 3);
    border_down = B(x, 1)+B(x, 3);
    for y = 1:size(B,1)
        border_right_c = B(y, 2)+B(y, 3);
        border_down_c = B(y, 1)+B(y, 3);
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

node1_names = {};
node2_names = {};
for x = 1:size(nodes, 1)
    node1_names{x} = strcat(num2str(B(nodes(x, 1), 5)), '_', num2str(B(nodes(x, 1), 6)));
    node2_names{x} = strcat(num2str(B(nodes(x, 2), 5)), '_', num2str(B(nodes(x, 2), 6)));
end

G = graph(node1_names, node2_names, nodes(:, 3));

x_array=[];
y_array=[];
for x = 1:size(G.Nodes, 1)
    name = cell2mat(G.Nodes(x, 1).Name);
    splited = cellfun(@str2num, split(name, '_'));
    x_array = [x_array; splited(1)];
    y_array = [y_array; splited(2)];
end

end