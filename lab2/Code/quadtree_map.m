I = rgb2gray(imread('ist_map_detail.png'));

S = qtdecomp(I,.05);
blocks = repmat(uint8(0),size(S));

for dim = [512 256 128 64 32 16 8 4 2 1];    
  numblocks = length(find(S==dim));    
  if (numblocks > 0)        
    values = repmat(uint8(1),[dim dim numblocks]);
    values(2:dim,2:dim,:) = 0;
    blocks = qtsetblk(blocks,S,dim,values);
  end
end

blocks(end,1:end) = 1;
blocks(1:end,end) = 1;

imshow(I)

figure
imshow(blocks,[])

%%
rotI = rgb2gray(imread('ist_map_detail.png'));
BW = edge(rotI,'log');
imshow(BW);

%% Gon?alo

%%
rotI = rgb2gray(imread('ist_map_detail.png'));
bin = imbinarize(rotI,'adaptive','Sensitivity',0.6);
skeleton = bwmorph(bin,'remove');
imshow(bin);
figure;
imshow(skeleton);

%%
rotI = rgb2gray(imread('ist_map_detail.png'));
rotI = mat2gray(rotI);
bw = rotI < 0.9;
imshow(bw);
grid = robotics.BinaryOccupancyGrid(bw);
show(grid)



