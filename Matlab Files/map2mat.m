[X,map]= imread('map_saves\eureca\4.jpg');
BW = im2bw(X,map,0.3);
obs = [];

scaler = 0.15;

for i = 1:size(BW, 1)
    for j = 1:size(BW, 2)
        if(~BW(i, j))
            coord = [round(j*scaler) round(size(BW, 1)*scaler)-round(i*scaler)+1];
            
            if(~sift(coord, obs))
                obs = [obs; coord];
            end
        end
    end
end

map = BW;