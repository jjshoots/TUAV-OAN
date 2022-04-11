function IM = COORD2IM(DATA, MAX_X, MAX_Y)
    IM(MAX_X, MAX_Y) = 0;
    
    for i = 1:size(DATA, 1)
        if DATA(i, 1) && DATA(i, 2)
            IM(DATA(i, 1), DATA(i, 2)) = 1;
        end
    end
end