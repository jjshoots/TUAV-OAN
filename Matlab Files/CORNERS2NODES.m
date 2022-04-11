function NODES = CORNERS2NODES(CORNERS, POSITION, END_GOAL)
    
    NODES = [POSITION];
    CORNERS = [CORNERS; END_GOAL];

    if size(CORNERS, 1) == 0
        NODES = NaN;
    end
end