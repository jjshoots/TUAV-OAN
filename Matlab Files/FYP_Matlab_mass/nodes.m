function NODES = nodes(OBSERVED_SET, MAX_X, MAX_Y)
    NODES = [0 0];
    for i = 1:MAX_X
        for j = 1:MAX_Y
            % create set of unoccupied / unobserved nodes, before
            % creating node, check that node does not exist within
            % observed set, current position, or target
            if ~sift([i j], OBSERVED_SET)
                NODES = [NODES; i j];
            end
        end
    end
end