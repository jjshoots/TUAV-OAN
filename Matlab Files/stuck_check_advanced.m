function [STUCK_CHECK] = stuck_check_advanced(PATH, OBSERVED_SET, PROX)

    STUCK_CHECK = 0;

    if size(PATH, 1) == 0
        STUCK_CHECK = 1;
    else
        ismem = ismember(PATH, OBSERVED_SET, 'rows');
        if not(all(not(ismem)))
            idx = find(ismem==1);
            STUCK_CHECK = near_obstacles(PATH(idx(1), :), OBSERVED_SET, PROX) > PROX;
        end
    end
    
end