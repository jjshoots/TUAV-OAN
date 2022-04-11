function [DISTANCES, ABS_DIST] = distance(OBSERVABLES, POSITION)
    if all(OBSERVABLES) ~= 0
        DISTANCES(:, 1) = OBSERVABLES(:, 1) - POSITION(1);
        DISTANCES(:, 2) = OBSERVABLES(:, 2) - POSITION(2);
    else
        DISTANCES = [0 0];
    end
    
    for i = 1:size(DISTANCES, 1)
        ABS_DIST(i) = norm(DISTANCES(i, :));
    end
end