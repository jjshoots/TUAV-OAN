function [CONCERN1, CONCERN2, MAG1, MAG2]  = concern(ABS_DIST, OBSERVABLES)
    temp_dist = ABS_DIST;   % temp storage abs_dist
    CONCERN1 = [0 0];
    CONCERN2 = [0 0];
    MAG1 = 0;
    MAG2 = 0;
    if all(OBSERVABLES) ~= 0
        % find the minimum
        [MAG1, i] = min(temp_dist);
        CONCERN1 = OBSERVABLES(i, :);
        temp_dist(i) = 10000;   % delete the distance from the temp storage
        if size(OBSERVABLES, 1) > 1
            % if more than 1 element, find the second minimum
            [MAG2, i] = min(temp_dist);
            CONCERN2 = OBSERVABLES(i, :);
        end
    end
end