function OBSERVABLES = observe(OBSTACLE, POSITION, OBS_RAD)
    OBSERVABLES = [0 0];
    n = 0; %counter for observables
    for i = 1:size(OBSTACLE, 1)
        if norm(OBSTACLE(i, :) - POSITION) <= OBS_RAD
            n = n + 1;
            OBSERVABLES(n, :) = OBSTACLE(i, :);
        end
    end
end