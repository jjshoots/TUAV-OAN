function new_observed_set = observed_set(OBSERVABLES, OBSERVED_SET)
    new_observed_set = OBSERVED_SET;
    for i = 1:size(OBSERVABLES, 1)
        if ~sift(OBSERVABLES(i, :), OBSERVED_SET)
            new_observed_set = [new_observed_set; OBSERVABLES(i, :)];
        end
    end
end