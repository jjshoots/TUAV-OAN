% note G = distance cost
% note H = heuristic cost

steps = [1 0; -1 0; 0 1; 0 -1];
wrong_direction = 0;

focus = floor(POSITION);
targ = floor(TARGET);
no_pass = focus;
CLOSED_LIST = focus;
current_G = 1;

while focus(1) ~= targ(1) && focus(2) ~= targ(2)
    % placeholder for open list
    OPEN_LIST = [focus 99999 99999 99999]; % x y G H F, note F = G+H
    for i = 1:4
        %look at adjacent tiles
        temp_test = focus + steps(i, :);
        %if adjacent tiles exist within nodes (not an observed obstacle OR
        %inaccessible path)
        if sift(temp_test, NODES) && i ~= wrong_direction && ~sift(temp_test, no_pass)
            % compute cost to get there based on previous cost
            G = current_G + 1;
            % compute heuristic
            H = norm(temp_test - targ);
            % sum cost and heuristic
            F = G + H;
            % add adjacent tile and costs to open list
            OPEN_LIST = [OPEN_LIST; temp_test G H F];
        end
    end
    % check that the open list actually has potential paths, this means
    % that it is not stuck in a dead end
    if size(OPEN_LIST, 1) > 1
        % identify tile in open list with smallest total cost
        [throw, idx] = min(OPEN_LIST(:, 5));
        wrong_direction = idx - 1;
        % make new smallest total cost tile the new focus
        focus = [OPEN_LIST(idx, 1) OPEN_LIST(idx, 2)];
        
        temp_size = size(CLOSED_LIST, 1);
        if temp_size > 1
            
        else
            temp_size = 2;
        end
        % check that the new path is not traversed one step before
        if ~all(focus == CLOSED_LIST(temp_size-1, :))
            % add smallest total cost tile to closed list
            CLOSED_LIST = [CLOSED_LIST; OPEN_LIST(idx, 1) OPEN_LIST(idx, 2)];
            current_G = OPEN_LIST(idx, 5);
            waypoints = CLOSED_LIST;
        else
            % if stuck in dead end or path has been travelled before,
            % move back one step, and add current step to
            % inaccessible list, ie: nodes
            CLOSED_LIST = CLOSED_LIST(1:temp_size-1, :);
            no_pass = [no_pass; OPEN_LIST(1, 1) OPEN_LIST(1, 2)];
        end
    end
end

plot(TARGET(1), TARGET(2), 'gd'); % greencolour diamond for target
plot(POSITION(1), POSITION(2), 'gd'); % greencolour diamond for target
scatter(CLOSED_LIST(:, 1), CLOSED_LIST(:, 2), 'mx'); % red O for obstacles