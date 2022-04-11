% note G = distance cost
% note H = heuristic cost

steps = [1 0; -1 0; 0 1; 0 -1];
no_pass = [0 0];

wrong_direction = 0;
focus = floor(POSITION);
targ = floor(TARGET);
CLOSED_LIST = [focus 0 0 0];
n = 1;  % counter for open list
current_G = 0;
OPEN_LIST = [];

while focus(1) ~= targ(1) && focus(2) ~= targ(2)
    for i = 1:4
        %open list for adjacent tiles at node n
        OPEN_LIST(i, :, n) = [focus+steps(i, :) 99999 99999 99999]; % x y G H F, note F = G+H
    end
    
    for i = 1:4
        %if adjacent tiles exist within nodes (not an observed obstacle OR
        %inaccessible path)
        if sift([OPEN_LIST(i, 1:2, n)], NODES) && ~sift([OPEN_LIST(i, 1:2, n)], no_pass) && i ~= wrong_direction
            % compute cost to get there based on previous cost
            G = current_G + 1;
            % compute heuristic
            H = norm(OPEN_LIST(i, 1:2, n) - targ);
            % sum cost and heuristic
            F = G + H;
            % add adjacent tile and costs to open list
            OPEN_LIST(i, 3:5, n) = [G H F];
        end
    end
    
    % identify tile in open list with smallest total cost
    [throw, idx] = min(OPEN_LIST(:, 5, n));
    
    if idx == 1
        wrong_direction = 2;
    elseif idx == 2
        wrong_direction = 1;
    elseif idx == 3
        wrong_direction = 4;
    elseif idx == 4
        wrong_direction = 3;
    end

    if throw ~= 99999
        % make new smallest total cost tile the new focus
        focus = [OPEN_LIST(idx, 1, n) OPEN_LIST(idx, 2, n)];
        % extend closed list with focus
        CLOSED_LIST = [CLOSED_LIST; OPEN_LIST(idx, 1:5, n)];
        % increase closed list length
        n = n + 1;
        %current cost
        current_G = current_G + 1;
    else
        % if open list is pointless, ie: no available paths
        % move back one point and solve from there
        no_pass = [no_pass; CLOSED_LIST(end, 1:2)];
        CLOSED_LIST = CLOSED_LIST(1:end-1, :);
        focus = CLOSED_LIST(end, 1:2);
        n = n + 1;
        current_G = current_G - 1;
    end
end

plot(TARGET(1), TARGET(2), 'gd'); % greencolour diamond for target
plot(POSITION(1), POSITION(2), 'gd'); % greencolour diamond for target
scatter(CLOSED_LIST(:, 1), CLOSED_LIST(:, 2), 'mx'); % red O for obstacles