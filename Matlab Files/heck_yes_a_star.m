function WAYPOINT = heck_yes_a_star(NODES, POSITION, TARGET, OBSERVED_SET, PROX, MAX_X, MAX_Y)
    zero_bar = zeros(size(NODES, 1), 1);
    Inf_bar = (ones(size(NODES, 1), 1).*Inf);
    NaN_bar = (ones(size(NODES, 1), 1).*NaN);
    steps = [1 0; -1 0; 0 1; 0 -1];
    WAYPOINT = [];

    START = floor(POSITION);
    END = floor(TARGET);
    CLOSED_SET = [];
    OPEN_SET = [START 0];
    CAME_FROM = [NODES NaN_bar NaN_bar];

    G_SCORE = [NODES Inf_bar];
    F_SCORE = [NODES Inf_bar];
    G_SCORE(locate(START, NODES), 3) = 0;
    F_SCORE(locate(START, NODES), 3) = norm(START - TARGET);

    while size(OPEN_SET, 1) > 0
        [~, idx_OPEN] = min(OPEN_SET(:, 3));
        idx_CIN = locate(OPEN_SET(idx_OPEN, 1:2), NODES);
        CURRENT = OPEN_SET(idx_OPEN, :);
%         plot(CURRENT(1), CURRENT(2), 'cx');
%         pause(0.0001);

        if CURRENT(1) == END(1) && CURRENT(2) == END(2)
            WAYPOINT = reconstruct_path(CAME_FROM, CURRENT, START);
            break;
        end

        %delete current from open set, the dumb way
        OPEN_SET(idx_OPEN, :) = [];
        CLOSED_SET = [CLOSED_SET; CURRENT];

        for i = 1:4
            % create a NEIGHBOUR
            NEIGHBOUR = [CURRENT(1) CURRENT(2)] + steps(i, :);
            
            % if NEIGHBOUR can be travelled on
            if NEIGHBOUR(1) > MAX_X || NEIGHBOUR(2) > MAX_Y || NEIGHBOUR(1) < 1 || NEIGHBOUR(2) < 1 || sift(NEIGHBOUR, OBSERVED_SET)
                continue;
            end
            
            % initialize neighbour
            NEIGHBOUR = [NEIGHBOUR 0];

            idx_NEIGHBOUR = locate(NEIGHBOUR(1:2), NODES(:, 1:2));

            % break if NEIGHBOUR is in CLOSED_SET,
            % ie: has been explored
            if sift(NEIGHBOUR(1:2), CLOSED_SET(:, 1:2))
                continue;
            end

            % if NEIGHBOUR is not part of OPEN_SET
            % add NEIGHBOUR to OPEN_SET
            if ~sift(NEIGHBOUR(1:2), OPEN_SET(:, 1:2))
                OPEN_SET = [NEIGHBOUR; OPEN_SET];
            end

            % tentative G_SCORE = OLD+1
            T_G_SCORE = G_SCORE(idx_CIN, 3) + 1;
            % if new G_SCORE is more than current G_SCORE
            % break, pointless to explore this path
            if T_G_SCORE >= G_SCORE(idx_NEIGHBOUR, 3)
                continue;
            end

            % if we made it this far, this path is best for now,
            % we record it
            CAME_FROM(idx_NEIGHBOUR, 3:4) = CURRENT(1:2);
            G_SCORE(idx_NEIGHBOUR, 3) = T_G_SCORE;
            heuristic = norm(NEIGHBOUR(1:2) - TARGET)*3 + 10*near_obstacles(NEIGHBOUR(1:2), OBSERVED_SET, PROX);
            F_SCORE(idx_NEIGHBOUR, 3) = T_G_SCORE + heuristic;

            idx_NIO = locate(NEIGHBOUR(1:2), OPEN_SET(:, 1:2));
            OPEN_SET(idx_NIO, 3) = F_SCORE(idx_NEIGHBOUR, 3);
        end
    end
end