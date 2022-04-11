function WAYPOINT = split_merge(PATH)
    path_end_idx = size(PATH, 1);   % last index on paht
    path_beg_idx = 1;               % first index on path
    way_idx = 2;                    % number of index on waypoint where split is about to happen

    WAYPOINT = [PATH(1, :); PATH(end, :)];

    while 1
        % split path into 2
        temp_mid = floor((path_end_idx + path_beg_idx) / 2);
        temp_mid_point = PATH(temp_mid, :);

        % if split and merge point is already in WAYPOINTs list, we 
        % have reached optimality
        if sift(temp_mid_point, WAYPOINT)
            break;
        end

        WAYPOINT = [WAYPOINT; 0 0];     % create empty spot at end of waypoint list
        WAYPOINT = [WAYPOINT(1:way_idx-1, :); temp_mid_point; WAYPOINT(way_idx:end-1, :)];
        % shift waypoint list from way_idx to end, by one unit to the
        % right, and put in new waypoint in empty spot

        LINES_NUM = size(WAYPOINT, 1) - 1;
        M_C_Matrix = zeros(LINES_NUM, 2);
        ERROR_SUM = zeros(LINES_NUM, 1);

        idx_WOP_prev = 1; % index of waypoint on path, previous

        for i = 1:LINES_NUM
            % perform y = mx
            temp_del_x = WAYPOINT(i+1, 1) - WAYPOINT(i, 1);
            % perform check for delta x, if delta x = 0,
            % we reach NaN later, which is bad, so we just approximate inf
            if temp_del_x < 1
                temp_m = inf;               % inf gradient
                temp_c = WAYPOINT(i, 1);    % store only the x value
            else
                temp_m = (WAYPOINT(i+1, 2) - WAYPOINT(i, 2)) / temp_del_x;
                temp_c = WAYPOINT(i, 2) - temp_m * WAYPOINT(i, 1);
            end
            M_C_Matrix(i, :) = [temp_m temp_c];
            
            
            % locate line end location index on path for error SOS
            % computation
            idx_WOP = locate(WAYPOINT(i+1, :), PATH);
            for j = idx_WOP_prev:idx_WOP % form error SOS on the line that we are observing
                if M_C_Matrix(i, 1) == inf
                    SOS = abs(M_C_Matrix(i, 2) - PATH(j, 1)) / (idx_WOP - idx_WOP_prev);
                    ERROR_SUM(i) = ERROR_SUM(i) + SOS;
                else
                    temp_eY = M_C_Matrix(i, 1) * PATH(j, 1) + M_C_Matrix(i, 2);
                    SOS = abs(PATH(j, 2) - temp_eY) / (idx_WOP - idx_WOP_prev);
                    ERROR_SUM(i) = ERROR_SUM(i) + SOS;
                end
            end
            idx_WOP_prev = idx_WOP;
            % shift the line begin index on path by one for next computation
            % of error SOS
        end

        [err, idx] = max(ERROR_SUM);

        if err > 3 % if SOS mean error > 3, we can optimize further            
            way_idx = idx+1;
            path_end_idx = locate(WAYPOINT(way_idx, :), PATH);
            path_beg_idx = locate(WAYPOINT(way_idx-1, :), PATH);
        else
            break;
        end
    end
end