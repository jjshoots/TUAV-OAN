path_end_idx = size(PATH, 1);
path_beg_idx = 1;
way_idx = 2;

WAYPOINT = [PATH(1, :); PATH(end, :)];

while 1
    % split path into 2
    temp_mid = floor((path_end_idx + path_beg_idx) / 2);
    temp_mid_point = PATH(temp_mid, :);
    
    %if split and merge point is already in WAYPOINTs list, we 
    % have reached optimality
    if sift(temp_mid_point, WAYPOINT)
        scatter(WAYPOINT(:, 1), WAYPOINT(:, 2), 'ko');
        break;
    end
    
    WAYPOINT = [WAYPOINT; 0 0];
    WAYPOINT = [WAYPOINT(1:way_idx-1, :); temp_mid_point; WAYPOINT(way_idx:end-1, :)]

    LINES_NUM = size(WAYPOINT, 1) - 1;
    M_C_Matrix = zeros(LINES_NUM, 2);
    ERROR_SUM = zeros(LINES_NUM, 1);

    idx_WOP_prev = 1;

    for i = 1:LINES_NUM
        % perform y = mx
        temp_m = (WAYPOINT(i+1, 2) - WAYPOINT(i, 2)) / (WAYPOINT(i+1, 1) - WAYPOINT(i, 1));
        temp_c = WAYPOINT(i, 2) - temp_m * WAYPOINT(i, 1);
        M_C_Matrix(i, :) = [temp_m temp_c];

        %locate waypoint location index on path
        idx_WOP = locate(WAYPOINT(i+1, :), PATH);
        for j = idx_WOP_prev:idx_WOP
            temp_eY = M_C_Matrix(i, 1) * PATH(j, 1) + M_C_Matrix(i, 2);
            SOS = abs(PATH(j, 2) - temp_eY) / (idx_WOP - idx_WOP_prev);
            ERROR_SUM(i) = ERROR_SUM(i) + SOS ;
        end    
    end
    
    [err, idx] = max(ERROR_SUM);
    
    if err > 2
        way_idx = idx+1;
        path_end_idx = locate(WAYPOINT(way_idx, :), PATH);
        path_beg_idx = locate(WAYPOINT(way_idx-1, :), PATH);
    else
        scatter(WAYPOINT(:, 1), WAYPOINT(:, 2), 'ko');
        break;
    end
end