function WAYPOINT = reconstruct_path(CAME_FROM, CURRENT, START)
    % highest waypoint is current
    WAYPOINT = [CURRENT(1:2)];
    %index of current in came_from
    idx = locate(CURRENT(1:2), CAME_FROM(:, 1:2));
    %if current observed point is not starting point
    while CURRENT(1) ~= START(1) || CURRENT(2) ~= START(2)
        %index of current in came_from
        idx = locate(CURRENT(1:2), CAME_FROM(:, 1:2));
        CURRENT(1:2) = CAME_FROM(idx, 3:4);
        %add current observed point to list of waypoints
        WAYPOINT = [CURRENT(1:2); WAYPOINT];
%         plot(CURRENT(1), CURRENT(2), 'mx');
    end
end