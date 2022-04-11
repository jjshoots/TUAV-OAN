clearvars -except obs;
clear all;
clc;
close all;
variables; % initialize variables and logs

% pull map from saves or rerun
% 0 for reerun, 1 for pull
PULL_OR_RERUN = 0;
DISPLAY_DEBUG = 0;
DISPLAY_COMPR = 1;

[END_GOAL, OBSTACLE, INITIAL] = interactive(ZVAR_MAX_X, ZVAR_MAX_Y, PULL_OR_RERUN);
TARGET = END_GOAL;
POSITION = INITIAL;
% benchmarking;

% map2mat;
% scatter(obs(:, 1), obs(:, 2), 'k.');
% hold on;
% INITIAL = [10 90];
% END_GOAL = [140 10];
% TARGET = END_GOAL;
% OBSTACLE = obs;
% POSITION = INITIAL;

if(DISPLAY_COMPR)
    display_engine(TARGET, OBSTACLE, POSITION, 5.*HEADING, 5.*[0 0], POSITION, POSITION, OBSERVED_SET, ZVAR_MAX_X, ZVAR_MAX_Y, ZVAR_OBS_RAD);
else
    display_engine_modded(OBSTACLE, POSITION, ZVAR_MAX_X, ZVAR_MAX_Y);
end

scatter(INITIAL(1), INITIAL(2), 'bd', 'filled');
text(INITIAL(1) + 1, INITIAL(2) - 3, 'START');
scatter(TARGET(1), TARGET(2), 'gd', 'filled');
text(TARGET(1) + 2, TARGET(2) + 3, 'END');

tic


while 1
    counter = counter + 1;
    
    % pick out observable obstacles
    OBSERVABLES = observe(OBSTACLE, POSITION, ZVAR_OBS_RAD);
    OBSERVED_SET = observed_set(OBSERVABLES, OBSERVED_SET);
    % identify distances of observable obstacles to drone
    [DISTANCES, ABS_DIST] = distance(OBSERVABLES, POSITION);    
    % find closest 2 obstacles
    [CONCERN1, CONCERN2, MAG1, MAG2] = concern(ABS_DIST, OBSERVABLES);
    
    % form heading vector from only concerns
    [HEADING1, AVOIDING1] = heading(CONCERN1, POSITION, TARGET, ZVAR_PROX);
    [HEADING2, AVOIDING2] = heading(CONCERN2, POSITION, TARGET, ZVAR_PROX);
    HEADING = normalize_heading(HEADING1, HEADING2, MAG1, MAG2);
    AVOIDING = AVOIDING1 || AVOIDING2;
    LOOK_HEADING = VELOCITY / norm(VELOCITY);
    CONCERN = CONCERN1;
    
    % compute velocity acceleration jerk snap crackle
    [VELOCITY, ACCELERATION, JERK, SNAP, CRACKLE] = dynamics(ZVAR_TSTEP, VELOCITY, ACCELERATION, JERK, SNAP, HEADING, TARGET, CONCERN, POSITION, ZVAR_MAX_VELO, ZVAR_PROX, AVOIDING);
    % move the drone to next position based on velocity every tstep seconds
    POSITION = move(POSITION, VELOCITY, ZVAR_TSTEP);
    
    % find average velocity over INTERVAL seconds
    [STUCK_CHECK1, MOVING_WINDOW] = stuck_check(VELOCITY, MOVING_WINDOW, ZVAR_THRESHOLD, ZVAR_TSTEP, ZVAR_GET_STUCK_INTERVAL);
    
    % advenced level stuck check
    if((size(OBSERVED_SET, 1) - set_size) > ZVAR_OBS_RAD)
        STUCK_CHECK2 = stuck_check_advanced(PATH, OBSERVED_SET, ZVAR_PROX);
        set_size = size(OBSERVED_SET, 1);
    end
    
    % Check for whether we are stuck
    % if stuck do search

    if  STUCK_CHECK1 || STUCK_CHECK2
        'performing A*'
        % form NODES set
        NODES = nodes(OBSERVED_SET, ZVAR_MAX_X, ZVAR_MAX_Y);
        
        % upgraded NODES set, IN PROGRESS
%         IM = COORD2IM(OBSTACLE, ZVAR_MAX_X, ZVAR_MAX_Y);
%         CORNERS = corner(IM);
        
        % perform A* search
        PATH = heck_yes_a_star(NODES, POSITION, END_GOAL, OBSERVED_SET, ZVAR_PROX, ZVAR_MAX_X, ZVAR_MAX_Y);
        % split and merge path
        WAYPOINT = split_merge(PATH);
%         scatter(WAYPOINT(:, 1), WAYPOINT(:, 2), 'ko', 'filled');
        % reset velocity average moving window
        MOVING_WINDOW = [];
        % new target and delete target from waypoint list
        TARGET = WAYPOINT(1, :);
        WAYPOINT(1, :) = [];
        STUCK_CHECK2 = 0;
    end
    
    if(DISPLAY_DEBUG)
        % store dynamics
        pos_log = [pos_log; POSITION];
        vel_log = [vel_log; VELOCITY];
        accel_log = [accel_log; ACCELERATION];
        jerk_log = [jerk_log; JERK];
        snap_log = [snap_log; SNAP];
        crackle_log = [crackle_log; CRACKLE];
        heading_log = [heading_log atan2(HEADING(2), HEADING(1))];
        yaw_log = [yaw_log atan2(VELOCITY(2), VELOCITY(1))];
    end
    
    %display stuff, once every 10 loops
    if rem(counter, 2) == 0
        if(DISPLAY_COMPR)
            display_engine(TARGET, OBSTACLE, POSITION, 5.*HEADING, 5.*[0 0], POSITION, POSITION, OBSERVED_SET, ZVAR_MAX_X, ZVAR_MAX_Y, ZVAR_OBS_RAD);
        else
            display_engine_modded(OBSTACLE, POSITION, ZVAR_MAX_X, ZVAR_MAX_Y);
%             scatter(OBSERVED_SET(:, 1), OBSERVED_SET(:, 2), 'k.');
        end
    end
    
    %check for end, end if drone to target is less than 1 unit
    temp_dist = pdist([TARGET(1) TARGET(2); POSITION(1) POSITION(2)], 'euclidean');
    if  temp_dist < ZVAR_PROX + 1
        if size(WAYPOINT, 1) >= 1
        TARGET = WAYPOINT(1, :);
        WAYPOINT(1, :) = [];
        else
            break;
        end
    end

    %for nice simulation
    pause(0.001);
     
end
sec = toc;

if(DISPLAY_DEBUG)
	display_debug;
end



