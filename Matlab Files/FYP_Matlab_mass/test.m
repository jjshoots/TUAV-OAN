%clearvars -except prev;
clc;
close all;
% variables; % initialize variables and logs
% thesis_var;

% pull map from saves or rerun
% 0 for reerun, 1 for pull
PULL_OR_RERUN = 1;
[END_GOAL, OBSTACLE, INITIAL] = interactive(ZVAR_MAX_X, ZVAR_MAX_Y, PULL_OR_RERUN);
TARGET = END_GOAL;

% initialize initial position
POSITION = INITIAL;

display_engine(TARGET, OBSTACLE, POSITION, 5.*HEADING, 5.*[0 0], POSITION, POSITION, OBSERVED_SET, ZVAR_MAX_X, ZVAR_MAX_Y, ZVAR_OBS_RAD);

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
    [ARB, VELOCITY, ACCELERATION, JERK, SNAP, CRACKLE] = dynamics(ACCE_GAIN, ZVAR_TSTEP, VELOCITY, ACCELERATION, JERK, SNAP, HEADING, TARGET, CONCERN, POSITION, ZVAR_MAX_VELO, ZVAR_PROX, AVOIDING);
    MAX_ARB = max_ARB(ARB, MAX_ARB);
    
    if MAX_ARB > 2.5
        DNF = 1;
        break;
    end
    
    % move the drone to next position based on velocity every tstep seconds
    POSITION = move(POSITION, VELOCITY, ZVAR_TSTEP);
    
    % find average velocity over INTERVAL seconds
    [STUCK_CHECK, MOVING_WINDOW] = stuck_check(VELOCITY, MOVING_WINDOW, ZVAR_THRESHOLD, ZVAR_TSTEP, ZVAR_GET_STUCK_INTERVAL);
    % Check for whether we are stuck
    if STUCK_CHECK
        'Hello to those who notice me'
        % form NODES set
        NODES = nodes(OBSERVED_SET, ZVAR_MAX_X, ZVAR_MAX_Y);
        % perform A* search
        if ~sift(round(POSITION), NODES)
            DNF = 1;
            break;
        end
            
        PATH = heck_yes_a_star(NODES, POSITION, END_GOAL, OBSERVED_SET, ZVAR_PROX);
        % split and merge path
        WAYPOINT = split_merge(PATH);
        scatter(WAYPOINT(:, 1), WAYPOINT(:, 2), 'ko', 'filled');
        % reset velocity average moving window
        MOVING_WINDOW = [];
        % new target and delete target from waypoint list
        TARGET = WAYPOINT(1, :);
        WAYPOINT(1, :) = [];
        NUM_A_STAR = NUM_A_STAR + 1;
        if(NUM_A_STAR > 20)
            DNF = 1;
            break;
        end
    end
    
    % store dynamics
    pos_log = [pos_log; POSITION];
    vel_log = [vel_log; VELOCITY];
    accel_log = [accel_log; ACCELERATION];
    jerk_log = [jerk_log; JERK];
    snap_log = [snap_log; SNAP];
    crackle_log = [crackle_log; CRACKLE];
    heading_log = [heading_log atan2(HEADING(2), HEADING(1))];
    yaw_log = [yaw_log atan2(VELOCITY(2), VELOCITY(1))];
    
    %display stuff, once every 10 loops
    if rem(counter, 2000000) == 0
        display_engine(TARGET, OBSTACLE, POSITION, 5.*HEADING, 5.*LOOK_HEADING, CONCERN1, CONCERN2, OBSERVED_SET, ZVAR_MAX_X, ZVAR_MAX_Y, ZVAR_OBS_RAD);
    end
    
    %check for end, end if drone to target is less than 1 unit
     if pdist([TARGET(1) TARGET(2); POSITION(1) POSITION(2)], 'euclidean') < ZVAR_WRT
         if size(WAYPOINT, 1) >= 1
             TARGET = WAYPOINT(1, :);
             WAYPOINT(1, :) = [];
         else
             break;
         end
     end
     
     %for nice simulation
%      pause(0.001);
     
end
display_debug;



