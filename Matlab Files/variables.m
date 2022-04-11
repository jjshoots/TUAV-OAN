%% Define map size
ZVAR_MAX_X = 50;
ZVAR_MAX_Y = 50;

%% Drone observabnle radius
ZVAR_OBS_RAD = 10;

%% Drone D_proximity
ZVAR_PROX = 3;

%% Loop time
ZVAR_TSTEP = 0.1;

%% Dynamic Limits
ZVAR_MAX_VELO = 5;

%% INITIALIZE STUFF
% initial conditions
PATH = [];
WAYPOINT = [];
VELOCITY = [0 0];
ACCELERATION = [0 0];
JERK = [0 0];
SNAP = [0 0];
CRACKLE = [0 0];
HEADING = [0 0];    % direction of target velocity
YAW = [0 0];        % direction of actual velocity
if(exist('OBSERVED_SET', 'var') == 0)
    OBSERVED_SET = [0 0];
end
AVOIDING = 0;       % are we avoiding stuff?

%% LOGS
%initialize logs
pos_log = [0 0];
vel_log = [0 0];
accel_log = [0 0];
jerk_log = [0 0];
snap_log = [0 0];
crackle_log = [0 0];
heading_log = [0];
yaw_log = [0];
counter = 0;

%% get-stuck-hyper-detection-mega-engine
ZVAR_GET_STUCK_INTERVAL = 3; % moving window interval
ZVAR_THRESHOLD = 0.05;
MOVING_WINDOW = []; % moving window for velocity
STUCK_CHECK1 = 0;
STUCK_CHECK2 = 1;
set_size = 0;


