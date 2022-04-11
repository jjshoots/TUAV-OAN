% map2mat;

END_GOAL = [140 10];
% END_GOAL = [130 30];
TARGET = END_GOAL;

OBSERVED_SET = OBSERVED_SET;
OBSTACLE = obs;

INITIAL = [10.5 22.5];
% INITIAL = [70.5 90.5];
POSITION = INITIAL;

axis equal;
hold on;
xlabel('x-axis, x (m)','Color','black');
ylabel('y-axis, y (m)','Color','black');
scatter(OBSTACLE(:, 1), OBSTACLE(:, 2), 'k.');
plot(END_GOAL(1), END_GOAL(2), 'gd'); % greencolour diamond for target

% % form NODES set
% OBSERVED_SET = obs;
% NODES = nodes(OBSERVED_SET, ZVAR_MAX_X, ZVAR_MAX_Y);
% % perform A* search
% PATH = heck_yes_a_star(NODES, POSITION, END_GOAL, OBSERVED_SET, ZVAR_PROX, ZVAR_MAX_X, ZVAR_MAX_Y);
% % split and merge path
% WAYPOINT = split_merge(PATH);
% % new target and delete target from waypoint list

WAYPOINT = [140 10; 105 75; 20 65];

TARGET = WAYPOINT(1, :);
WAYPOINT(1, :) = [];




