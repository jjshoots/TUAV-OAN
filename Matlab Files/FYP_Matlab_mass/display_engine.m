function display_engine(TARGET, OBSTACLE, POSITION, HEADING, LOOK_HEADING, CONCERN1, CONCERN2, OBSERVED_SET, MAX_X, MAX_Y, OBS_RAD)

axis([1 MAX_X+1 1 MAX_Y+1]);
hold on;
xlabel('x-axis, x (m)','Color','black');
ylabel('y-axis, y (m)','Color','black');

if all(CONCERN1) ~= 0
    quiver(POSITION(1), POSITION(2), CONCERN1(1) - POSITION(1), CONCERN1(2)- POSITION(2), 'color', [0.4 0.4 0.4]);
end

if all(CONCERN2) ~= 0
    quiver(POSITION(1), POSITION(2), CONCERN2(1) - POSITION(1), CONCERN2(2)- POSITION(2), 'color', [0.9 0.9 0.9]);
end

plot(TARGET(1), TARGET(2), 'gd'); % greencolour diamond for target

scatter(OBSTACLE(:, 1), OBSTACLE(:, 2), 'ro'); % red O for obstacles
scatter(OBSERVED_SET(:, 1), OBSERVED_SET(:, 2), 'ro', 'filled'); % red O for observed obstacles

plot(POSITION(1), POSITION(2), 'bo'); % blue O for position

quiver(POSITION(1), POSITION(2), HEADING(1), HEADING(2), 'color', [.8 .8 1]);
quiver(POSITION(1), POSITION(2), LOOK_HEADING(1), LOOK_HEADING(2), 'color', [.3 .3 1]);

%viscircles(POSITION, OBS_RAD, 'LineStyle', '--', 'Color', 'b');

end