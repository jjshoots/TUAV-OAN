function display_engine_modded(OBSTACLE, POSITION, MAX_X, MAX_Y)

plot(POSITION(1), POSITION(2), 'b.'); % blue O for position
hold on;
axis([1 MAX_X+1 1 MAX_Y+1]);
xlabel('x-axis, x (m)','Color','black');
ylabel('y-axis, y (m)','Color','black');

end