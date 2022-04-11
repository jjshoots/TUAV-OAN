close all;
x_val = 1;
y_val = 1;
axis([1 51 1 51]);
hold on;

TARGET = [10 10];
INITIAL = [45 45];
CONCERN = [27 32];
OBSTACLE = CONCERN;

% Draw the target on screen
plot(TARGET(1), TARGET(2), 'gd'); % greencolour diamond 
text(TARGET(1) + .5, TARGET(2) + .5, 'Target');

plot(INITIAL(1), INITIAL(2), 'bo');

plot(27, 32, 'ro'); % red colour O

for i = 1:5:51
    for j = 1:5:51
        POSITION = [i, j];
        CONCERN = OBSTACLE;

        temp_O2T = TARGET - CONCERN;
        temp_P2O = CONCERN - POSITION;
        a1 = atan2(temp_O2T(1), temp_O2T(2));
        a2 = atan2(temp_P2O(1), temp_P2O(2));
        if a1 < 0
            a1 = a1 + 2*pi;
        end
        
        if a2 < 0
            a2 = a2 + 2*pi;
        end
        angle = a1 - a2;
        
        if 0 < angle && angle < pi/2
            HEADING = [temp_P2O(2) -temp_P2O(1)];
        elseif -pi/2 < angle && angle < 0
            HEADING = [-temp_P2O(2) temp_P2O(1)];
        else
            HEADING = TARGET - POSITION;
        end
        
        HEADING = 4* HEADING / norm(HEADING);
        
        quiver(i, j, HEADING(1), HEADING(2), 'color', [0.5 0.5 1], 'MaxHeadSize', 10);
%         txt = num2str(a1);
%         text(i, j, txt);
        
    end
end

set(gca,'xtick',[])
set(gca,'xticklabel',[])

set(gca,'ytick',[])
set(gca,'yticklabel',[])
