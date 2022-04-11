function [TARGET, OBSTACLE, INITIAL] = interactive(MAX_X, MAX_Y, PULL_OR_RERUN)

    if PULL_OR_RERUN
        load('map_saves\initial.mat', 'INITIAL');
        load('map_saves\obstacle.mat', 'OBSTACLE');
        load('map_saves\target.mat', 'TARGET');
    else
        j=0;
        x_val = 1;
        y_val = 1;
        axis([1 MAX_X+1 1 MAX_Y+1]);
        hold on;

        % Interactibe UI for target
        xlabel('Please Select the Target using the Left Mouse button','Color','green');
        but = 0;
        while (but ~= 1) %Repeat until the Left button is not clicked
            [xval, yval, but] = ginput(1);
        end

        TARGET = [xval yval];

        % Draw the target on screen
        plot(TARGET(1), TARGET(2), 'gd'); % greencolour diamond 
        text(TARGET(1) + .5, TARGET(2) + .5, 'Target')
        pause(0.2);

        % Interactibe UI for obstacles
        xlabel('Please Select obstacle locations, right click to end ','Color','red');
        n = 0; % Number of Obsatcels
        while but == 1
            [xval, yval, but] = ginput(1);
            %floor to make it look nice
            xval = floor(xval);
            yval = floor(yval);
            n = n + 1; % Number of Obstacles plus one
            OBSTACLE(n, :) = [xval yval]; % Put on the closed list as well
            plot(OBSTACLE(n, 1), OBSTACLE(n, 2), 'k.'); % red colour O
        end

        % Interactibe UI for target location
        xlabel('Please Select the Vehicle initial position ','Color','blue');
        but = 0;
        while (but ~= 1) %Repeat until the Left button is not clicked
            [xval, yval, but] = ginput(1);
        end

        INITIAL = [xval yval];
        plot(INITIAL(1), INITIAL(2), 'bo'); %blue O

        xlabel(' ');
        
        % save stuff
        save('map_saves\initial.mat', 'INITIAL');
        save('map_saves\obstacle.mat', 'OBSTACLE');
        save('map_saves\target.mat', 'TARGET');
    end
    
end