function [STUCK_CHECK, new_mw] = stuck_check(VELOCITY, MOVING_WINDOW, THRESHOLD, TSTEP, GET_STUCK_INTERVAL)

    new_mw = MOVING_WINDOW;
    STUCK_CHECK = 0;
    
    % check whether window is filled
    if size(new_mw, 1) < GET_STUCK_INTERVAL/TSTEP
        new_mw = [new_mw; VELOCITY];
    else
        %delete earliest element and add new element
        new_mw(1, :) = [];
        new_mw = [new_mw; VELOCITY];
        
        %calculate average velocity
        VELOCITY_AVERAGE = [mean(new_mw(:, 1)), mean(new_mw(:, 2))];
        if norm(VELOCITY_AVERAGE) < THRESHOLD
            STUCK_CHECK = 1;
        end
    end
end