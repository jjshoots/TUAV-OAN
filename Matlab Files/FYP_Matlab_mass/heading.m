function [HEADING, AVOIDING] = heading(CONCERN, POSITION, TARGET, PROX)
    if all(CONCERN) ~= 0
        % because of bad naming schemes, Concern = O
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
        
        if all(HEADING) == 0
            temp_P2T = TARGET - POSITION;
            HEADING = temp_P2T;
        end
        
        % hyper_param 
        % if proximity of activation = hyper param = x
        % then UAV will stop when x away from wall, ie: identifiable
        % obstacle
        
        temp_prox = norm(temp_P2O);
        if temp_prox < PROX
            temp_scalar = PROX / temp_prox;
        else
            temp_scalar = 0;
        end
        
        % normalize stuff to be unit length
        % mix in obstacle avoidance vector
        HEADING = HEADING / norm(HEADING);
        head_mix = -temp_P2O / norm(temp_P2O);
        HEADING = HEADING + temp_scalar * head_mix;
        HEADING = HEADING / norm(HEADING);
        AVOIDING = 1;
    else
        temp_P2T = TARGET - POSITION;
        HEADING = temp_P2T;
        HEADING = HEADING / norm(HEADING);
    end
    AVOIDING = 0;
end