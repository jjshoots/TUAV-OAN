function HEADING_SUM = heading_sum(OBSERVABLES, POSITION, DISTANCES, TARGET, OBS_RAD)
    HEADING_SUM = [0 0];
    temp_length = 1;
    
    % pass through list of observables
    for i = 1:size(OBSERVABLES)
        % look at each observable one by one
        temp_hold = OBSERVABLES(i, :);
        
        if all(temp_hold) ~= 0
            % because of bad naming schemes, Concern = O
            temp_O2T = TARGET - temp_hold;
            temp_P2O = temp_hold - POSITION;
            angle = atan2(temp_O2T(1), temp_O2T(2)) - atan2(temp_P2O(1), temp_P2O(2));
            
            % find angle of position to obstacle relative to obstacle to
            % target and do 90 degree or just to head to target
            if angle < pi/2 && 0 < angle
                temp_heading = [temp_P2O(2) -temp_P2O(1)];
            elseif -pi/2 < angle && angle < 0
                temp_heading = [-temp_P2O(2) temp_P2O(1)];
            else
                temp_heading = 0;
            end

            if temp_heading == 0
                temp_P2T = TARGET - POSITION;
                temp_heading = temp_P2T;
            end
            
            % find how close position is to observed obstacle
            temp_length = pdist([DISTANCES(i, 1) DISTANCES(i, 2); 0 0], 'euclidean');
            
            temp_scalar = 0;
            if temp_scalar < OBS_RAD
                temp_scalar = temp_length / OBS_RAD;
            end
            
            % normalize stuff to be unit length
            % mix in obstacle avoidance vector
            temp_heading = temp_heading / norm(temp_heading);
            head_mix = -temp_P2O / norm(temp_P2O);
            temp_heading = temp_heading + temp_scalar * head_mix;
            temp_heading = temp_heading / norm(temp_heading);
        else
            temp_P2T = TARGET - POSITION;
            temp_heading = temp_P2T;
            temp_heading = temp_heading / norm(temp_heading);
        end
        
        HEADING_SUM = HEADING_SUM + temp_heading / temp_length;
    end
    
    HEADING_SUM = HEADING_SUM / norm(HEADING_SUM);
end