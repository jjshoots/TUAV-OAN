function [new_vel, new_accel, new_jerk, new_snap, new_crackle] = dynamics(TSTEP, VELOCITY, ACCELERATION, JERK, SNAP, HEADING, TARGET, CONCERN, POSITION, MAX_VELO, PROX, AVOIDING)
    % compute maximum allowable speed
    temp_speed = MAX_VELO;
    temp_vec1 = CONCERN - POSITION;
    temp_vec2 = TARGET - POSITION;
    temp_length = norm(temp_vec1);
    
    if all(CONCERN) ~= 0  && ~AVOIDING
        if dot(temp_vec1, temp_vec2) > 0
            temp_speed = MAX_VELO/(1+exp(2*(-temp_length + PROX)));
        end
    end
    %end max speed computation
    
    
    temp_vel = temp_speed .* HEADING;           % target stuff
    temp_accel = temp_vel - VELOCITY;
    temp_jerk = temp_accel - ACCELERATION;
    temp_snap = temp_jerk - JERK;
    temp_crackle = temp_snap - SNAP;
        
    new_crackle = temp_crackle * TSTEP;
    new_snap = SNAP + 5 * temp_crackle * TSTEP;
    new_jerk = JERK + 5 * temp_snap * TSTEP;
    new_accel = ACCELERATION + 5 * temp_jerk * TSTEP;
    new_vel = VELOCITY + 1 * new_accel * TSTEP;
    
%     new_vel = VELOCITY + 5 * temp_accel * TSTEP;
    
    %FIX THIS
end