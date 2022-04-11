function [new_ARB, new_vel, new_accel, new_jerk, new_snap, new_crackle] = dynamics(ACCE_GAIN, TSTEP, VELOCITY, ACCELERATION, JERK, SNAP, HEADING, TARGET, CONCERN, POSITION, MAX_VELO, PROX, AVOIDING)
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
    
    new_ARB = 0;
    if temp_length < PROX
        new_ARB = abs(temp_length - PROX);
    end
    
    
    temp_vel = temp_speed .* HEADING;           % target stuff
    temp_accel = temp_vel - VELOCITY;
    temp_jerk = temp_accel - ACCELERATION;
    temp_snap = temp_jerk - JERK;
    temp_crackle = temp_snap - SNAP;
        
    new_crackle = temp_crackle;
    new_snap = SNAP + 1 * temp_crackle;
    new_jerk = JERK + 1 * temp_snap;
    new_accel = ACCELERATION + 1 * new_jerk;
    new_vel = VELOCITY + 0.25 * new_accel;
    
    new_vel = VELOCITY + ACCE_GAIN * temp_accel * TSTEP;
    
    %FIX THIS
end