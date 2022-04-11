function new_pos = move(POSITION, VELOCITY, TSTEP)
    new_pos = POSITION + TSTEP .* VELOCITY;
end