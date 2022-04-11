for i = 1:size(vel_log,1)
     vel_dat(i) = norm(vel_log(i, :));
end

time = ZVAR_TSTEP*(0:counter);
accel_dat = diff(vel_dat(:))./diff(time(:));
accel_dat = [0; accel_dat];
jerk_dat = diff(accel_dat(:))./diff(time(:));
jerk_dat(1) = [];

abs_jerk_dat = abs(jerk_dat);

TOTAL_TIME = ZVAR_TSTEP * counter;
MAX_ARB;
NUM_A_STAR;
MAX_JERK = max(abs_jerk_dat);
AVE_JERK = mean(abs_jerk_dat);