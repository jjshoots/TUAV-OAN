time = ZVAR_TSTEP*(0:counter);
AAA_VAR = 1;

%% DISPLAY VELOCITY PLOT
AAA_VAR = AAA_VAR + 1;
figure(AAA_VAR);
plot(time, vel_log(:, 1), 'b-', time, vel_log(:,2), 'r-');
title('VELOCITY PLOT');
xlabel('time, t (seconds)');
ylabel('velocity, V (m/s)');

%% DISPLAY ACCELERATION PLOT
AAA_VAR = AAA_VAR + 1;
figure(AAA_VAR);
plot(time, accel_log(:, 1), 'b-', time, accel_log(:,2), 'r-');
title('ACCELERATION PLOT');
xlabel('time, t (seconds)');
ylabel('accleration, A (m^2/s)');

%% DISPLAY YAW PLOT
% AAA_VAR = AAA_VAR + 1;
% figure(AAA_VAR);
% plot(time, heading_log, 'g-', time, yaw_log, 'b-');
% title('YAW PLOT');

%% DISPLAY A STAR NODES
% AAA_VAR = AAA_VAR + 1;
% figure(AAA_VAR);
% axis([1 ZVAR_MAX_X+1 1 ZVAR_MAX_Y+1]);
% hold on;
% NODES = nodes(OBSERVED_SET, TARGET, POSITION, ZVAR_MAX_X, ZVAR_MAX_Y);
% scatter(NODES(:, 1), NODES(:, 2));

%% DISPLAY PREVIOUS PATH
% AAA_VAR = AAA_VAR + 1;
% figure(AAA_VAR);
% scatter(prev(1:10:size(prev, 1), 1), prev(1:10:size(prev, 1), 2), 'go');
