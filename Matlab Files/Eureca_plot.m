clear all;
close all
clc;

ccA = [1.4434e+10; 1.0358e+10; 3.1187e+10; 3.4894e+10];
ccC = [4.9720e+09; 1.6714e+09; 8.2780e+09; 1.2785e+10];
ttA = [49; 39; 80; 55];
ttC = [56; 40; 79; 157];

figure(1);
bar([ccA ccC]);
legend([{'A*'} {'COAA*'}], 'Location', 'northwest');
title('Computation Cycle Number of A* and COAA* in the 4 maps ');
xlabel('Map ID');
ylabel('Computation Cycle Number, n');

figure(2);
bar([ttA ttC]);
legend([{'A*'} {'COAA*'}], 'Location', 'northwest');
title('Time to Completion of A* and COAA* in the 4 maps ');
xlabel('Map ID');
ylabel('Time to Completion, t (s)');


