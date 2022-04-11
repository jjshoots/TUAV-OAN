clear all;
clc;
close all;

% extract and reformat data
spa_data_raw = csvread('A:\Users\Tai\Desktop\FYP_Matlab\the_data\data\sparse.csv');
two_data_raw = csvread('A:\Users\Tai\Desktop\FYP_Matlab\the_data\data\twowall.csv');
cor_data_raw = csvread('A:\Users\Tai\Desktop\FYP_Matlab\the_data\data\corridor.csv');
mix_data_raw = csvread('A:\Users\Tai\Desktop\FYP_Matlab\the_data\data\mixed.csv');

temp_size = size(spa_data_raw, 1);

spa_data = [];
two_data = [];
cor_data = [];
mix_data = [];

for i = 1:temp_size
    if spa_data_raw(i, 7) ~= -1
        spa_data = [spa_data; spa_data_raw(i, :)];
    end
    if two_data_raw(i, 7) ~= -1
        two_data = [two_data; two_data_raw(i, :)];
    end
    if cor_data_raw(i, 7) ~= -1
        cor_data = [cor_data; cor_data_raw(i, :)];
    end
    if mix_data_raw(i, 7) ~= -1
        mix_data = [mix_data; mix_data_raw(i, :)];
    end
end
% end data extraction and reformating

% start plotting stuff
plot_stuff(spa_data, 1);
plot_stuff(two_data, 2);
plot_stuff(cor_data, 3);
plot_stuff(mix_data, 4);

% [spa_rho, spa_p] = anova_pearson(spa_data);
% [two_rho, two_p] = anova_pearson(two_data);
% [cor_rho, cor_p] = anova_pearson(cor_data);
% [mix_rho, mix_p] = anova_pearson(mix_data);
% 
% blank = [0 0 0 0 0 0];
% rho_data = [spa_rho; blank; two_rho; blank; cor_rho; blank; mix_rho];
% p_data = [spa_p; blank; two_p; blank; cor_p; blank; mix_p];
% 
% dlmwrite('A:\Users\Tai\Desktop\FYP_Matlab\the_data\rho_data.csv', rho_data, 'precision', '%4f');
% dlmwrite('A:\Users\Tai\Desktop\FYP_Matlab\the_data\p_data.csv', p_data, 'precision', '%.4f');













