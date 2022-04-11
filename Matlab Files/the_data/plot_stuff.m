function plot_stuff(data, label)
    % plot stuff
    figure(label);

    % fig 1 row 3
    subplot(5, 6, 1);
    scatter(data(:, 1), data(:, 7), 'x');
    grid on;
    y = ylabel('T_C', 'FontWeight', 'bold');
    set(get(gca,'YLabel'),'Rotation',0)
    set(y, 'Units', 'Normalized', 'Position', [-0.4, 0.5, 0]);
    subplot(5, 6, 2);
    scatter(data(:, 2), data(:, 7), 'x');
    grid on;
    subplot(5, 6, 3);
    scatter(data(:, 3), data(:, 7), 'x');
    grid on;
    subplot(5, 6, 4);
    scatter(data(:, 4), data(:, 7), 'x');
    grid on;
    subplot(5, 6, 5);
    scatter(data(:, 5), data(:, 7), 'x');
    grid on;
    subplot(5, 6, 6);
    scatter(data(:, 6), data(:, 7), 'x');
    grid on;

    % fig 1 row 2
    subplot(5, 6, 7);
    scatter(data(:, 1), data(:, 8), 'x');
    grid on;
    y = ylabel('D_P_A_B', 'FontWeight', 'bold');
    set(get(gca,'YLabel'),'Rotation',0)
    set(y, 'Units', 'Normalized', 'Position', [-0.4, 0.5, 0]);
    subplot(5, 6, 8);
    scatter(data(:, 2), data(:, 8), 'x');
    grid on;
    subplot(5, 6, 9);
    scatter(data(:, 3), data(:, 8), 'x');
    grid on;
    subplot(5, 6, 10);
    scatter(data(:, 4), data(:, 8), 'x');
    grid on;
    subplot(5, 6, 11);
    scatter(data(:, 5), data(:, 8), 'x');
    grid on;
    subplot(5, 6, 12);
    scatter(data(:, 6), data(:, 8), 'x');
    grid on;

    % fig 1 row 3
    subplot(5, 6, 13);
    scatter(data(:, 1), data(:, 9), 'x');
    grid on;
    y = ylabel('n_A_*', 'FontWeight', 'bold');
    set(get(gca,'YLabel'),'Rotation',0)
    set(y, 'Units', 'Normalized', 'Position', [-0.4, 0.5, 0]);
    subplot(5, 6, 14);
    scatter(data(:, 2), data(:, 9), 'x');
    grid on;
    subplot(5, 6, 15);
    scatter(data(:, 3), data(:, 9), 'x');
    grid on;
    subplot(5, 6, 16);
    scatter(data(:, 4), data(:, 9), 'x');
    grid on;
    subplot(5, 6, 17);
    scatter(data(:, 5), data(:, 9), 'x');
    grid on;
    subplot(5, 6, 18);
    scatter(data(:, 6), data(:, 9), 'x');
    grid on;

    % fig 1 row 4
    subplot(5, 6, 19);
    scatter(data(:, 1), data(:, 10), 'x');
    grid on;
    y = ylabel('J_m_a_x', 'FontWeight', 'bold');
    set(get(gca,'YLabel'),'Rotation',0)
    set(y, 'Units', 'Normalized', 'Position', [-0.4, 0.5, 0]);
    subplot(5, 6, 20);
    scatter(data(:, 2), data(:, 10), 'x');
    grid on;
    subplot(5, 6, 21);
    scatter(data(:, 3), data(:, 10), 'x');
    grid on;
    subplot(5, 6, 22);
    scatter(data(:, 4), data(:, 10), 'x');
    grid on;
    subplot(5, 6, 23);
    scatter(data(:, 5), data(:, 10), 'x');
    grid on;
    subplot(5, 6, 24);
    scatter(data(:, 6), data(:, 10), 'x');
    grid on;

    % fig 1 row 5
    subplot(5, 6, 25);
    scatter(data(:, 1), data(:, 11), 'x');
    grid on;
    xlabel('R_O_B_S', 'FontWeight', 'bold');
    y = ylabel('J_a_v_e', 'FontWeight', 'bold');
    set(get(gca,'YLabel'),'Rotation',0)
    set(y, 'Units', 'Normalized', 'Position', [-0.4, 0.5, 0]);
    subplot(5, 6, 26);
    scatter(data(:, 2), data(:, 11), 'x');
    grid on;
    xlabel('D_p_r_o_x', 'FontWeight', 'bold');
    subplot(5, 6, 27);
    scatter(data(:, 3), data(:, 11), 'x');
    grid on;
    xlabel('V_m_a_x', 'FontWeight', 'bold');
    subplot(5, 6, 28);
    scatter(data(:, 4), data(:, 11), 'x');
    grid on;
    xlabel('K_1', 'FontWeight', 'bold');
    subplot(5, 6, 29);
    scatter(data(:, 5), data(:, 11), 'x');
    grid on;
    xlabel('D_W_P', 'FontWeight', 'bold');
    subplot(5, 6, 30);
    scatter(data(:, 6), data(:, 11), 'x');
    grid on;
    xlabel('V_G_G', 'FontWeight', 'bold');
end