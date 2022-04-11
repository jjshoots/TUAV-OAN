function plot_anova(data, label)
    % plot stuff
    figure(label);

    % fig 1 row 3
    subplot(5, 6, 1);
    anova1(data(:, 7), data(:, 1));
    grid on;
    y = ylabel('T_C', 'FontWeight', 'bold');
    set(get(gca,'YLabel'),'Rotation',0)
    set(y, 'Units', 'Normalized', 'Position', [-0.4, 0.5, 0]);
    subplot(5, 6, 2);
    anova1(data(:, 7), data(:, 2));
    grid on;
    subplot(5, 6, 3);
    anova1(data(:, 7), data(:, 3));
    grid on;
    subplot(5, 6, 4);
    anova1(data(:, 7), data(:, 4));
    grid on;
    subplot(5, 6, 5);
    anova1(data(:, 7), data(:, 5));
    grid on;
    subplot(5, 6, 6);
    anova1(data(:, 7), data(:, 6));
    grid on;

    % fig 1 row 2
    subplot(5, 6, 7);
    anova1(data(:, 8), data(:, 1));
    grid on;
    y = ylabel('D_P_A_B', 'FontWeight', 'bold');
    set(get(gca,'YLabel'),'Rotation',0)
    set(y, 'Units', 'Normalized', 'Position', [-0.4, 0.5, 0]);
    subplot(5, 6, 8);
    anova1(data(:, 8), data(:, 2));
    grid on;
    subplot(5, 6, 9);
    anova1(data(:, 8), data(:, 3));
    grid on;
    subplot(5, 6, 10);
    anova1(data(:, 8), data(:, 4));
    grid on;
    subplot(5, 6, 11);
    anova1(data(:, 8), data(:, 5));
    grid on;
    subplot(5, 6, 12);
    anova1(data(:, 8), data(:, 6));
    grid on;

    % fig 1 row 3
    subplot(5, 6, 13);
    anova1(data(:, 9), data(:, 1));
    grid on;
    y = ylabel('n_A_*', 'FontWeight', 'bold');
    set(get(gca,'YLabel'),'Rotation',0)
    set(y, 'Units', 'Normalized', 'Position', [-0.4, 0.5, 0]);
    subplot(5, 6, 14);
    anova1(data(:, 9), data(:, 2));
    grid on;
    subplot(5, 6, 15);
    anova1(data(:, 9), data(:, 3));
    grid on;
    subplot(5, 6, 16);
    anova1(data(:, 9), data(:, 4));
    grid on;
    subplot(5, 6, 17);
    anova1(data(:, 9), data(:, 5));
    grid on;
    subplot(5, 6, 18);
    anova1(data(:, 9), data(:, 6));
    grid on;

    % fig 1 row 4
    subplot(5, 6, 19);
    anova1(data(:, 10), data(:, 1));
    grid on;
    y = ylabel('J_m_a_x', 'FontWeight', 'bold');
    set(get(gca,'YLabel'),'Rotation',0)
    set(y, 'Units', 'Normalized', 'Position', [-0.4, 0.5, 0]);
    subplot(5, 6, 20);
    anova1(data(:, 10), data(:, 2));
    grid on;
    subplot(5, 6, 21);
    anova1(data(:, 10), data(:, 3));
    grid on;
    subplot(5, 6, 22);
    anova1(data(:, 10), data(:, 4));
    grid on;
    subplot(5, 6, 23);
    anova1(data(:, 10), data(:, 5));
    grid on;
    subplot(5, 6, 24);
    anova1(data(:, 10), data(:, 6));
    grid on;

    % fig 1 row 5
    subplot(5, 6, 25);
    anova1(data(:, 11), data(:, 1));
    grid on;
    xlabel('R_O_B_S', 'FontWeight', 'bold');
    y = ylabel('J_a_v_e', 'FontWeight', 'bold');
    set(get(gca,'YLabel'),'Rotation',0)
    set(y, 'Units', 'Normalized', 'Position', [-0.4, 0.5, 0]);
    subplot(5, 6, 26);
    anova1(data(:, 11), data(:, 2));
    grid on;
    xlabel('D_p_r_o_x', 'FontWeight', 'bold');
    subplot(5, 6, 27);
    anova1(data(:, 11), data(:, 3));
    grid on;
    xlabel('V_m_a_x', 'FontWeight', 'bold');
    subplot(5, 6, 28);
    anova1(data(:, 11), data(:, 4));
    grid on;
    xlabel('K_1', 'FontWeight', 'bold');
    subplot(5, 6, 29);
    anova1(data(:, 11), data(:, 5));
    grid on;
    xlabel('D_W_P', 'FontWeight', 'bold');
    subplot(5, 6, 30);
    anova1(data(:, 11), data(:, 6));
    grid on;
    xlabel('V_G_G', 'FontWeight', 'bold');
end