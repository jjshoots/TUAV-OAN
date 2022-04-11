function [rho, p] = anova_pearson(data)

    p1 = anovan(data(:, 7), data(:, 1:6), 'display', 'off');
    p2 = anovan(data(:, 8), data(:, 1:6), 'display', 'off');
    p3 = anovan(data(:, 9), data(:, 1:6), 'display', 'off');
    p4 = anovan(data(:, 10), data(:, 1:6), 'display', 'off');
    p5 = anovan(data(:, 11), data(:, 1:6), 'display', 'off');

    p = [p1'; p2'; p3'; p4'; p5'];

    rho = corr(data(:, 1:6), data(:, 7:11))';

end