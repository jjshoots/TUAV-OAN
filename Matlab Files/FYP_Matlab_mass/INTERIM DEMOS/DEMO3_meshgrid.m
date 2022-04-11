clear all;
close all;

[X, Y] = meshgrid(0:0.2:11, 0:0.2:11);

for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        hold1 = norm([5-X(i, j) 5-Y(i, j)]);
        hold2 = 5/(1+exp(-hold1 + 3));
        Z(i, j) = min(5, max(0, hold2));
    end
end

surf(X, Y, Z);
xlabel('X-position');
ylabel('Y-position');
zlabel('Velocity, V');