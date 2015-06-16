%
% plot_data.m
%
% plots data from gazebo simulation
%

clear all
close all

data = dlmread('ThreePiSim_out.csv', ',');

figure(1)
hold on
plot(data(:, 1), data(:, 4), 'm');
plot(data(:, 1), data(:, 2), 'b');
xlabel('iteration');
ylabel('left wheel velocity');
legend('desired', 'actual');

figure(2)
hold on
plot(data(:, 1), data(:, 5), 'm');
plot(data(:, 1), data(:, 3), 'b');
xlabel('iteration');
ylabel('right wheel velocity');
legend('actual', 'desired');

figure(3)
hold on
plot(data(:, 1), data(:, 10), 'b');
plot(data(:, 1), data(:, 11), 'm');
xlabel('iteration');
ylabel('tourque');
legend('left', 'right');

