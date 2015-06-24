%
% plot_data.m
%
% plots data from gazebo simulation
%

clear all
close all

data = dlmread('ThreePiSim_out.csv', ',');
state = dlmread('ThreePiSim_pose.csv', ',');

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

figure(4)
hold on
plot(state(:, 1), state(:, 2))
xlabel('x posiiton');
ylabel('y position');

figure(5)
plot(state(:, 3))
xlabel('sample')
ylabel('theta')
