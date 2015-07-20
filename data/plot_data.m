%
% plot_data.m
%
% plots data from gazebo simulation
%

clear all
close all

robot = dlmread('ThreePiSim_Rob.csv', ',');
plane = dlmread('ThreePiSim_Plane.csv', ',');

figure(1)
plot(plane(:, 1), plane(:, 2));
xlabel('time (s)');
ylabel('angle (rad)');

