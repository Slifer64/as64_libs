clc;
close all;
clear;

set_matlab_utils_path();

train_data_file = 'train_data3.mat';
data_path = '../data/';

%% Load training data
load([data_path train_data_file], 'Data');

Time = Data.Time;
P_data = Data.Pos;
dP_data = Data.Vel;
ddP_data = Data.Accel;


figure;
for i=1:3
    subplot(3,3,i);
    plot(Time, P_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
    axis tight;
end
for i=1:3
    subplot(3,3,3+i);
    plot(Time, dP_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
    axis tight;
end
for i=1:3
    subplot(3,3,6+i);
    plot(Time, ddP_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','green');
    axis tight;
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
end

figure; hold on;
plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth',3, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
scatter3(P_data(1,1), P_data(2,1), P_data(3,1), 'SizeData',150, 'LineWidth',5, 'Marker','o', 'MarkerEdgeColor','blue');
scatter3(P_data(1,end), P_data(2,end), P_data(3,end), 'SizeData',150, 'LineWidth',5, 'Marker','x', 'MarkerEdgeColor','green');
xlabel('X [$m$]', 'interpreter','latex', 'fontsize',15);
xlabel('Y [$m$]', 'interpreter','latex', 'fontsize',15);
xlabel('Z [$m$]', 'interpreter','latex', 'fontsize',15);
