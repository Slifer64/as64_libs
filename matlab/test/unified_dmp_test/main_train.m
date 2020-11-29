clc;
close all;
clear;

set_matlab_utils_path();
data_path = '../data/';

%% Load training data
load([data_path 'train_data1.mat'], 'Data');

Time = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

[Time, Pd_data, dPd_data, ddPd_data] = trimData(Time, Pd_data, dPd_data, ddPd_data, 5e-3);

Ts = Time(2)-Time(1);

n_data = length(Time);

tau = Time(end);
x = Time / tau;
x0 = 0;
xf = 1;

%% initialize and train GMP
N_kernels = 15;
kernels_std_scaling = 1;
smp = SMP(N_kernels, kernels_std_scaling);
tic
Y_data = 1.5*(Pd_data-Pd_data(1)) + Pd_data(1);
smp.train(x, Pd_data, Y_data);
toc

is_spd = isSPD(smp.Sigma_w)

save('data.mat', 'Time','Pd_data','smp');

% figure;
% subplot(3,1,1); hold on;
% plot(Time, P2_data, 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
% plot(Time, Pd2_data, 'LineWidth',2, 'LineStyle','-', 'Color','blue');
% plot(Time, Pd_data, 'LineWidth',2, 'LineStyle','--', 'Color','green');
% legend({'MP','DMP','demo'}, 'interpreter','latex', 'fontsize',15);
% axis tight;
% subplot(3,1,2); hold on;
% plot(Time, dP_data, 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
% plot(Time, dPd2_data, 'LineWidth',2, 'LineStyle','-', 'Color','blue');
% plot(Time, dPd_data, 'LineWidth',2, 'LineStyle','--', 'Color','green');
% axis tight;
% subplot(3,1,3); hold on;
% plot(Time, ddP2_data, 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
% plot(Time, ddPd2_data, 'LineWidth',2, 'LineStyle','-', 'Color','blue');
% plot(Time, ddPd_data, 'LineWidth',2, 'LineStyle','--', 'Color','green');
% axis tight;
% xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);

% figure; hold on;
% plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
% plot3(P2_data(1,:), P2_data(2,:), P2_data(3,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
% plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'LineStyle','--', 'Color','green');
% xlabel('X [$m$]', 'interpreter','latex', 'fontsize',15);
% xlabel('Y [$m$]', 'interpreter','latex', 'fontsize',15);
% xlabel('Z [$m$]', 'interpreter','latex', 'fontsize',15);

