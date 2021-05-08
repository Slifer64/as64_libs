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

% test againt this SMP
smp2 = SMP(N_kernels, kernels_std_scaling);
smp2.train(x, Pd_data); % train smp
w_o = smp2.w;
% set covariance matrix
S = zeros(N_kernels,N_kernels);
for i=1:N_kernels
    for j=i+1:N_kernels
        S(i,j) = exp(-0.1 * abs(i-j));
    end
end
S = S / max(S(:));
S = S + S' + eye(N_kernels,N_kernels);
smp2.Sigma_w = S;
% create some trajectories
yg = Pd_data(end);
y0 = Pd_data(1);
yg_new = [0.8 0.9 1.1 1.3]*yg;
y0_new = [1.0 1.0 1.0 1.0]*y0;
m = length(yg_new);
x = 0:0.005:1;
% create nominal trajectory
y_o = smp2.simulate(x, 1/tau, 0);
Y_data = zeros(m, length(x));
% Y_data = 1.5*(y_o - y_o(1)) + y_o(1);
for i=1:m
    smp2.updatePos([x(1); x(end)], [y0_new(i); yg_new(i)]);
    Y_i = smp2.simulate(x, 1/tau, 0);
    Y_data(i,:) = Y_i;
    smp2.w = w_o; % restore previous weights
end

figure;
plot(Y_data')
% return
%Y_data = 1.5*(Pd_data-Pd_data(1)) + Pd_data(1);

tic
smp.train(x, y_o, Y_data);
toc

is_spd = isSPD(smp.Sigma_w)

save('data.mat', 'Time','Pd_data','smp');

figure;
smp2.plotWeightsCovariance(subplot(2,1,1));
smp.plotWeightsCovariance(subplot(2,1,2));

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

