clc;
close all;
clear;

set_matlab_utils_path();
data_path = '../data/';

%% Load training data
load([data_path 'train_data1.mat'], 'Data');

Time = Data.Time;
Pd_data = Data.Pos(3,:);
dPd_data = Data.Vel(3,:);
ddPd_data = Data.Accel(3,:);

[Time, Pd_data, dPd_data, ddPd_data] = trimData(Time, Pd_data, dPd_data, ddPd_data, 5e-3);

Ts = Time(2)-Time(1);

n_data = length(Time);

tau = Time(end);
x = Time / tau;
x0 = 0;
xf = 1;

%% initialize and train GMP
N_kernels = 20;
kernels_std_scaling = 1;
mp = MP(N_kernels, kernels_std_scaling);
tic
offline_train_mse = mp.train('LS', x, Pd_data);
offline_train_mse
toc
mp.optWeightsCovariance();

dx = 1/tau;
[P_data, dP_data, ddP_data] = mp.simulate(x, dx, 0);


P0 = Pd_data(:,1);
Pg = Pd_data(:,end);

ks = 1.2;
Pg2 = ks*(Pg - P0) + P0;

% calc scaled demo
Pd2_data = ks*(Pd_data - P0) + P0;
dPd2_data = ks*dPd_data;
ddPd2_data = ks*ddPd_data;

% update weights based on final goal
x_m = [x0; xf];
z_m = [P0; Pg2];

% update weights based on intermediate position
k = round(n_data*0.5);
xk = x(k);
Pk = Pd2_data(:,k);
x_m = [x0; xk];
z_m = [P0; Pk];

% calc scaled MP
mp.updatePos(x_m, z_m); 
[P2_data, dP2_data, ddP2_data] = mp.simulate(x, dx, 0);

figure;
subplot(2,1,1); hold on;
plot(x, P2_data, 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
plot(x, Pd2_data, 'LineWidth',2, 'LineStyle','--', 'Color','blue');
plot(x, Pd_data, 'LineWidth',2, 'LineStyle','--', 'Color','green');
scatter(xk, Pk, 'SizeData',150, 'LineWidth',5, 'Marker','o', 'MarkerEdgeColor','red');
legend({'MP','DMP','demo','$P_k$'}, 'interpreter','latex', 'fontsize',15);
axis tight;
ax = subplot(2,1,2); hold on;
mp.plotPsi(x, ax);
xlabel('canonical time [$t/\tau$]', 'interpreter','latex', 'fontsize',15);


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

