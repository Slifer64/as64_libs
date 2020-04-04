%% Simulates a GMP as a DMP
%  Loads a reference trajecory.
%  Trains a GMP based on the reference trajectory.
%  Plots and compares the results.
function test_gmp_as_dmp()

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);

%% initialize and train GMP
train_method = 'LS';
N_kernels = 50;
kernels_std_scaling = 1.5;
gmp = GMP(N_kernels, 30, 100, kernels_std_scaling);
tic
offline_train_mse = gmp.train(train_method, Timed, Pd_data);
offline_train_mse
toc

%% DMP simulation
disp('GMP simulation...');
tic

ks = 1.5; % spatial scale
kt = 1.3; % temporal scale
P0 = Pd_data(1);
Pgd = Pd_data(end);
Pg = P0 + ks*(Pgd - P0);
T = Timed(end) / kt;
dt = Ts;
[Time, P_data, dP_data, ddP_data] = simulateGMP(gmp, P0, Pg, T, dt);
toc


%% Reference trajectory (scaled)
Timed = Timed / kt;
Pd_data = ks*( Pd_data-P0 ) + P0;
dPd_data = ks*dPd_data*kt;
ddPd_data = ks*ddPd_data*kt^2;

%% Plot results
figure;
subplot(3,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed, Pd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
title(['temporal scale: $' num2str(kt) '$     ,     spatial scale: $' num2str(ks) '$'], 'interpreter','latex', 'fontsize',18);
legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);

axis tight;
hold off;

subplot(3,1,2);
hold on;
plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, dPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(3,1,3);
hold on;
plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, ddPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;


end


