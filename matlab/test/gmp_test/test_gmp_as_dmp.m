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
a_z = 20;
b_z = a_z/4;
train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
N_kernels = 50;
kernels_std_scaling = 2;
gmp = GMP(N_kernels, a_z, b_z, can_clock_ptr, kernels_std_scaling);
tic
offline_train_mse = gmp.train(train_method, Timed, Pd_data);
offline_train_mse
toc

%% DMP simulation
disp('GMP simulation...');
tic

spat_s = 1.0; % spatial scale
temp_s = 1.0; % temporal scale
P0 = Pd_data(1);
Pgd = Pd_data(end);
Pg = P0 + spat_s*(Pgd - P0);
T = Timed(end) / temp_s;
dt = Ts;
[Time, P_data, dP_data, ddP_data] = simulateDMP(gmp, P0, Pg, T, dt);
toc


%% Reference trajectory (scaled)
Timed = Timed / temp_s;
Pd_data = spat_s*( Pd_data-P0 ) + P0;
dPd_data = spat_s*dPd_data*temp_s;
ddPd_data = spat_s*ddPd_data*temp_s^2;

%% Plot results
figure;
subplot(3,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed, Pd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
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


