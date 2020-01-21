function test_gmp()

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);

%% initialize DMP
a_z = 20;
b_z = a_z/4;
train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
% shape_attr_gat_ptr = LinGatingFunction(1.0, 0.05);
N_kernels = 50;
gmp = GMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);
tic
offline_train_mse = gmp.train(train_method, Timed, Pd_data);
offline_train_mse
toc

%% DMP simulation
disp('GMP simulation...');
tic

spat_s = 1; % spatial scale
temp_s = 0.5; % temporal scale
P0 = Pd_data(1);
Pgd = Pd_data(end);
Pg = P0 + spat_s*(Pgd - P0);
T = temp_s*Timed(end);
dt = Ts;
[Time, P_data, dP_data, ddP_data] = simulateDMP(gmp, P0, Pg, T, dt);
toc

%% Plot results

figure;
subplot(3,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed*temp_s, spat_s*Pd_data(1,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(3,1,2);
hold on;
plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed*temp_s, spat_s*dPd_data(1,:)/temp_s, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(3,1,3);
hold on;
plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed*temp_s, spat_s*ddPd_data(1,:)/temp_s^2, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;


end


