%% Simulates a GMP as a DMP
%  Loads a reference trajecory.
%  Trains a GMP based on the reference trajectory.
%  Plots and compares the results.
function test_reverse_gmp()

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);

%% initialize and train GMP
D = 30;
K = 100;
train_method = GMP_TRAIN.LS;
N_kernels = 50;
kernels_std_scaling = 2;
gmp = GMP(N_kernels, D, K, kernels_std_scaling);
tic
offline_train_mse = gmp.train(train_method, Timed, Pd_data);
offline_train_mse
toc

%% DMP simulation
disp('GMP simulation...');
tic

spat_s = 1.5; % spatial scale
temp_s = 1.3; % temporal scale
P0 = Pd_data(1);
Pgd = Pd_data(end);
Pg = P0 + spat_s*(Pgd - P0);
T = Timed(end) / temp_s;
dt = Ts;
[Time, P_data, dP_data, ddP_data, F_dist, x_data] = simulateReverseGMP(gmp, P0, Pg, T, dt, @(t)disturbance_function(t, T, 0));
toc


%% Reference trajectory (scaled)
Timed = Timed / temp_s;
Pd_data0 = spat_s*( Pd_data-P0 ) + P0;
dPd_data0 = spat_s*dPd_data*temp_s;
ddPd_data0 = spat_s*ddPd_data*temp_s^2;

Pd_data = fliplr( Pd_data0 );
dPd_data = fliplr( -dPd_data0 );
ddPd_data = fliplr( ddPd_data0 );

%% Plot results
figure;
subplot(5,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed, Pd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
plot(Timed, Pd_data0, 'LineWidth',2.0, 'LineStyle','--', 'Color',[0 0.7 0]);
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
title(['temporal scale: $' num2str(temp_s) '$     ,     spatial scale: $' num2str(spat_s) '$'], 'interpreter','latex', 'fontsize',18);
legend({'sim','reverse demo','demo'}, 'interpreter','latex', 'fontsize',15);

axis tight;
hold off;

subplot(5,1,2);
hold on;
plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, dPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
plot(Timed, dPd_data0, 'LineWidth',2.0, 'LineStyle','--', 'Color',[0 0.7 0]);
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(5,1,3);
hold on;
plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, ddPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
plot(Timed, ddPd_data0, 'LineWidth',2.0, 'LineStyle','--', 'Color',[0 0.7 0]);
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;


subplot(5,1,4);
hold on;
plot(Time, x_data, 'LineWidth',2.0, 'Color','blue');
ylabel('phase var $x$', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(5,1,5);
hold on;
plot(Time, F_dist, 'LineWidth',2.0, 'Color','red');
ylabel('$F_{dist}$ [$N$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;


end


function fdist = disturbance_function(t, T, f_max)
    
    t1 = 0.35*T;
    t2 = 0.45*T;
    t3 = 0.65*T;
    t4 = 0.75*T;

    if (t < t1), fdist = 0;
    elseif (t < t2), fdist = f_max*(t - t1)/(t2-t1);
    elseif (t < t3), fdist = f_max;
    elseif (t < t4), fdist = f_max - f_max*(t - t3)/(t4-t3);
    else, fdist = 0;
    end

end
