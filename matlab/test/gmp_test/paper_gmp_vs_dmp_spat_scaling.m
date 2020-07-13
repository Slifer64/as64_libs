set_matlab_utils_path();

%% Load training data
% load('data/train_data.mat', 'Data');
load('data/fifthOrd_train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);

train_method = 'LS';
N_kernels = 20;
%% initialize and train GMP
kernels_std_scaling = 1;
K = 500;
D = 100;
gmp = GMP(N_kernels, D, K, kernels_std_scaling);
% tic
offline_train_mse = gmp.train(train_method, Timed, Pd_data);
% offline_train_mse
% toc

%% initialize DMP
a_z = D;
b_z = K/D;
can_clock_ptr = CanonicalClock();
% shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
% shape_attr_gat_ptr = LinGatingFunction(1.0, 0.05);
shape_attr_gat_ptr = LinGatingFunction(1.0, 0.01);
dmp = DMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);
% tic
offline_train_mse = dmp.train(train_method, Timed, Pd_data, dPd_data, ddPd_data);
% offline_train_mse;
% toc

ks_data = [-2, -1.5, -0.5, 0.5 1.5 2];

figure;
ax = axes();
hold(ax, 'on');

plot(nan,nan, 'LineWidth',3.0 ,  'LineStyle','-', 'Color','blue', 'DisplayName','novel');
plot(nan,nan, 'LineWidth',3.0 ,  'LineStyle',':', 'Color','magenta', 'DisplayName','classical');
plot(nan, nan, 'LineWidth',3.0 ,  'LineStyle','-', 'Color',[0 0.7 0], 'DisplayName','training');
legend({}, 'interpreter','latex', 'fontsize',15);

plot(Timed, Pd_data, 'LineWidth',3.0 ,  'LineStyle','-', 'Color',[0 0.7 0], 'HandleVisibility','off');

for k=1:length(kt_data)
    
    ks = ks_data(k);
    
    kt = 1; % spatial scale
    % kt = 1.3; % temporal scale
    P0 = Pd_data(1);
    Pgd = Pd_data(end);
    Pg = P0 + ks*(Pgd - P0);
    T = Timed(end) / kt;
    dt = Ts;
    
    [Time, P_data] = simulateGMP(gmp, P0, Pg, T, dt);
    [Time2, P_data2] = simulateDMP(dmp, P0, Pg, T, dt);

    %% Plot results
    plot(Time, P_data, 'LineWidth',3.0 ,  'LineStyle','-', 'Color','blue', 'HandleVisibility','off');
    plot(Time2, P_data2, 'LineWidth',3.0, 'LineStyle',':', 'Color','magenta', 'HandleVisibility','off');
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);

end

% plot(ax.XLim, [Pgd Pgd], 'LineWidth',3.0, 'LineStyle','-', 'Color',[0 0 1 0.3], 'HandleVisibility','off');

axis tight;
hold off;



