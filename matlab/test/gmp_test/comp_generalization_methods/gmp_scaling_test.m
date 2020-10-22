
set_matlab_utils_path()

%% Load training data
load('../data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos;

Ts = Timed(2)-Timed(1);

%figure
%plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'Color','blue');

%% initialize and train GMP
N_kernels = 30;
kernels_std_scaling = 1;
gmp = GMP_nDoF(3, N_kernels, 30, 150, kernels_std_scaling);
tic
offline_train_mse = gmp.train('LS', Timed, Pd_data);
offline_train_mse
toc

return

%% GMP simulation

temp_s = 1;
spat_s = 1;
compareScaledTrajectories(gmp, Timed, Pd_data, dPd_data, ddPd_data, temp_s, spat_s);


%% Compares the scaled trajectories produced by reference trajectory (Timed, Pd_data, dPd_data, ddPd_data)
%% and the scaled trajectory produced by a GMP trained with the reference trajectory.
% param[in] gmp: GMP object
% param[in] Timed: reference trajectory timestamps
% param[in] Pd_data: reference position
% param[in] dPd_data: reference velocity
% param[in] ddPd_data: reference acceleration
% param[in] temp_s: temporal scaling
% param[in] spat_s: spatial scaling
function compareScaledTrajectories(gmp, Timed, Pd_data, dPd_data, ddPd_data, temp_s, spat_s)

    Pgd = Pd_data(end);
    P0d = Pd_data(1);
    P0 = P0d;
    Pg = spat_s*(Pgd-P0d) + P0;
    
    % scaled trajectory of GMP
    [Time, P_data, dP_data, ddP_data] = getScaledTrajectory(gmp, Timed, temp_s, Pg);

    % scaled result of reference trajectory
    Time2 = Timed / temp_s;
    P_data2 = spat_s*(Pd_data - P0d) + P0;
    dP_data2 = spat_s*dPd_data * temp_s;
    ddP_data2 = spat_s*ddPd_data * temp_s^2 ;

    % compare results
    figure;
    
    subplot(3,1,1);
    hold on;
    plot(Time, P_data, 'LineWidth',2.0, 'Color', 'blue');
    plot(Time2, P_data2, 'LineWidth',2.0, 'Color', 'magenta', 'LineStyle',':');
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
    title(['temporal scale: $' num2str(temp_s) '$     ,     spatial scale: $' num2str(spat_s) '$'], 'interpreter','latex', 'fontsize',18);
    legend({'GMP','ref'}, 'interpreter','latex', 'fontsize',15);
    hold off;
    
    subplot(3,1,2);
    hold on;
    plot(Time, dP_data, 'LineWidth',2.0, 'Color', 'blue');
    plot(Time2, dP_data2, 'LineWidth',2.0, 'Color', 'magenta', 'LineStyle',':');
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
    hold off;
    
    subplot(3,1,3);
    hold on;
    plot(Time, ddP_data, 'LineWidth',2.0, 'Color', 'blue');
    plot(Time2, ddP_data2, 'LineWidth',2.0, 'Color', 'magenta', 'LineStyle',':');
    ylabel('pos [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
    hold off;
    
    N = length(P_data);
    pos_err = norm(P_data - P_data2) / N;
    vel_err = norm(dP_data - dP_data2) / N;
    accel_err = norm(ddP_data - ddP_data2) / N;
    
    fprintf('==================================\n');
    fprintf('*temp_s: %.1f,  *spatial-scale: %.1f\n', temp_s, spat_s);
    fprintf('> Position error    : %f\n', pos_err);
    fprintf('> Velocity error    : %f\n', vel_err);
    fprintf('> Acceleration error: %f\n', accel_err);
    fprintf('==================================\n');

end


%% Returns the scaled trajectory for a GMP.
% param[in] gmp: GMP object
% param[in] Time0: initial timestamps (before scaling)
% param[in] temp_s: temporal scaling
% param[in] Pg: New goal (which will result in spatial scaling)
% param[out] Time: timestamps of scaled trajectory
% param[out] P_data: scaled position
% param[out] dP_data: scaled velocity
% param[out] ddP_data: scaled acceleration
function [Time, P_data, dP_data, ddP_data] = getScaledTrajectory(gmp, Time0, temp_s, Pg)

    tau = Time0(end) / temp_s;
    Time = Time0 / temp_s;
    x = Time / tau;
    x_dot = 1/tau;
    N = length(x);
    P_data = zeros(1,N);
    dP_data = zeros(1,N);
    ddP_data = zeros(1,N);

    gmp.setGoal(Pg);
    
    for i=1:N
        P_data(i) = gmp.getYd(x(i));
        dP_data(i) = gmp.getYdDot(x(i), x_dot);
        ddP_data(i) = gmp.getYdDDot(x(i), x_dot, 0);
    end

end

