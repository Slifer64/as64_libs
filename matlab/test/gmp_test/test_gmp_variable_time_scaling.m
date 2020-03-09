%% Simulates a GMP as a DMP
%  Loads a reference trajecory.
%  Trains a GMP based on the reference trajectory.
%  Plots and compares the results.
function test_gmp_variable_time_scaling()

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;

Ts = Timed(2)-Timed(1);

%% initialize and train GMP
train_method = GMP_TRAIN.LS;
N_kernels = 50;
kernels_std_scaling = 2;
n_dof = 3;
gmp = GMP_nDoF(n_dof, N_kernels, 30, 100, kernels_std_scaling);
tic
offline_train_mse = gmp.train(train_method, Timed, Pd_data);
offline_train_mse
toc

%% DMP simulation
disp('GMP simulation...');
tic

spat_s = 1.5; % spatial scale
temp_s = 1.3; % temporal scale
P0 = Pd_data(:,1);
Pgd = Pd_data(:,end);
Pg = P0 + spat_s*(Pgd - P0);
T = Timed(end) / temp_s;
dt = Ts;

%% set initial values
Dim = gmp.length();
y_ddot = zeros(Dim,1);
y_dot = zeros(Dim,1);
y0 = P0;
g = Pg;
y = y0;
t = 0.0;
z_dot = zeros(Dim,1);
z = zeros(Dim,1);

t_end = T;
tau = t_end;
x = 0.0;
x_dot = 1/tau;
xd_dot = 1/tau;
x_ddot = 0;
s = [x; x_dot; x_ddot];

iters = 0;
Time = [];
P_data = [];
dP_data = [];
ddP_data = [];
x_data = [];

gmp.setY0(y0);
gmp.setGoal(g);

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    P_data = [P_data y];
    dP_data = [dP_data y_dot];  
    ddP_data = [ddP_data y_ddot];
    x_data = [x_data x];

    %% DMP simulation
    y_c = 0.0;
    z_c = 0.0;
    gmp.update(s, y, z, y_c, z_c);
    y_dot = gmp.getYdot();
    z_dot = gmp.getZdot();

    yc_dot = 0.0;
    y_ddot = gmp.getYddot(yc_dot);

    %% Update phase variable
    if (t < 0.15*t_end), fv = 0;
    elseif (t < 0.2*t_end), fv = 4.5;
    elseif (t < 0.3*t_end), fv = 0;
    elseif (t < 0.6*t_end), fv = -5.5;
    elseif (t < 0.75*t_end), fv = 3.5;
    else, fv = 0;
    end
    
    xd_ddot = 50*(xd_dot - x_dot) + fv;
    x_3dot = 2*(xd_ddot - x_ddot);

    %% Stopping criteria
    if (x>=1.05) % && norm(y-g)<5e-3 && norm(dy)<5e-3)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + x_dot*dt;
    x_dot = x_dot + x_ddot*dt;
    x_ddot = x_ddot + x_3dot*dt;
    y = y + y_dot*dt;
    z = z + z_dot*dt;

    s = [x; x_dot; x_ddot];
    
end

toc


%% Reference trajectory (scaled)
Timed = Timed / temp_s;
Pd_data = spat_s*( Pd_data-P0 ) + P0;
dPd_data = spat_s*dPd_data*temp_s;
ddPd_data = spat_s*ddPd_data*temp_s^2;

%% Plot results
for i=1:3
    figure;
    subplot(3,1,1);
    hold on;
    plot(Time, P_data(i,:), 'LineWidth',2.0 , 'Color','blue');
    plot(Timed, Pd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
    title(['temporal scale: $' num2str(temp_s) '$     ,     spatial scale: $' num2str(spat_s) '$'], 'interpreter','latex', 'fontsize',18);
    legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);

    axis tight;
    hold off;

    subplot(3,1,2);
    hold on;
    plot(Time, dP_data(i,:), 'LineWidth',2.0, 'Color','blue');
    plot(Timed, dPd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
    axis tight;
    hold off;

    subplot(3,1,3);
    hold on;
    plot(Time, ddP_data(i,:), 'LineWidth',2.0, 'Color','blue');
    plot(Timed, ddPd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
    axis tight;
    hold off;
end


figure;
hold on;
plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','blue');
hold off;

figure;
hold on;
plot(Time, x_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
plot([Time(1) Time(end)], [0 Time(end)/tau], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
hold off;

end


