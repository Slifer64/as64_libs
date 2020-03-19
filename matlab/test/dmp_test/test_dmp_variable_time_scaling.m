%% Simulates a GMP as a DMP
%  Loads a reference trajecory.
%  Trains a GMP based on the reference trajectory.
%  Plots and compares the results.
function test_dmp_variable_time_scaling()

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;

Ts = Timed(2)-Timed(1);

%% initialize DMP
a_z = 1;
b_z = 1/a_z;
train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
N_kernels = 60;
dmp = DMP_pos(DMP_TYPE.STD, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);
tic
offline_train_mse = dmp.train(train_method, Timed, Pd_data, dPd_data, ddPd_data);
offline_train_mse
toc

%% Compare with the learned trajectory
% [Timed, Pd_data, dPd_data, ddPd_data] = simulateGMP_nDoF(gmp, Pd_data(:,1), Pd_data(:,end), Timed(end), Ts);

%% DMP simulation
disp('DMP simulation...');
tic

spat_s = 1.5; % spatial scale
temp_s = 0.5; % temporal scale
P0 = Pd_data(:,1);
Pgd = Pd_data(:,end);
Pg = P0 + spat_s*(Pgd - P0);
T = Timed(end) / temp_s;
dt = 0.0001; %Ts;

%% set initial values
Dim = size(Pd_data,1);
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
fv_prev = 0;

iters = 0;
Time = [];
P_data = [];
dP_data = [];
ddP_data = [];
x_data = [];
x_dot_data = [];
zc_data = [];
z_c = zeros(3,1);

dmp.setY0(y0);

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    P_data = [P_data y];
    dP_data = [dP_data y_dot];  
    ddP_data = [ddP_data y_ddot];
    x_data = [x_data x];
    x_dot_data = [x_dot_data x_dot];
    zc_data = [zc_data z_c];

    %% DMP simulation
    y_c = zeros(Dim,1);
    z_c = zeros(Dim,1);
    dmp.update(x, y, z, g, y_c, z_c);
    y_dot = dmp.getYdot();
    z_dot = dmp.getZdot();

    yc_dot = 0.0;
    y_ddot = dmp.getYddot();

    %% Update phase variable
    if (x < 0.1), fv = 10;
    elseif (x < 0.2), fv = 20;
    elseif (x < 0.3), fv = 50;
    elseif (x < 0.5), fv = 0;
    elseif (x < 0.7), fv = 50;
    elseif (x < 0.9), fv = 0;
    else, fv = 10;
    end
    
    a_v = 0.99;
    fv = a_v*fv_prev + (1-a_v)*fv;
    fv_prev = fv;
    x_ddot = 50*(xd_dot - x_dot) + fv;
    tau_dot = -tau^2*x_ddot;

    %% Stopping criteria
    if (x>=1.1 && norm(y-g)<5e-3) % && norm(y_dot)<5e-3)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + x_dot*dt;
    x_dot = x_dot + x_ddot*dt;
    tau = tau + tau_dot*dt;

    y = y + y_dot*dt;
    z = z + z_dot*dt;

    dmp.setTau(tau);
    
end

toc

tau = t_end;


%% Reference trajectory (scaled)
Timed = Timed / temp_s;
Pd_data = spat_s*( Pd_data-P0 ) + P0;
dPd_data = spat_s*dPd_data*temp_s;
ddPd_data = spat_s*ddPd_data*temp_s^2;


[dist, i_pd, i_p] = dtw(Pd_data, P_data);

N2 = length(i_p);
dist = dist / N2;

Time2 = (0:(N2-1))*dt;
Pd_data2 = Pd_data(:, i_pd');
P_data2 = P_data(:, i_p');

figure;
for i=1:3
subplot(3,2, (i-1)*2+1);
hold on;
plot(Time2, Pd_data2(i,:), 'LineWidth',2, 'Color','blue', 'LineStyle','-');
plot(Time2, P_data2(i,:), 'LineWidth',2, 'Color','magenta', 'LineStyle',':');
hold off;
subplot(3,2, (i-1)*2+2);
plot(Time2, P_data2(i,:)-Pd_data2(i,:), 'LineWidth',2, 'Color','red');
end


figure;
subplot(2,1,1);
hold on;
plot(Time, x_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
plot([Time(1) Time(end)], [0 Time(end)/tau], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
ylabel('$x$', 'interpreter','latex', 'fontsize',15);
hold off;
subplot(2,1,2);
hold on;
plot(Time, x_dot_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
plot([Time(1) Time(end)], [1/tau 1/tau], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
ylabel('$\dot{x}$', 'interpreter','latex', 'fontsize',15);
hold off;

ax = axes('Parent',figure());
hold(ax,'on');
plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue', 'Parent',ax);
plot3(Pg(1), Pg(2), Pg(3), 'LineWidth',4, 'LineStyle','-', 'Marker','o', 'MarkerSize',10, 'Color','red', 'Parent',ax);
plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta', 'Parent',ax);
hold(ax,'off');

end
