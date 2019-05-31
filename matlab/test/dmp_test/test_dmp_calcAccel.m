function test_dmp_calcAccel()

set_matlab_utils_path();

%% Load training data

load('data/train_data.mat', 'Data');

Timed = Data.Time;
yd_data = Data.Pos(1,:);
dyd_data = Data.Vel(1,:);
ddyd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);

%% initialize DMP
a_z = 20;
b_z = a_z/4;
train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
N_kernels = 50;
dmp = DMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);

disp('DMP training...')
tic
offline_train_mse = dmp.train(train_method, Timed, yd_data, dyd_data, ddyd_data);
offline_train_mse
toc

xd_data = can_clock_ptr.getPhase(Timed);

%% DMP simulation
disp('DMP simulation...');
tic
y0 = yd_data(:,1);
ygd = yd_data(:,end);
yg = y0 + 1*(ygd - y0);
t_end = Timed(end);
dt = Ts;

[Time, y_data, dy_data, ddy_data, x_data] = simDMP(dmp, y0, yg, t_end, dt);
[Time2, y2_data, dy2_data, ddy2_data, x2_data] = simDMP2(dmp, y0, yg, t_end, dt);


line_width = 2.5;
font_size = 15;

figure;
subplot(4,1,1);
hold on;
plot(Time, x_data, 'LineWidth',line_width, 'LineStyle','-', 'Color','blue');
plot(Time2, x2_data, 'LineWidth',line_width, 'LineStyle','--', 'Color',[0 0.85 0]);
plot(Timed, xd_data, 'LineWidth',line_width, 'LineStyle','--', 'Color','magenta');
axis tight;
legend({'$x$,$x_2$'}, 'interpreter','latex', 'fontsize',font_size);
hold off;
subplot(4,1,2);
hold on;
plot(Time, y_data, 'LineWidth',line_width, 'LineStyle','-', 'Color','blue');
plot(Time2, y2_data, 'LineWidth',line_width, 'LineStyle','--', 'Color',[0 0.85 0]);
plot(Timed, yd_data, 'LineWidth',line_width, 'LineStyle','--', 'Color','magenta');
legend({'$y$','$y_2$','$y_d$'}, 'interpreter','latex', 'fontsize',font_size);
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',font_size);
axis tight;
hold off;
subplot(4,1,3);
hold on;
plot(Time, dy_data, 'LineWidth',line_width, 'LineStyle','-', 'Color','blue');
plot(Time2, dy2_data, 'LineWidth',line_width, 'LineStyle','--', 'Color',[0 0.85 0]);
plot(Timed, dyd_data, 'LineWidth',line_width, 'LineStyle','--', 'Color','magenta');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',font_size);
axis tight;
hold off;
subplot(4,1,4);
hold on;
plot(Time, ddy_data, 'LineWidth',line_width, 'LineStyle','-', 'Color','blue');
plot(Time2, ddy2_data, 'LineWidth',line_width, 'LineStyle','--', 'Color',[0 0.85 0]);
plot(Timed, ddyd_data, 'LineWidth',line_width, 'LineStyle','--', 'Color','magenta');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',font_size);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',font_size);
axis tight;
hold off;

end

function [Time, y_data, dy_data, ddy_data, x_data] = simDMP(dmp, y0, yg, t_end, dt)

%% set initial values
t = 0.0;
x = 0.0;
y = y0;
dy = 0;
ddy = 0;

tau = t_end;
can_clock_ptr = dmp.can_clock_ptr;
can_clock_ptr.setTau(tau);

iters = 0;
Time = [];
y_data = [];
dy_data = [];
ddy_data = [];
x_data = [];

dmp.setY0(y0);

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    y_data = [y_data y];
    dy_data = [dy_data dy];  
    ddy_data = [ddy_data ddy];
    x_data = [x_data x];

    %% DMP simulation
    
    tau_dot = getTauDot(t, tau, t_end);
    
    zc = 0;
    yc = 0;
    yc_dot = 0;
    ddy = dmp.calcYddot(x, y, dy, yg, tau_dot, yc, zc, yc_dot);

    %% Update phase variable
    can_clock_ptr.setTau(tau);
    dx = can_clock_ptr.getPhaseDot(x);
    
    %% Stopping criteria
    if (x>=1 && norm(y-yg)<5e-3 && norm(dy)<5e-3)
        break;
    end
    
    if (x>=1.5)
        warning('[simDMP]: Time limit reached... Stopping simulation!');
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    tau = tau + tau_dot*dt;
    x = x + dx*dt;
    y = y + dy*dt;
    dy = dy + ddy*dt;

end

end

function [Time, y_data, dy_data, ddy_data, x_data] = simDMP2(dmp, y0, yg, t_end, dt)

%% set initial values
t = 0.0;
x = 0.0;
y = y0;
dy = 0;
z = 0;
ddy = 0;

tau = t_end;
can_clock_ptr = dmp.can_clock_ptr;
can_clock_ptr.setTau(tau);

iters = 0;
Time = [];
y_data = [];
dy_data = [];
ddy_data = [];
x_data = [];

dmp.setY0(y0);

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    y_data = [y_data y];
    dy_data = [dy_data dy];  
    ddy_data = [ddy_data ddy];
    x_data = [x_data x];

    %% DMP simulation
    yc = 0;
    zc = 0;
    yc_dot = 0;
    tau_dot = getTauDot(t, tau, t_end);
    dmp.update(x, y, z, yg, yc, zc);
    
    dy = dmp.getYdot();
    dz = dmp.getZdot();
    ddy = (dz + yc_dot -tau_dot*dy )/dmp.getTau();
    % ddy = dz/dmp.getTau();

    %% Update phase variable
    can_clock_ptr.setTau(tau);
    dx = can_clock_ptr.getPhaseDot(x);
    
    %% Stopping criteria
    if (x>=1 && norm(y-yg)<5e-3 && norm(dy)<5e-3)
        break;
    end
    
    if (x>=1.5)
        warning('[simDMP]: Time limit reached... Stopping simulation!');
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    tau = tau + tau_dot*dt;
    x = x + dx*dt;
    y = y + dy*dt;
    z = z + dz*dt;   

end

end


function tau_dot = getTauDot(t, tau, t_end)
    
% tau_dot = 0;
% return

    if (t <= 0.45*t_end), tau_dot = 0;
    elseif (t<=0.6*t_end), tau_dot = 0.3*(250 - tau);
    elseif ( t>0.6*t_end), tau_dot = 0.5*(0.3*t_end - tau);
    end
    
end

