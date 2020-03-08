clc;
% close all;
clear;

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);


%% initialization and training
a_z = 20;
b_z = a_z/4;
N_kernels = 50;
kernels_std_scaling = 2;
gmp = GMP(N_kernels, a_z, a_z*b_z, kernels_std_scaling);

gmp.train(DMP_TRAIN.LS, Timed, Pd_data);

taud = Timed(end);
yd0 = Pd_data(1);
gd = Pd_data(end);

temp_s = 1; % temporal scaling
spat_s = 1; % spatial scaling
tau = taud/temp_s;
y0 = yd0; % + 0.1;
g = spat_s*(gd - yd0) + y0;
ydot_0 = 0;
% =======  Position constraints  =======
pos_constr = [GMPConstr(0,y0,'='); GMPConstr(3,0.45,'='); GMPConstr(tau,0.7,'=')];
ti = 0:0.04:tau;
pos_constr = [pos_constr; lowerBoundConstr(ti, 0.4)];
% =======  Velocity constraints  =======
vel_constr = [GMPConstr(0,0,'='); GMPConstr(6,-0.05,'='); GMPConstr(tau,0.1,'=')];
ti = 0:0.04:tau;
vel_thres = 0.12;
vel_constr = [vel_constr; thresholdConstr(ti, 0.12)];
% =======  Acceleration constraints  =======
accel_constr = [GMPConstr(0,0,'='); GMPConstr(2,0.15,'='); GMPConstr(tau,0,'=')];
ti = 0:0.04:tau;
accel_constr = [accel_constr; upperBoundConstr(ti, 0.4); lowerBoundConstr(ti, -0.2)];
% tic
gmp.setY0(y0);
gmp.setGoal(g);
opt_set = GMPOptSet(true, true, true, 1, 0.5, 0.1);
gmp.constrOpt(tau, pos_constr, vel_constr, accel_constr, opt_set);
gmp.setOptTraj(true);
% toc


%% calculate scaled demo trajectory
Time2 = Timed / temp_s;
P_data2 = spat_s*(Pd_data - Pd_data(1)) + y0;
dP_data2 = temp_s*spat_s*dPd_data;
ddP_data2 = (temp_s^2)*spat_s*ddPd_data;


%% simulation

Time = [];
P_data = [];
dP_data = [];
ddP_data = [];

p = y0;
p_dot = ydot_0;
p_ddot = 0;

t = 0;
dt = 0.01;

while (true)
    
    Time = [Time t];
    P_data = [P_data p];
    dP_data = [dP_data p_dot];
    ddP_data = [ddP_data p_ddot];
    
    x = t/tau;
    x_dot = 1/tau;
    p = gmp.getYdOpt(x);
    p_dot = gmp.getYdDotOpt(x, x_dot);
    p_ddot = gmp.getYdDDotOpt(x, x_dot, 0);
    
%     x = t/tau;
%     p_ddot = gmp.calcYddot(x, p, p_dot, g);
%     
    t = t + dt;
%     p = p + p_dot*dt;
%     p_dot = p_dot + p_ddot*dt;
    
    if (t >= 1.05*tau), break; end
    
end


%% plot results
figure;

ax1 = subplot(3,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0, 'Color', 'blue');
plot(Time2, P_data2, 'LineWidth',2.0, 'Color', 'magenta', 'LineStyle',':');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
title(['temporal scale: $' num2str(temp_s) '$     ,     spatial scale: $' num2str(spat_s) '$'], 'interpreter','latex', 'fontsize',18);
scatter(nan, nan, 'MarkerEdgeColor',[0 0.7 0], 'Marker','o', 'MarkerEdgeAlpha',0.6, 'LineWidth',4, 'SizeData', 100); % dummy plot for legend
scatter(nan, nan, 'MarkerEdgeColor',[1 0 0], 'Marker','^', 'MarkerEdgeAlpha',0.3, 'LineWidth',2, 'SizeData', 100); % dummy plot for legend
scatter(nan, nan, 'MarkerEdgeColor',[1 0 0], 'Marker','v', 'MarkerEdgeAlpha',0.3, 'LineWidth',2, 'SizeData', 100); % dummy plot for legend
% legend({'GMP','ref','eq-constr','low-bound','upper-bound'}, 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

ax2 = subplot(3,1,2);
hold on;
plot(Time, dP_data, 'LineWidth',2.0, 'Color', 'blue');
plot(Time2, dP_data2, 'LineWidth',2.0, 'Color', 'magenta', 'LineStyle',':');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

ax3 = subplot(3,1,3);
hold on;
plot(Time, ddP_data, 'LineWidth',2.0, 'Color', 'blue');
plot(Time2, ddP_data2, 'LineWidth',2.0, 'Color', 'magenta', 'LineStyle',':');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

tic
plotConstr(pos_constr, ax1);
plotConstr(vel_constr, ax2);
plotConstr(accel_constr, ax3);
toc

%% =======================================================
%% =======================================================

function plotConstr(constr, ax)

    for i=1:length(constr)
        t = constr(i).t;
        value = constr(i).value;
        const_type = constr(i).type;
        
        if (const_type == '=') 
           marker_type = 'o';
           marker_color = [0 0.7 0];
           line_width = 4;
           alpha = 0.6;
        elseif (const_type == '<')
           marker_type = 'v';
           marker_color = [1 0 0];
           line_width = 1;
           alpha = 0.3;
        elseif (const_type == '>')
            marker_type = '^';
           marker_color = [1 0 0];
           line_width = 1;
           alpha = 0.3;
        end

        hold(ax, 'on');
        scatter(t, value, 'MarkerEdgeColor',marker_color, 'Marker',marker_type, ...
            'MarkerEdgeAlpha',alpha, 'LineWidth',line_width, 'SizeData', 100, 'Parent',ax);
    end

end

function constr = upperBoundConstr(ti, bound)

    constr = repmat(GMPConstr(), length(ti), 1);
    for i=1:length(ti)
        constr(i) = GMPConstr(ti(i),bound,'<');
    end

end

function constr = lowerBoundConstr(ti, bound)

    constr = repmat(GMPConstr(), length(ti), 1);
    for i=1:length(ti)
        constr(i) = GMPConstr(ti(i),bound,'>');
    end

end

function constr = thresholdConstr(ti, thres)
    
    constr = [upperBoundConstr(ti, thres); lowerBoundConstr(ti, -thres)];

end
