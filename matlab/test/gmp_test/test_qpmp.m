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
kernels_std_scaling = 1;
qpmp = QPMP(N_kernels, a_z, b_z, kernels_std_scaling);
taud = Timed(end);
yd0 = Pd_data(1);
gd = Pd_data(end);
qpmp.setDemo(ddPd_data, taud, yd0, gd);

temp_s = 1; % temporal scaling
spat_s = 1; % spatial scaling
tau = taud/temp_s;
y0 = yd0; % + 0.1;
g = spat_s*(gd - yd0) + y0;
ydot_0 = 0;
% =======  Position constraints  =======
pos_constr = [struct('t',3, 'value',0.45, 'type','='); struct('t',tau, 'value',0.7, 'type','=')];
ti = 0:0.04:tau;
pos_constr = [pos_constr; lowerBoundConstr(ti, 0.4)];
% =======  Velocity constraints  =======
vel_constr = [struct('t',tau, 'value',0.1, 'type','='); struct('t',6, 'value',-0.05, 'type','=')];
ti = 0:0.04:tau;
vel_thres = 0.12;
vel_constr = [vel_constr; thresholdConstr(ti, 0.12)];
% =======  Acceleration constraints  =======
accel_constr = [struct('t',0, 'value',0.0, 'type','='); struct('t',2, 'value',0.15, 'type','='); struct('t',tau, 'value',0, 'type','=')];
ti = 0:0.04:tau;
accel_constr = [accel_constr; upperBoundConstr(ti, 0.4); lowerBoundConstr(ti, -0.2)];
% tic
qpmp.train(tau, y0, ydot_0, g, pos_constr, vel_constr, accel_constr);
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
p_dot = 0;

dt = 0.005;
qpmp.init();

while (true)
    
    [t, p_ddot] = qpmp.getAccel(p, p_dot, dt);
    
    Time = [Time t];
    P_data = [P_data p];
    dP_data = [dP_data p_dot];
    ddP_data = [ddP_data p_ddot];
    
    p = p + p_dot*dt;
    p_dot = p_dot + p_ddot*dt;
    
    if (t >= tau), break; end
    
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
legend({'QPMP','ref','eq-constr','low-bound','upper-bound'}, 'interpreter','latex', 'fontsize',15);
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
ylabel('pos [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
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

    constr = repmat(struct('t',[], 'value',[], 'type',[]), length(ti), 1);
    for i=1:length(ti)
        constr(i) = struct('t',ti(i), 'value',bound, 'type','<');
    end

end

function constr = lowerBoundConstr(ti, bound)

    constr = repmat(struct('t',[], 'value',[], 'type',[]), length(ti), 1);
    for i=1:length(ti)
        constr(i) = struct('t',ti(i), 'value',bound, 'type','>');
    end

end

function constr = thresholdConstr(ti, thres)
    
    constr = [upperBoundConstr(ti, thres); lowerBoundConstr(ti, -thres)];

end
