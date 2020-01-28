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
y0 = yd0;
g = spat_s*(gd - yd0) + y0;
ydot_0 = 0;
% pos_constr = [];
% vel_constr = [];
pos_constr = [struct('t',3, 'pos',0.45, 'type','='); struct('t',tau, 'pos',0.7, 'type','=')];
vel_constr = [struct('t',tau, 'vel',0.1, 'type','=')];
% add velocity inequality constraints
ti = 0:0.2:tau;
vel_thres_constr = repmat(struct('t',[], 'vel',[], 'type',[]), 2*length(ti), 1);
vel_thres = 0.12;
j = 1;
for i=1:length(ti)
    vel_thres_constr(j) = struct('t',ti(i), 'vel',vel_thres, 'type','<');
    vel_thres_constr(j+1) = struct('t',ti(i), 'vel',-vel_thres, 'type','>');
    j = j+2;
end
vel_constr = [vel_constr; vel_thres_constr];
accel_constr = [struct('t',0, 'accel',0.0, 'type','=')];
qpmp.train(tau, y0, ydot_0, g, pos_constr, vel_constr, accel_constr);


%% calculate scaled demo trajectory
Time2 = Timed / temp_s;
P_data2 = spat_s*(Pd_data - Pd_data(1)) + Pd_data(1);
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
legend({'QPMP','ref'}, 'interpreter','latex', 'fontsize',15);
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

plotPosConstr(pos_constr, ax1);
plotVelConstr(vel_constr, ax2);
plotAccelConstr(accel_constr, ax3);


function plotPosConstr(constr, ax)

    for i=1:length(constr)
        t = constr(i).t;
        p = constr(i).pos;
        type = constr(i).type;
        
        plotConstr(t, p, type, ax);
    end

end

function plotVelConstr(constr, ax)

    for i=1:length(constr)
        t = constr(i).t;
        v = constr(i).vel;
        type = constr(i).type;
        
        plotConstr(t, v, type, ax);
    end

end

function plotAccelConstr(constr, ax)

    for i=1:length(constr)
        t = constr(i).t;
        a = constr(i).accel;
        type = constr(i).type;
        
        plotConstr(t, a, type, ax);
    end

end

function plotConstr(t, p, const_type, ax)
 
    if (const_type == '=') 
       marker_type = 'o';
       marker_color = [0 0.7 0];
       line_width = 4;
    elseif (const_type == '<')
       marker_type = 'v';
       marker_color = [1 0 0];
       line_width = 2;
    elseif (const_type == '>')
        marker_type = '^';
       marker_color = [1 0 0];
       line_width = 2;
    end

    hold(ax, 'on');
    scatter(t, p, 'MarkerEdgeColor',marker_color, 'Marker',marker_type, 'LineWidth',line_width, 'SizeData', 100, 'Parent',ax);


end

