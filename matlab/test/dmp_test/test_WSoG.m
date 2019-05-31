clc;
close all;
clear;

set_matlab_utils_path();

load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;
Qd_data = Data.Quat;
vRotd_data = Data.RotVel;
dvRotd_data = Data.RotAccel;

Ts = Timed(2) - Timed(1);

% Timed = Timed/Timed(end)*2;
% Ts = Timed(2) - Timed(1);
% Pd_data = cos(2*pi*0.5*Timed);
% dPd_data = [diff(Pd_data) 0]/Ts;
% ddPd_data = [diff(dPd_data) 0]/Ts;


d3Pd_data = zeros(size(Pd_data));
for i=1:size(d3Pd_data,1), d3Pd_data(i,:) = [diff(ddPd_data(i,:)) 0]/Ts; end

train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
% shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
shape_attr_gat_ptr = LinGatingFunction(1.0, 1.0);
% shape_attr_gat_ptr = ExpGatingFunction(1.0, 0.02);
N_kernels = 400;

wsog = WSoG(N_kernels, @shape_attr_gat_ptr.getOutput);
can_clock_ptr.setTau(Timed(end));
x = can_clock_ptr.getPhase(Timed);
train_error = wsog.train(train_method, x, Pd_data(1,:));
train_error

t_end = Timed(end);
can_clock_ptr.setTau(t_end);

Time = [];
P_data = [];
dP_data = [];
ddP_data = [];
t = 0;
x = 0;
dx = 0;
ddx = 0;
dt = Ts;

while (t <= t_end)
   
    p = wsog.output(x);
    dp = wsog.outputDot(x, dx);
    
    x_next = x + dx*dt;
    dx_next = can_clock_ptr.getPhaseDot(x_next);
    dp_next = wsog.outputDot(x_next, dx_next);
    ddp = (dp_next - dp) / dt;
%     ddp = wsog.outputDDot(x, dx, ddx);
    
    dx = can_clock_ptr.getPhaseDot(x);
    
    Time = [Time t];
    P_data = [P_data p];
    dP_data = [dP_data dp];
    ddP_data = [ddP_data ddp];
    
    t = t + dt;
    x = x + dx*dt;
    
end

figure;
subplot(3,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed, Pd_data(1,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);
axis tight;
min_y = min(Pd_data(1,:));
max_y = max(Pd_data(1,:));
y_off = (max_y - min_y)*0.1;
ylim([min_y-y_off max_y+y_off]);
hold off;


subplot(3,1,2);
hold on;
plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, dPd_data(1,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
axis tight;
min_y = min(dPd_data(1,:));
max_y = max(dPd_data(1,:));
y_off = (max_y - min_y)*0.1;
ylim([min_y-y_off max_y+y_off]);
hold off;

subplot(3,1,3);
hold on;
plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed, ddPd_data(1,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
min_y = min(ddPd_data(1,:));
max_y = max(ddPd_data(1,:));
y_off = (max_y - min_y)*0.1;
ylim([min_y-y_off max_y+y_off]);
hold off;
