% clc;
% close all;
% clear;

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

d3Pd_data = zeros(size(Pd_data));
for i=1:size(d3Pd_data,1), d3Pd_data(i,:) = [diff(ddPd_data(i,:)) 0]/Ts; end

train_method = ProMP.LS;
can_clock_ptr = CanonicalClock();
N_kernels = 50;

pro_mp = ProMP(N_kernels, can_clock_ptr);
pro_mp.setTau(Timed(end));
train_error = pro_mp.train(train_method, Timed, Pd_data(1,:), dPd_data(1,:));
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

while (x < 1.0)
   
    p = pro_mp.output(x);
    dp = pro_mp.outputDot(x);
    
%     x_next = x + dx*dt;
%     dx_next = can_clock_ptr.getPhaseDot(x_next);
%     dp_next = pro_mp.outputDot(x_next);
%     ddp = (dp_next - dp) / dt;
    ddp = pro_mp.outputDDot(x);
    
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
