clc;
close all;
clear;

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);

taud = Timed(end);
yd0 = Pd_data(1);
gd = Pd_data(end);
temp_s = 1; % temporal scaling
spat_s = 5; % spatial scaling
tau = taud/temp_s;
y0 = yd0 + 0.2;
g = spat_s*(gd - yd0) + y0;

%% calculate scaled demo trajectory
Time2 = Timed / temp_s;
P_data2 = spat_s*(Pd_data - Pd_data(1)) + y0;
P_data3 = -spat_s*(gd - Pd_data) + g;

norm(P_data2-P_data3)

%% plot results
figure;
hold on;
plot(Time2, P_data2, 'LineWidth',2.0, 'Color', 'blue');
plot(Time2, P_data3, 'LineWidth',2.0, 'Color', 'magenta', 'LineStyle',':');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
title(['temporal scale: $' num2str(temp_s) '$     ,     spatial scale: $' num2str(spat_s) '$'], 'interpreter','latex', 'fontsize',18);
legend({'$p_2$','$p_3$'}, 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;



%% ============================================================
%% ============================================================

Qd_data = Data.Quat;
vRotd_data = Data.RotVel;
dvRotd_data = Data.RotAccel;


Q0d = Qd_data(:,1);
Qgd = Qd_data(:,end);
Q0 = Q0d;
% Q0 = quatProd(rotm2quat(rotx(80)*rotz(40))', Q0d);
ks = [4.0; 3.0; 2.5];
e0 = ks.*quatLog( quatProd( Qgd, quatInv(Q0d) ) );
Qg = quatProd(quatExp(e0), Q0);
T = 1.0*Timed(end);
dt = Ts;

ks = quatLog( quatProd( Qg, quatInv(Q0) ) ) ./ quatLog( quatProd( Qgd, quatInv(Q0d) ) );

ks

Qd_data2 = Qd_data;
for j=1:size(Qd_data,2)
   Qd_data2(:,j) = quatProd( quatExp(ks.*quatLog( quatProd(Qd_data(:,j),quatInv(Q0d))) ), Q0);
end

Qd_data3 = Qd_data;
for j=1:size(Qd_data,2)
   Qd_data3(:,j) = quatProd( quatInv(quatExp(ks.*quatLog( quatProd(Qgd, quatInv(Qd_data(:,j)))) )), Qg);
end

figure;
for i=1:4
   subplot(4,1,i);
   hold on;
   plot(Timed, Qd_data2(i,:), 'LineWidth', 2, 'LineStyle','-');
   plot(Timed, Qd_data3(i,:), 'LineWidth', 2, 'LineStyle',':');
   legend({'$Q_2$','$Q_3$'}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Unit Quaternion', 'interpreter','latex', 'fontsize',17); end
   if (i==4), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end



