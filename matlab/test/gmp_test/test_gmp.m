function test_gmp()

set_matlab_utils_path();

%% Load training data
load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos(1,:);
dPd_data = Data.Vel(1,:);
ddPd_data = Data.Accel(1,:);

Ts = Timed(2)-Timed(1);

%% initialize DMP
a_z = 20;
b_z = a_z/4;
train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
% shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
shape_attr_gat_ptr = LinGatingFunction(1.0, 1.0);
N_kernels = 50;
gmp = GMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);
tic
offline_train_mse = gmp.train(train_method, Timed, Pd_data);
offline_train_mse
toc

%% DMP simulation
disp('GMP simulation...');
tic

% ddx = 0;
% 
% wsog = WSoG(N_kernels);
% wsog.train(DMP_TRAIN.LS, Timed/Timed(end), Pd_data);
% 
% temp_s = 1;
% T = Timed(end) * temp_s;
% dx = 1/T;
% Time = Timed * temp_s;
% x = Time / T;
% P_data = [];
% dP_data = [];
% ddP_data = [];
% for i=1:length(x)
%     P = wsog.output(x(i));
%     dP = wsog.outputDot(x(i), dx);
%     ddP = wsog.outputDDot(x(i), dx, ddx);
%    
%     P_data = [P_data P];
%     dP_data = [dP_data dP];
%     ddP_data = [ddP_data ddP];
% end
% 
% 
% temp_s = 0.5;
% T = Timed(end) * temp_s;
% dx = 1/T;
% Time2 = Timed * temp_s;
% x = Time2 / T;
% P_data2 = [];
% dP_data2 = [];
% ddP_data2 = [];
% for i=1:length(x)
%     P = wsog.output(x(i));
%     dP = wsog.outputDot(x(i), dx);
%     ddP = wsog.outputDDot(x(i), dx, ddx);
%    
%     P_data2 = [P_data2 P];
%     dP_data2 = [dP_data2 dP];
%     ddP_data2 = [ddP_data2 ddP];
% end
% 
% temp_s = 0.5;
% Time3 = Time * temp_s;
% P_data3 = Pd_data;
% dP_data3 = dPd_data / temp_s;
% ddP_data3 = ddPd_data / temp_s^2;
% 
% figure;
% subplot(3,1,1);
% hold on;
% plot(Timed, Pd_data, 'Color','cyan');
% plot(Time, P_data, 'Color','blue');
% plot(Time2, P_data2, 'Color','magenta');
% plot(Time3, P_data3, 'Color','green');
% hold off;
% subplot(3,1,2);
% hold on;
% plot(Timed, dPd_data, 'Color','cyan');
% plot(Time, dP_data, 'Color','blue');
% plot(Time2, dP_data2, 'Color','magenta');
% plot(Time3, dP_data3, 'Color','green');
% hold off;
% subplot(3,1,3);
% hold on;
% plot(Timed, ddPd_data, 'Color','cyan');
% plot(Time, ddP_data, 'Color','blue');
% plot(Time2, ddP_data2, 'Color','magenta');
% plot(Time3, ddP_data3, 'Color','green');
% hold off;
% 
% return

spat_s = 2; % spatial scale
temp_s = 0.5; % temporal scale
P0 = Pd_data(1);
Pgd = Pd_data(end);
Pg = P0 + spat_s*(Pgd - P0);
T = temp_s*Timed(end);
dt = Ts;
[Time, P_data, dP_data, ddP_data] = simulateDMP(gmp, P0, Pg, T, dt);
toc

%% Plot results

figure;
subplot(3,1,1);
hold on;
plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
plot(Timed*temp_s, spat_s*(Pd_data(1,:)-P0)+P0, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(3,1,2);
hold on;
plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed*temp_s, spat_s*dPd_data(1,:)/temp_s, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;

subplot(3,1,3);
hold on;
plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
plot(Timed*temp_s, spat_s*ddPd_data(1,:)/temp_s^2, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
axis tight;
hold off;


end


