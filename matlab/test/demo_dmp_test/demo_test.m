
set_matlab_utils_path();

%% Load training data

load('data/train_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;

% for i=1:size(Pd_data,1)
%     dPd_data(i,:) = smooth(dPd_data(i,:), 'moving', 10);
%     ddPd_data(i,:) = smooth(ddPd_data(i,:), 'moving', 10);
% end

Ts = Timed(2)-Timed(1);

%% initialize DMP
a_z = 20;
b_z = a_z/4;
train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
N_kernels = [60; 60; 60];
dmp_p = DMP_pos(DMP_TYPE.STD, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);

disp('Position DMP training...')
tic
offline_train_mse = dmp_p.train(train_method, Timed, Pd_data, dPd_data, ddPd_data);
offline_train_mse
toc


%% DMP simulation
disp('DMP simulation...');
tic
P0 = Pd_data(:,1);
Pgd = Pd_data(:,end);
Pg = P0 + 1*(Pgd - P0);
T = Timed(end);
dt = Ts;

%% set initial values
t = 0.0;

x = 0.0;
dx = 0.0;

t_end = 0.5*T;

P = P0;
dP = zeros(3,1);
Z = zeros(3,1);
dZ = zeros(3,1);
ddP = zeros(3,1);
x2 = 0;
tau2 = t_end;

P2_data = [];
dP2_data = [];
P2 = P0;
dP2 = zeros(3,1);
Z2 = zeros(3,1);
dZ2 = zeros(3,1);
ddP2 = zeros(3,1);
dmp2 = dmp_p.deepCopy();
dmp2.setTau(tau2);

F_ext = zeros(3,1);
fv = 0;

dmp_p.setTau(T);

iters = 0;
Time = [];
P_data = [];
dP_data = [];
ddP_data = [];
F_ext_data = [];
Fv_data = [];
x_data = [];
x_hat_data = [];
x2_data = [];


dmp_p.setY0(P0);

d_x = 60;
k_x = 250;
a_f = 0.5;
a_tau = 100;

x_hat = x;
dx = 0;

Mp = ones(3,1)*5;
Dp = ones(3,1)*60;
Kp = ones(3,1)*250;

% stop_x = @(x) ( 1 + exp(100*(-0.001)) ) ./ ( 1 + exp(100*(x-0.001)) );
% 
% x = 0:0.01:10;
% s = stop_x(x);
% 
% figure;
% plot(x,s);
% 
% return

P_prev = P;

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    P_data = [P_data P];
    dP_data = [dP_data dP];  
    ddP_data = [ddP_data ddP];
    F_ext_data = [F_ext_data F_ext];
    Fv_data = [Fv_data fv];
    x_data = [x_data x];
    x_hat_data = [x_hat_data x_hat];
    P2_data = [P2_data P2];
    dP2_data = [dP2_data dP2];
    x2_data = [x2_data x2];

    %% DMP simulation
    % ddP = dmp_p.calcYddot(x_hat, P, dP, Pg);
    dmp_p.update(x_hat, P, Z, Pg);
    dZ = dmp_p.getZdot();
    dP = dmp_p.getYdot();
    ddP = dZ / dmp_p.getTau();
    
    % ddP2 = dmp2.calcYddot(x2, P2, dP2, Pg);
    dmp2.update(x2, P2, Z2, Pg);
    dZ2 = dmp2.getZdot();
    dP2 = dmp2.getYdot();
    dx2 = dmp2.phaseDot(x2);
    ddP2 = dZ2 / dmp2.getTau();
    
    F_ext = Mp.*(ddP2-ddP) + Dp.*(dP2-dP) + Kp.*(P2-P);
      
    
%     v2 = dP+ddP*dt;
%     if (norm(v2) > 1e-10), v = v2/norm(v2); end
% 
%     if (isnan(v))
%        
%         norm(v2)
%         v
%         pause
%         
%     end
    
%     fv = dot(v, F_ext);


    fv = norm(F_ext);
    
%     F_ext
%     fv
%     pause
    
    ddx = -d_x*dx - k_x*(x - x_hat) + a_f*fv;

    %% Update phase variable

    %% Stopping criteria
    if (t>=t_end) % && norm(y-g)<5e-3 && norm(dy)<5e-3)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    
    tau = 1/(a_tau*(x-x_hat) + 1e-16);
%     tau = ((x - x_hat) + 1e16)/dt;
%     x_err = abs(x - x_hat)
%     fv
%     pause
    
    dmp_p.setTau(tau);
    dx_hat = dmp_p.phaseDot(x_hat);
    x_hat = x_hat + dx_hat*dt;
        
    x = x + dx*dt;
    dx = dx + ddx*dt;
    
    P = P + dP*dt;
    Z = Z + dZ*dt;
    % dP = dP + ddP*dt;
    
    x2 = x2 + dx2*dt;
    P2 = P2 + dP2*dt;
    Z2 = Z2 + dZ2*dt;
    % dP2 = dP2 + ddP2*dt;

end

toc

%% Plot results
line_width = 2.5;

figure;
hold on;
plot(Time, x2_data, 'LineWidth', 2.5, 'Color','blue');
plot(Time, x_hat_data, 'LineWidth', 2.5, 'Color','magenta');
plot(Time, x_data, 'LineWidth', 2.5, 'Color','green', 'LineStyle',':');
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',20);
legend({'$x_2$', '$\hat{x}$', '$x$'}, 'interpreter','latex', 'fontsize',16);
hold off;


figure;
dim_label = {'$x$','$y$', '$z$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Timed, Pd_data(i,:), 'LineWidth', line_width, 'Color','blue');
   plot(Time, P_data(i,:), 'LineWidth', line_width, 'Color','magenta');
   plot(Time, P2_data(i,:), 'LineWidth', line_width, 'LineStyle',':', 'Color','green');
   ylabel(dim_label{i}, 'interpreter','latex', 'fontsize',15);
   if (i==1), legend({'demo', '$p$', '$p_2$'}, 'interpreter','latex', 'fontsize',15); end
   if (i==1), title('Cartesian Position', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

figure;
dim_label = {'$\dot{x}$','$\dot{y}$', '$\dot{z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Timed, dPd_data(i,:), 'LineWidth', line_width, 'Color','blue');
   plot(Time, dP_data(i,:), 'LineWidth', line_width, 'Color','magenta');
   plot(Time, dP2_data(i,:), 'LineWidth', line_width, 'LineStyle',':', 'Color','green');
   ylabel(dim_label{i}, 'interpreter','latex', 'fontsize',15);
   if (i==1), legend({'demo', '$\dot{p}$', '$\dot{p}_2$'}, 'interpreter','latex', 'fontsize',15); end
   if (i==1), title('Cartesian Velocity', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end


figure;
hold on;
plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth', line_width, 'LineStyle','-');
plot3(P2_data(1,:), P2_data(2,:), P2_data(3,:), 'LineWidth', line_width, 'LineStyle','-');
plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth', line_width, 'LineStyle','--');
legend({'sim','demo'});
hold off;

% ======== Plot forces =========
figure;
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, F_ext_data(i,:), 'LineWidth', line_width);
   axis tight;
   if (i==1), title('$F_{ext}$', 'interpreter','latex', 'fontsize',18); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17); end
   hold off;
end

figure;
plot(Time, Fv_data, 'LineWidth', line_width);
title('$f_{v}$', 'interpreter','latex', 'fontsize',18);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17);



