
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

tau0 = T;

%% set initial values
t = 0.0;

x = 0.0;
dx = 0.0;

w0 = -20;
wmax = 10;
fv_max = 1;
Kw = fv_max / (wmax - w0);
Kw = 2;
Dw = 10;
aw = 1;
ew = 10;

w = w0;
dw = 0;

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
x2 = x;
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
w_data = [];
x2_data = [];
v_data = [];
v2_data = [];

dmp_p.setY0(P0);

Mp = ones(3,1)*10;
Dp = ones(3,1)*60;
Kp = ones(3,1)*250;

P_prev = P;

v2 = zeros(3,1);
v = zeros(3,1);
v_prev = v;
v_thres = 1e-5;


%% simulate
while (true)

    %% data logging
    Time = [Time t];
    P_data = [P_data P];
    dP_data = [dP_data dP];  
    ddP_data = [ddP_data ddP];
    
    P2_data = [P2_data P2];
    dP2_data = [dP2_data dP2];  
    
    F_ext_data = [F_ext_data F_ext];
    Fv_data = [Fv_data fv];
    x_data = [x_data x];
    w_data = [w_data w];
    v_data = [v_data v];
    v2_data = [v2_data v2];

    %% DMP simulation
    % ddP = dmp_p.calcYddot(x_hat, P, dP, Pg);
    dmp_p.update(x, P, Z, Pg);
    dZ = dmp_p.getZdot();
    dP = dmp_p.getYdot();
    ddP = dZ / dmp_p.getTau();
    
    % ddP2 = dmp2.calcYddot(x2, P2, dP2, Pg);
%     dmp2.update(x, P, Z, Pg);
%     dZ2 = dmp2.getZdot();
%     F_ext = Mp.*(dZ2-dZ);
      
    dmp2.update(x2, P2, Z2, Pg);
    dZ2 = dmp2.getZdot();
    dP2 = dmp2.getYdot();
    dx2 = dmp2.getXdot();
    
    v2 = dP2 / (norm(dP2) + 1e-10);
    
    F_ext = Mp.*dP2;
    
    if (norm(dP) < v_thres)
        v = v_prev;
        if (norm(v) < v_thres), v = findDirection(dmp_p, P, dP, Pg, x, v_thres); end
    else, v = dP/norm(dP); end
    v_prev = v;
    
    fv = dot(v, F_ext);
    
%     if (norm(fv) < 1e-5), fv = norm(F_ext); end
    
    
    ddw = -Dw*dw -Kw*(w-w0) + aw*fv;


    %% Stopping criteria
    if (t>=t_end && norm(P-Pg)<5e-3 && norm(dP)<5e-3)
        break;
    end
    if (t>=1.2*t_end) % && norm(y-g)<5e-3 && norm(dy)<5e-3)
        warning('Time limit reached!');
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    
    w = w + dw*dt;
    dw = dw + ddw*dt;

    tau = 1 / exp(w/ew);
    dmp_p.setTau(tau);
    dx = dmp_p.phaseDot(x);
    x = x + dx*dt;

    P = P + dP*dt;
    Z = Z + dZ*dt;
    
    x2 = x2 + dx2*dt;
    P2 = P2 + dP2*dt;
    Z2 = Z2 + dZ2*dt;

end

toc

%% Plot results
line_width = 2.5;


figure;
subplot(3,1,1);
plot(Time, w_data, 'LineWidth', line_width, 'Color','blue');
ylabel('$w$', 'interpreter','latex', 'fontsize',15);
subplot(3,1,2);
plot(Time, Fv_data, 'LineWidth', line_width, 'Color','magenta');
ylabel('$f_v$', 'interpreter','latex', 'fontsize',15);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
subplot(3,1,3);
tau_data = 1 ./ exp(w_data/ew);
plot(Time, tau_data, 'LineWidth', line_width, 'Color','green');
ylabel('$\tau$', 'interpreter','latex', 'fontsize',15);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
% ylim([-1 100]);

figure;
for i=1:3
    subplot(3,1,i);
    hold on;
    plot(Time, v_data(i,:), 'LineWidth', line_width, 'Color','blue');
    plot(Time, v2_data(i,:), 'LineWidth', line_width, 'Color','magenta', 'LineStyle',':');
    if (i==1), legend({'$v$','$v_2$'}, 'interpreter','latex', 'fontsize',15); end
    if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
    ylabel(['$v_' num2str(i) '$'], 'interpreter','latex', 'fontsize',15);
    hold off;
end

% figure;
% hold on;
% plot(Time, x2_data, 'LineWidth', 2.5, 'Color','blue');
% plot(Time, x_hat_data, 'LineWidth', 2.5, 'Color','magenta');
% plot(Time, x_data, 'LineWidth', 2.5, 'Color','green', 'LineStyle',':');
% xlabel('time [$s$]', 'interpreter','latex', 'fontsize',20);
% legend({'$x_2$', '$\hat{x}$', '$x$'}, 'interpreter','latex', 'fontsize',16);
% hold off;


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



function v = findDirection(dmp, P, dP, Pg, x, v_thres)

tau0 = dmp.getTau();
dt = 0.002;

tau = 1.0;
dmp.setTau(tau);
Z = dP*tau;

while (norm(dP) < v_thres)

    dmp.update(x, P, Z, Pg);
    dZ = dmp.getZdot();
    dP = dmp.getYdot();
    
    P = P + dP*dt;
    Z = Z + dZ*dt;
    
end

dmp.setTau(tau0);

v = dP / norm(dP);

end
