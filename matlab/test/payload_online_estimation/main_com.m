clc;
close all;
clear;

set_matlab_utils_path();

use_namespace_math_;

rng(0);

% ========= object params =========
g = [0; 0; -10];
m_o = 5;
p_o = [0.05; 0.08; 0.2];


% ========= force estimator params =========
noise_std = 1e-100;


% ========= payload estimator params =========
R = 1*eye(3,3);
Q = 0.001;
P0 = 1;
a_p = 1; % forgetting factor


% ========= robot params =========
Mo = 2*ones(3,1);
Do = 30*ones(3,1);
Ko = 150*ones(3,1);


% ========= create desired trajectory =========
Tf = 6;
dt = 0.002;
Time = 0:dt:Tf;
y0 = [0; 0; 0];
yf = [0.5; 0.7; 1.2];
[yd, yd_dot, yd_ddot] = get5thOrderPol(y0, yf, Time);

Qo = [1 0 0 0]';

Q0 = [1 0 0 0]';
Qf = 

y0 = quatLog(quatDiff(Q0, Qo));



n_steps = length(Time);


% ========= data log space allocation =========
y_data = zeros(3, n_steps);
y_dot_data = zeros(3, n_steps);
y_ddot_data = zeros(3, n_steps);
f_h_data = zeros(3, n_steps);
f_h_hat_data = zeros(3, n_steps);
f_o_hat_data = zeros(3, n_steps);
P_data = zeros(1, n_steps);
f_ext_data = zeros(3, n_steps);
f_dist_data = zeros(3, n_steps);
e_z_data = zeros(3, n_steps);



% ========= initialization =========
y = y0;
y_dot = zeros(3,1);
y_ddot = zeros(3,1);
f_h = zeros(3,1);
f_o = m_o * g;
f_ext = f_h + f_o + 0*g; % noise_std*randn(3,1);

f_h_hat = zeros(3,1);
m_o_hat = (f_ext(3) - f_h_hat(3)) / g(3); % + 1*rand();
f_o_hat = m_o_hat * g;
P = P0;

f_dist = f_ext - f_o_hat - f_h;

% ========= run loop =========
for k=1:n_steps
    
    % log data
    y_data(:, k) = y;
    y_dot_data(:, k) = y_dot;
    y_ddot_data(:, k) = y_ddot;
    f_ext_data(:, k) = f_ext;
    f_h_data(:, k) = f_h;
    f_h_hat_data(:, k) = f_h_hat;
    f_o_hat_data(:, k) = f_o_hat;
    f_dist_data(:, k) = f_dist;
    P_data(k) = P;

    % get external force
    f_h = M.*yd_ddot(:,k) + D.*yd_dot(:,k) + K.*(yd(:,k) - y);
    f_ext_prev = f_ext;
    f_ext = f_h + f_o + noise_std*randn(3,1);
    
    % update object force estimation 
    f_o_hat = m_o_hat*g;
    
    % calculate disturbance force (if our estimation is correct, this should be zero)
    f_dist = f_ext - f_o_hat - f_h_hat;
    %f_dist = f_ext - f_o_hat - f_h;

    % dynamic equations
    y_ddot = (-D.*y_dot + f_ext - f_o_hat) ./ M;
    
    % estimation: predict
    m_o_hat = m_o_hat;
    P = a_p^2*P + Q;
    
    % numerical integration
    y = y + y_dot*dt;
    y_dot = y_dot + y_ddot*dt;
    
    f_h_hat = f_h_hat + (f_ext - f_ext_prev);
    
    % estimation: correct
    z_hat = m_o_hat*g;
    z = f_ext - f_h_hat;
    H = g;
    Kg = P*H'/(H*P*H' + R);
    m_o_hat = m_o_hat + Kg*(z - z_hat);
    P = P - Kg*H*P;
 
    e_z_data(:,k) = z - z_hat;
    
end

% figure
% for i=1:3
%     subplot(3,1,i);
%     plot(Time, e_z_data(i,:));
% end

% ========= plot results =========
figure; 
subplot(2,1,1); hold on;
plot(Time, f_o_hat_data(3,:)/g(3), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
plot([Time(1) Time(end)], [m_o m_o], 'LineWidth',2, 'LineStyle','--', 'Color','magenta');
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
legend({'$\hat{m}_{o}$','$m_{o}$'}, 'interpreter','latex', 'fontsize',15);
subplot(2,1,2);
plot(Time, P_data, 'LineWidth',2, 'LineStyle','-', 'Color','red');
legend({'$P$'}, 'interpreter','latex', 'fontsize',15);

% ----------------------

figure; 
for i=1:3
    subplot(2,3,i);
    hold on;
    plot(Time, f_h_hat_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
    plot(Time, f_h_data(i,:), 'LineWidth',2, 'LineStyle','--', 'Color','magenta');
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
    if (i==1), legend({'$\hat{f}_{h}$','$f_{h}$'}, 'interpreter','latex', 'fontsize',15); end
    axis tight;
end
for i=1:3
    subplot(2,3,3+i);
    plot(Time, f_h_hat_data(i,:)-f_h_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','red');
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
    if (i==1), legend({'$\hat{f}_{h}-f_{h}$'}, 'interpreter','latex', 'fontsize',15); end
    axis tight;
end

% ----------------------

fig = figure;
fig.Position(3:4) = [870 878];
k = 1;
for i=1:3
    subplot(4,3,k); hold on;
    plot(Time, y_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
    plot(Time, yd(i,:), 'LineWidth',2, 'LineStyle','--', 'Color','magenta');
    if (i==1), legend({'$y$','$y_d$'}, 'interpreter','latex', 'fontsize',15); end
    k = k+1;
    axis tight;
end
for i=1:3
    subplot(4,3,k); hold on;
    plot(Time, y_dot_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
    plot(Time, yd_dot(i,:), 'LineWidth',2, 'LineStyle','--', 'Color','magenta');
    if (i==1), legend({'$\dot{y}$','$\dot{y}_d$'}, 'interpreter','latex', 'fontsize',15); end
    k = k+1;
    axis tight;
end
for i=1:3
    subplot(4,3,k); hold on;
    plot(Time, y_ddot_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
    plot(Time, yd_ddot(i,:), 'LineWidth',2, 'LineStyle','--', 'Color','magenta');
    if (i==1), legend({'$\ddot{y}$','$\ddot{y}_d$'}, 'interpreter','latex', 'fontsize',15); end
    k = k+1;
    axis tight;
end
for i=1:3
    subplot(4,3,k);
    plot(Time, f_dist_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','red');
    if (i==1), legend({'$f_{dist}$'}, 'interpreter','latex', 'fontsize',15); end
    k = k+1;
    axis tight;
end




