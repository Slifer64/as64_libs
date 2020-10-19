clc;
close all;
clear;

set_matlab_utils_path();

rng(0);

% ========= object params =========
g = [0; 0; -10];
m_o = 8;
% p_o = [0.05; 0.08; 0.2];


% ========= force estimator params =========
noise_std = 1e-100;


% ========= payload estimator params =========
R = 1000*eye(2,2);
Q = 0.001 * eye(2,2);
P0 = diag([1000; 1]);
a_p = 1.0; % forgetting factor


% ========= robot params =========
M = 2*ones(3,1);
D = 30*ones(3,1);
K = 150*ones(3,1);


% ========= create desired trajectory =========
Tf = 6;
dt = 0.002;
Time = 0:dt:Tf;
y0 = [0; 0; 0];
yf = [0.5; 0.7; 1.2];
[yd, yd_dot, yd_ddot] = get5thOrderPol(y0, yf, Time);

n_steps = length(Time);
% n_steps = round(length(Time)/2);
% Time = Time(1:n_steps);
% yd = yd(:, 1:n_steps);
% yd_dot = yd_dot(:, 1:n_steps);
% yd_ddot = yd_ddot(:, 1:n_steps);

% ========= data log space allocation =========
y_data = zeros(3, n_steps);
y_dot_data = zeros(3, n_steps);
y_ddot_data = zeros(3, n_steps);

x_data = zeros(2, n_steps);
x_hat_data = zeros(2, n_steps);
P_data = zeros(2, n_steps);

f_ext_data = zeros(3, n_steps);
f_dist_data = zeros(1, n_steps);

% ========= initialization =========
y = y0;
y_dot = zeros(3,1);
y_ddot = zeros(3,1);
f_h = zeros(3,1);
f_o = m_o * g;
f_ext = f_h + f_o + 0*g; % noise_std*randn(3,1);

f_h_hat = zeros(3,1) + [0; 0; -0*10];
m_o_hat = 0*(f_ext(3) - f_h_hat(3)) / g(3); %+ 1*rand();
f_o_hat = m_o_hat * g;
x_hat = [f_o_hat(3); f_h_hat(3)];
P = P0;

f_dist = f_ext(3) - f_o_hat(3) - f_h(3);

% ========= run loop =========
for k=1:n_steps
    
    % log data
    y_data(:, k) = y;
    y_dot_data(:, k) = y_dot;
    y_ddot_data(:, k) = y_ddot;
    f_ext_data(:, k) = f_ext;
    
    x_data(:, k) = [f_o(3); f_h(3)];
    x_hat_data(:, k) = [f_o_hat(3); f_h_hat(3)];
    P_data(:,k) = sqrt(diag(P));
    f_dist_data(:, k) = f_dist;

    % get external force
    f_h = M.*yd_ddot(:,k) + D.*yd_dot(:,k) + K.*(yd(:,k) - y);
    f_ext_prev = f_ext;
    f_ext = f_h + f_o + noise_std*randn(3,1);
    
    % update object force estimation
    f_o_hat(3) = x_hat(1);
    f_h_hat(3) = x_hat(2);
    
    % calculate disturbance force (if our estimation is correct, this should be zero)
    %f_dist = f_ext - f_o_hat - f_h_hat;
    f_dist = f_ext(3) - f_o_hat(3) - f_h(3);

    % dynamic equations
    y_ddot = (-D.*y_dot + f_ext - f_o_hat) ./ M;
    
    % estimation: predict
    x_hat = x_hat + [0; (f_ext(3) - f_ext_prev(3))];
    P = a_p^2*P + Q;
    
    % numerical integration
    y = y + y_dot*dt;
    y_dot = y_dot + y_ddot*dt;
       
    % estimation: correct
    z_hat = x_hat;
    z = f_ext(3)*ones(2,1) - [f_h_hat(3); f_o_hat(3)];
    H = eye(2,2);
    Kg = P*H'/(H*P*H' + R);
    x_hat = x_hat + Kg*(z - z_hat);
    P = P - Kg*H*P;
    
    %if (sqrt(P(1)) < 1.5), P(1) = P(1) + 1000; end
    
end

fo_hat_data = x_hat_data(1,:);
fh_hat_data = x_hat_data(2,:);


% N = length(f_ext_data(3,:));
% H = N;
% f = -sum(f_ext_data(3,:));
% A = 1;
% b = 0;
% fo2_hat = quadprog(H,f,A,b);
fo2_hat = mean(f_ext_data(3,:))
fo_hat = fo_hat_data(end)
% fo3_hat = mean(f_ext_data(3,:))

figure; hold on;
plot(Time, (f_ext_data(3,:)), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
plot(Time, abs(f_ext_data(3,:) - fo_hat_data), 'LineWidth',2, 'LineStyle','-', 'Color','magenta');
plot(Time, abs(f_ext_data(3,:) - fo2_hat), 'LineWidth',2, 'LineStyle','--', 'Color','green');

% ========= plot results =========
fig = figure;
fig.Position(3:4) = [766 746];
legend_lb_x = {'$f_o$', '$\hat{f}_o$'; '$f_h$', '$\hat{f}_h$'};
legend_lb_p = {'$\sigma_o$'; '$\sigma_h$'};
for i=1:2
    subplot(3,2,i); hold on;
    plot(Time, x_hat_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
    plot(Time, x_data(i,:), 'LineWidth',2, 'LineStyle','--', 'Color','magenta');
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
    legend(legend_lb_x(i,:), 'interpreter','latex', 'fontsize',15);
    subplot(3,2,i+2);
    plot(Time, x_data(i,:)-x_hat_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','red');
    legend({'$e$'}, 'interpreter','latex', 'fontsize',15);
    subplot(3,2,i+4);
    plot(Time, P_data(i,:), 'LineWidth',2, 'LineStyle','-', 'Color','red');
    legend(legend_lb_p(i,:), 'interpreter','latex', 'fontsize',15);
end

return 

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
    plot(Time, f_dist_data, 'LineWidth',2, 'LineStyle','-', 'Color','red');
    if (i==1), legend({'$f_{dist}$'}, 'interpreter','latex', 'fontsize',15); end
    k = k+1;
    axis tight;
end




