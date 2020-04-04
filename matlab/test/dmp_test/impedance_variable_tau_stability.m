clc;
close all;
clear;


% syms az bz real;
% 
% A = [0 1; -az*bz -az];
% 
% P = [bz + 1/az + 1/bz,  1/(az*bz);  1/(az*bz),  (1/az + 1/(az^2*bz))];
% 
% Q = -simplify( A'*P + P*A );
% 
% P1 = P(1,1);
% P2 = simplify( P(2,2) - P(2,1)'*(1/P(1,1))*P(1,2) );



K = 100;
D = 2*sqrt(K);
az = D;
bz = K/D;


a_tau = 10;
tau = 10;
tau2 = 1;

dt = 0.002;
t = 0;
t_end = 10;
y = 1;
y_dot = 0;
z = 0;

Time = [];
y_data = [];
dy_data = [];
z_data = [];
tau_data = [];
D_data = [];
K_data = [];

while (true)

    %% stopping criteria
    if (norm(y) < 2e-2 && norm(y_dot)<1e-2), break; end
    
    
    %% update
    z_dot = ( -az*bz*y - az*z ) / tau;
    y_dot = z / tau;
    tau_dot = a_tau * ( tau2 - tau);
    
    D = (az + tau_dot) / tau;
    K = az*bz / tau^2;
    
    
    %% logging
    Time = [Time t];
    y_data = [y_data y];
    dy_data = [dy_data y_dot];
    z_data = [z_data z];
    tau_data = [tau_data tau];
    D_data = [D_data D];
    K_data = [K_data K];
    
    
    %% integration
    t = t + dt;
    y = y + y_dot*dt;
    z = z + z_dot*dt;
    tau = tau + tau_dot*dt;
   
end

k = 1;
for j=1:length(Time)
    if (D_data(j) > 0)
        k = j;
        break;
    end
end


%% ===============================================================

P = 0.5*[bz + 1/az + 1/bz,  1/(az*bz);  1/(az*bz),  (1/az + 1/(az^2*bz))];
lambda_P = eig(P);
p_min = min(lambda_P);
p_max = max(lambda_P);
tau_max = max(tau_data);
q_min = 1/tau_max;
S0 = [y_data(1); z_data(1)];
S_bound = sqrt(p_max/p_min)*norm(S0)*exp(-q_min*Time/(2*p_max));
S_norm = zeros(size(Time));
for j=1:length(S_norm), S_norm(j) = norm([y_data(j); z_data(j)]); end

figure('Position',[472 470 565 472]);
hold on;
plot(Time, S_norm, 'LineWidth',2, 'Color','blue');
plot(Time, S_bound, 'LineWidth',2, 'Color',[1 0 1 0.8], 'LineStyle','-.');
legend({'$||x||$', '$\sqrt{\frac{p_2}{p_1}}||x_0||e^{-\frac{q_1}{p_2}t}$'}, 'interpreter','latex', 'fontsize',17, 'Position',[0.571 0.453 0.317 0.156]);
title([{'state $x=[y;z]$'}; ...
    {'Lyapunov: $V = x^T P x$, $P = -A^T(t)P - PA(t) -Q(t)$'}; ...
    {'$p_1=\lambda_{min}(P)$, $p_2=\lambda_{max}(P)$, $q_1=\lambda_{min}(Q)$'}], ...
    'interpreter','latex', 'fontsize',17);
hold off;

%% ===============================================================
%  x = [y; z]
%  s = L*x;
%  V = x'Px = x'L'Lx = s's
%  V_dot <= -x'Qx = -(Lx)'*(L^{-T}*Q*L^{-1})*(Lx) = -s'*Q2*s <= -q2_min*||s||^2 = -q2_min*V
%  Q2 = inv(P')/tau,  q2_min = 1/(p_max*tau_max)
L = chol(P, 'upper');
s_data = L*[y_data; z_data];
q_min = 1/(p_max*tau_max);
p_min = 1;
p_max = 1;
S0 = s_data(:,1);
S_bound = sqrt(p_max/p_min)*norm(S0)*exp(-q_min*Time/(2*p_max));
S_norm = zeros(size(Time));
for j=1:length(S_norm), S_norm(j) = norm(s_data(:,j)); end
figure('Position',[597 416 643 548]);
hold on;
plot(Time, S_norm, 'LineWidth',2, 'Color','blue');
plot(Time, S_bound, 'LineWidth',2, 'Color',[1 0 1 0.8], 'LineStyle','-.');
legend({'$||s||$', '$||s_0||e^{-q_{2,1}t}$'}, 'interpreter','latex', 'fontsize',17, 'Position',[0.682 0.607 0.215 0.108]);
title([{'state $s=L*[y;z]$, where $L = chol(P)$'}; ...
    {'Lyapunov: $V = x^T P x = s^Ts$, $P = -A^T(t)P - PA(t) -Q(t)$'}; ...
    {'$Q_2 = P^{-T}/\tau$, $q_{2,1}=\lambda_{min}(Q_2)$'}], ...
    'interpreter','latex', 'fontsize',17);
hold off;

%% ===============================================================


label_font = 17;

figure('Position',[378, 1, 862, 973]);
ax = subplot(5,1,1);
hold on;
plot(Time, y_data, 'LineWidth',2);
plot([Time(k) Time(k)], ylim, 'LineStyle','--', 'Color','red', 'LineWidth',2);
ylabel('$y$', 'interpreter','latex', 'fontsize',label_font); 
ax.FontSize = 15; axis tight; hold off;
ax = subplot(5,1,2);
hold on;
plot(Time, z_data, 'LineWidth',2);
plot([Time(k) Time(k)], ylim, 'LineStyle','--', 'Color','red', 'LineWidth',2);
ylabel('$z$', 'interpreter','latex', 'fontsize',label_font);
ax.FontSize = 15; axis tight; hold off;
ax = subplot(5,1,3);
hold on;
plot(Time, dy_data, 'LineWidth',2);
plot([Time(k) Time(k)], ylim, 'LineStyle','--', 'Color','red', 'LineWidth',2);
ylabel('$\dot{y}$', 'interpreter','latex', 'fontsize',label_font); 
ax.FontSize = 15; axis tight; hold off;
ax = subplot(5,1,4);
hold on;
plot(Time, D_data, 'LineWidth',2);
plot([Time(k) Time(k)], ylim, 'LineStyle','--', 'Color','red', 'LineWidth',2);
plot([Time(1) Time(end)], [0 0], 'LineStyle',':', 'Color','[0.5 0.5 0.5]', 'LineWidth',2);
ylabel('$D$', 'interpreter','latex', 'fontsize',label_font); 
ax.FontSize = 15; axis tight; hold off;
ax = subplot(5,1,5);
hold on;
plot(Time, tau_data, 'LineWidth',2);
plot([Time(k) Time(k)], ylim, 'LineStyle','--', 'Color','red', 'LineWidth',2);
ylabel('$\tau$', 'interpreter','latex', 'fontsize',label_font);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font); 
ax.FontSize = 15; axis tight; hold off;

%% ===============================================================

