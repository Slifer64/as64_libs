clc;
close all;
clear;


K = 1;
D = 0.8;

tau = 835;
tau2 = 1.67;
a_tau = 2;
dtau = 0;


e = 0.1;
de = 0;
dde = 0;

t = 0;
dt = 0.002;

Time = [];
e_data = [];
de_data = [];
dde_data = [];
tau_data = [];
dtau_data = [];
D_data = [];
K_data = [];

ti = 0;

while (true)
    
    dtau = a_tau*(tau2 - tau);
    K_ = K / tau^2;
    D_ = (D + dtau) / tau;
    dde = -D_*de - K_*e;
    
    if (D_ <= 0), ti = t; end
    
    Time = [Time t];
    e_data = [e_data e];
    de_data = [de_data de];
    dde_data = [dde_data dde];
    tau_data = [tau_data tau];
    dtau_data = [dtau_data dtau];
    D_data = [D_data D_];
    K_data = [K_data K_];
    
    
    if (t >= 8), break; end
    
    t = t + dt;
    e = e + de*dt;
    de = de + dde*dt;
    tau = tau + dtau*dt;
    
end


figure;
subplot(3,1,1); hold on;
plot(Time, e_data, 'LineWidth',2, 'Color','blue');
plot([ti ti], ylim, 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
ylabel('$e$', 'interpreter','latex', 'fontsize',15); 
axis tight; hold off;
subplot(3,1,2); hold on;
plot(Time, de_data, 'LineWidth',2, 'Color','green');
plot([ti ti], ylim, 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
ylabel('$\dot{e}$', 'interpreter','latex', 'fontsize',15);
axis tight; hold off;
subplot(3,1,3); hold on;
plot(Time, dde_data, 'LineWidth',2, 'Color','red');
plot([ti ti], ylim, 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
ylabel('$\ddot{e}$', 'interpreter','latex', 'fontsize',15);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
axis tight; hold off;


figure;
subplot(2,1,1); hold on;
plot(Time, tau_data, 'LineWidth',2, 'Color','blue');
plot([ti ti], ylim, 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
ylabel('$\tau$', 'interpreter','latex', 'fontsize',15);
axis tight; hold off;
subplot(2,1,2); hold on;
plot(Time, dtau_data, 'LineWidth',2, 'Color','green');
plot([ti ti], ylim, 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
ylabel('$\dot{\tau}$', 'interpreter','latex', 'fontsize',15);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
axis tight; hold off;

figure;
subplot(2,1,1); hold on;
plot(Time, D_data, 'LineWidth',2, 'Color','blue');
plot([ti ti], ylim, 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
ylabel('$damping$', 'interpreter','latex', 'fontsize',15);
axis tight; hold off;
subplot(2,1,2); hold on;
plot(Time, K_data, 'LineWidth',2, 'Color',[0.64 0.08 0.18]);
plot([ti ti], ylim, 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
ylabel('$stiffness$', 'interpreter','latex', 'fontsize',15);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
axis tight; hold off;



