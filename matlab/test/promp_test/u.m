clc;
close all;
clear;

rng(0);

set_matlab_utils_path();

dt = 0.005;
tf = 4;
y0 = 0;
yf = 1;

Time = 0:dt:tf;

[yd, yd_dot, yd_ddot] = get5th1D(y0, yf, Time);



N_kernels = 5;
c = ((0:(N_kernels-1))/(N_kernels-1))';
h = 1 / ( c(2) - c(1) )^2;
w = zeros(N_kernels,1);
zero_tol = 1e-32;

kernel_fun = @(x) exp(-h*(x-c).^2);
regress_fun = @(x) kernel_fun(x) / (sum(kernel_fun(x)) + zero_tol);
output_fun =  @(x) dot(regress_fun(x), w);

n_data = length(Time);
x = Time/Time(end);
Psi = zeros(N_kernels, n_data);
for j=1:n_data
    psi = kernel_fun(x(j));
    Psi(:,j) = psi / (sum(psi) + zero_tol);
end
w = (yd/Psi)';

mu_w = w;
Sigma_w = zeros(N_kernels, N_kernels);
for i=1:N_kernels
   psi = kernel_fun(c(i)); 
   Sigma_w(i,:) = psi';
   Sigma_w(:,i) = psi;
end
Sigma_w = 0.1*Sigma_w/N_kernels;

% Sigma_w = 1e-2*diag([1 1.2 1.3 1.2 1]);
L = sqrt(Sigma_w);

n_samples = 50;
y_data = cell(n_samples,1);

for k=1:n_samples
    
    w = mu_w + L*randn(N_kernels, 1);

    y = zeros(size(yd));
    for j=1:n_data
       %y(j) =  output_fun(x(j));
       psi = kernel_fun(x(j));
       y(j) =  w'*psi / (sum(psi) + zero_tol);
    end
    
    y_data{k} = y;
end



figure;
subplot(3,1,1); hold on;
for k=1:length(y_data), plot(Time, y_data{k}, 'LineWidth',2, 'Color',[1 0 1 0.3]); end
plot(Time, yd, 'LineWidth',2, 'Color','blue', 'LineStyle','--');
ylabel('$y$', 'interpreter','latex', 'fontsize',15);
% legend({'$y$','$y_d$'}, 'interpreter','latex', 'fontsize',15);
axis tight;
subplot(3,1,2);
plot(Time, yd_dot, 'LineWidth',2, 'Color','green');
ylabel('$\dot{y}$', 'interpreter','latex', 'fontsize',15);
axis tight;
subplot(3,1,3);
plot(Time, yd_ddot, 'LineWidth',2, 'Color','magenta');
ylabel('$\ddot{y}$', 'interpreter','latex', 'fontsize',15);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
axis tight;






