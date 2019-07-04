clc;
close all;
clear;

a = 0.2;
b = 0.1;
tau = 17;
x0 = 1.2;

deltat = 0.1;

sample_n = 20*tau/deltat; % 120000;
[T, X] = mackeyglassDataSet(a, b, tau, x0, deltat, sample_n);

n_in = 180;
net = feedforwardnet([1]);
net.numinputs = n_in;
net.inputConnect = [ones(1, n_in); zeros(1, n_in)];

n_data = length(T);
X_data = zeros(n_in,n_data);
x_in = zeros(n_in,1);
for j=1:n_data
    x_in(2:end) = x_in(1:end-1);
    x_in(1) = X(j);
    X_data(:,j) = x_in;
end
Xin = cell(n_in,1);
for i=1:n_in, Xin{i} = X_data(i,:); end            
Xout = T';

net = train(net,Xin,Xout);

X_hat = zeros(1,n_data);
x_in = zeros(n_in,1);
% for j=1:n_data
%     if (mod(j,100)==0), fprintf('%i/%i\n', j,n_data); end
%     
%     x_out = net({x_in'});
%     X_hat(j) = x_out{1};
%     
% %     x_in(2:end) = x_in(1:end-1);
% %     x_in(1) = X_hat(j);
%     
%     x_in(2:end) = x_in(1:end-1);
%     x_in(1) = X(j);
% end

X_hat = net(Xin);
X_hat = [X_hat{:}];

figure;
hold on;
plot(T,X, 'LineWidth',2, 'Color','blue');
plot(T,X_hat, 'LineWidth',2, 'Color','magenta');
legend({'$x$','$\hat{x}$'}, 'interpreter','latex', 'fontsize',16);
hold off;
            


%% =================================================================
%% =================================================================

%% This function returns dx/dt of Mackey-Glass delayed differential equation
% $$\frac{dx(t)}{dt}=\frac{ax(t-\tau)}{1+x(t-\tau)^{10}}-bx(t)$$
% 
function x_dot = mackeyglass_eq(x_t, x_t_minus_tau, a, b)
    x_dot = -b*x_t + a*x_t_minus_tau/(1 + x_t_minus_tau^10.0);
end

%% This function computes the numerical solution of the Mackey-Glass
%% delayed differential equation using the 4-th order Runge-Kutta method
% $$k_1=\Delta t \cdot mackeyglass\_eq(x(t), x(t-\tau), a, b)$$
% $$k_2=\Delta t \cdot mackeyglass\_eq(x(t+\frac{1}{2}k_1), x(t-\tau), a, b)$$
% $$k_3=\Delta t \cdot mackeyglass\_eq(x(t+\frac{1}{2}k_2), x(t-\tau), a, b)$$
% $$k_4=\Delta t \cdot mackeyglass\_eq(x(t+k_3), x(t-\tau), a, b)$$
% $$x(t+\Delta t) = x(t) + \frac{k_1}{6}+ \frac{k_2}{3} + \frac{k_3}{6} + \frac{k_4}{6}$$
%
function x_t_plus_deltat = mackeyglass_rk4(x_t, x_t_minus_tau, deltat, a, b)
    k1 = deltat*mackeyglass_eq(x_t,          x_t_minus_tau, a, b);
    k2 = deltat*mackeyglass_eq(x_t+0.5*k1,   x_t_minus_tau, a, b);
    k3 = deltat*mackeyglass_eq(x_t+0.5*k2,   x_t_minus_tau, a, b);
    k4 = deltat*mackeyglass_eq(x_t+k3,       x_t_minus_tau, a, b);
    x_t_plus_deltat = (x_t + k1/6 + k2/3 + k3/3 + k4/6);
end

%% Returns dataset from the Mackey-Glass time series for the input parameters.
% @param[in] a: value for a in eq (1)
% @param[in] b: value for b in eq (1)
% @param[in] tau: delay constant in eq (1)
% @param[in] x0: value for a in eq (1)
% @param[in] deltat: time step size (which coincides with the integration step)
% @param[in] sample_n: total no. of samples, excluding the given initial condition
%
function [T, X] = mackeyglassDataSet(a, b, tau, x0, deltat, sample_n)

    %% Main algorithm
    % * x_t             : x at instant t         , i.e. x(t)        (current value of x)
    % * x_t_minus_tau   : x at instant (t-tau)   , i.e. x(t-tau)   
    % * x_t_plus_deltat : x at instant (t+deltat), i.e. x(t+deltat) (next value of x)
    % * X               : the (sample_n+1)-dimensional vector containing x0 plus all other computed values of x
    % * T               : the (sample_n+1)-dimensional vector containing time samples
    % * x_history       : a circular vector storing all computed samples within x(t-tau) and x(t)

    time = 0;
    index = 1;
    history_length = floor(tau/deltat);
    x_history = zeros(history_length, 1); % here we assume x(t)=0 for -tau <= t < 0
    x_t = x0;

    X = zeros(sample_n+1, 1); % vector of all generated x samples
    T = zeros(sample_n+1, 1); % vector of time samples

    for i = 1:sample_n+1
        X(i) = x_t;
    %     if (mod(i-1, interval) == 0),
    %          disp(sprintf('%4d %f', (i-1)/interval, x_t));
    %     end
        if tau == 0
            x_t_minus_tau = 0.0;
        else
            x_t_minus_tau = x_history(index);
        end

        x_t_plus_deltat = mackeyglass_rk4(x_t, x_t_minus_tau, deltat, a, b);

        if (tau ~= 0)
            x_history(index) = x_t_plus_deltat;
            index = mod(index, history_length)+1;
        end
        time = time + deltat;
        T(i) = time;
        x_t = x_t_plus_deltat;
    end
end