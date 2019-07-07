function trainNN(a, b, tau, x0, deltat)

    setdemorandstream(491218381);

    sample_n = round(30*tau/deltat);
    [Time, X] = mackeyglassDataSet(a, b, tau, x0, deltat, sample_n);

    n_data = length(Time);

    n_lag = tau/deltat + 1;

    net = narnet(1:n_lag, [20 10]);
    T = num2cell(X');
    [Xs,Xi,Ai,Ts] = preparets(net,{},{},T);

    % figure;
    % hold on;
    % plot(Time, X, 'LineWidth',2, 'Color','blue', 'LineStyle','-');
    % plot(Time(1:n_lag), cell2mat(Xi), 'LineWidth',2, 'Color','green', 'LineStyle','--');
    % plot(Time(n_lag+1:end), cell2mat(Xs), 'LineWidth',2, 'Color','magenta', 'LineStyle','--');
    % legend({'$X$','$X_i$','$X_s$'}, 'interpreter','latex', 'fontsize',18);
    % hold off;

    net = train(net,Xs,Ts,Xi,Ai);

    [Y,Xf,Af] = net(Xs,Xi,Ai);
    % perf = perform(net,Ts,Y)
    X_hat = [X(1:n_lag); cell2mat(Y)'];

    X2_hat = zeros(1, n_data);
    xin = Xi;
    X2_hat(1:n_lag) = cell2mat(xin);
    for j=n_lag+1:n_data
        xout = net(cell(0,1),xin,Ai);
        % xout = neural_function(xin,xin);
        X2_hat(j) = xout{1};
        xin(1:end-1) = xin(2:end);
        xin{end} = xout{1};
    end

    % [netc,Xic,Aic] = closeloop(net,Xf,Af);
    % % view(netc)
    % % Y2 = netc(cell(0,n_data-n_lag),Xic,Aic);
    % Y2 = cell(1, n_data-n_lag);
    % for j=1:length(Y2)
    %     xout = netc(cell(0,j),Xic,Aic);
    %     Y2{j} = xout{j};
    %     % Xic = xout;
    % end
    % X2_hat = [X(1:n_lag); cell2mat(Y2)'];


    figure;
    hold on;
    plot(Time,X, 'LineWidth',2, 'Color','blue');
    plot(Time,X_hat, 'LineWidth',2, 'Color','magenta');
    plot(Time,X2_hat, 'LineWidth',2, 'Color','green');
    legend({'$x$','$\hat{x} (open loop)$','$\hat{x} (closed loop)$'}, 'interpreter','latex', 'fontsize',16);
    hold off;

    genFunction(net, 'mackeyGlassNN');

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

        