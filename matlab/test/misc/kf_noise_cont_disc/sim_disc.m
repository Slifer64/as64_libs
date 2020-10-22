
%% disc KF params
R = R_ / dt;
Q = Q_ * dt;
a_p = exp(a_p_cont*dt);

F = eye(2,2) + A*dt + 0.5*(A*dt)^2 + 0.5*(A*dt)^2 + (A*dt)^3/6;

%% Run simulation
for j=1:n_steps
    
    % get measurements
    y = msr_fun(x);
    y_msr = y + noise(:,j);
    y_hat = msr_fun(x_hat);
    
    % correct
    K = P*C'/(C*P*C'+R);
    x_hat = x_hat + K*(y_msr-y_hat);
    P = P - K*C*P;
    
    % logging
    log_data();
    
    % update
    t = t + dt;
    x = F*x;
    x_hat = F*x_hat;
    P = a_p^2*F*P*F' + Q;

end