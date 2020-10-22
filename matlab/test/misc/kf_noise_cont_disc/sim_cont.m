
%% cont KF params
R = R_;
Q = Q_;
a_p = a_p_cont;

%% Run simulation
for j=1:n_steps
    
    % get measurements
    y = msr_fun(x);
    y_msr = y + noise(:,j);
    y_hat = msr_fun(x_hat);
    
    % correct
    
    % logging
    log_data();
    
    % update
    t = t + dt;
    x = x + state_trans_fun(x)*dt;
    
    K = P*C'/R;
    x_dot_hat = state_trans_fun(x_hat) + K*(y_msr-y_hat);
    x_hat = x_hat + x_dot_hat*dt;
    
    P_dot = 2*a_p*P + A*P + P*A' + Q - K*C*P;
    P = P + P_dot*dt;

end
