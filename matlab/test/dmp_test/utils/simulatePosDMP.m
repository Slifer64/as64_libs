function [Time, P_data, dP_data, ddP_data] = simulatePosDMP(dmp_p, P0, Pg, T, dt)
%% Simulates a dmp encoding Cartesian orientation usning unit quaternions.


%% set initial values
t = 0.0;

x = 0.0;
dx = 0.0;

P = P0;
dP = zeros(3,1);
ddP = zeros(3,1);

t_end = T;
dmp_p.setTau(t_end);

iters = 0;
Time = [];
P_data = [];
dP_data = [];
ddP_data = [];
x_data = [];


dmp_p.setY0(P0);

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    P_data = [P_data P];
    dP_data = [dP_data dP];  
    ddP_data = [ddP_data ddP];
    % x_data = [x_data x];

    %% DMP simulation
    ddP = dmp_p.calcYddot(x, P, dP, Pg);

    %% Update phase variable
    dx = dmp_p.phaseDot(x);

    %% Stopping criteria
    if (t>=t_end) % && norm(y-g)<5e-3 && norm(dy)<5e-3)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + dx*dt;
    P = P + dP*dt;
    dP = dP + ddP*dt;

end


end

