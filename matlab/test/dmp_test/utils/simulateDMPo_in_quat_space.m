function [Time, Q_data, rotVel_data, rotAccel_data] = simulateDMPo_in_quat_space(dmp_o, Q0, Qg, T, dt)
%% Simulates a dmp encoding Cartesian orientation usning unit quaternions.


%% set initial values
t_end = T;
tau = t_end;
dmp_o.setTau(tau);

Time = [];
Q_data = [];
rotVel_data = [];
rotAccel_data = [];

t = 0.0;
x = 0.0;
dx = 0.0;
Q = Q0;
rotVel = zeros(3,1);
rotAccel = zeros(3,1);

dmp_o.setQ0(Q0);

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    Q_data = [Q_data Q];
    rotVel_data = [rotVel_data rotVel];  
    rotAccel_data = [rotAccel_data rotAccel];

    %% DMP simulation
    yc = 0; % optional coupling for 'y' state
    zc = 0; % optional coupling for 'z' state
    tau_dot = 0;
    yc_dot = 0; % derivative of coupling for 'y' state

    rotAccel = dmp_o.calcRotAccel(x, Q, rotVel, Qg, tau_dot, yc, zc, yc_dot);

    %% Update phase variable
    dx = dmp_o.phaseDot(x);

    %% Stopping criteria   
    if (t>1.5*t_end)
        warning('Time limit reached... Stopping simulation!');
        break;
    end
    
    eo = quatLog(quatProd(Qg, quatInv(Q)));
    if (t>=t_end && norm(eo)<0.02)
        break;
    end

    %% Numerical integration
    t = t + dt;
    tau = tau + tau_dot*dt;
    x = x + dx*dt;
    Q = quatProd( quatExp(rotVel*dt), Q);
    rotVel = rotVel + rotAccel*dt;  
    
    dmp_o.setTau(tau);
    
end

end

