function [Time, Q_data, rotVel_data, rotAccel_data] = simulateDMPeo_in_quat_space(dmp_o, Q0, Qg, T, dt)
%% Simulates a dmp encoding Cartesian orientation usning unit quaternions.


%% set initial values
t_end = T;
tau = t_end;
dmp_o.setTau(tau);

Time = [];
Q_data = [];
rotVel_data = [];
rotAccel_data = [];
rotAccel_data2 = [];

t = 0.0;
x = 0.0;
dx = 0.0;
Q = Q0;
rotVel = zeros(3,1);
rotAccel = zeros(3,1);
rotAccel2 = zeros(3,1);

T1 = [];
T2 = [];

dmp_o.setQ0(Q0);
dmp_o.setQg(Qg);

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    Q_data = [Q_data Q];
    rotVel_data = [rotVel_data rotVel];  
    rotAccel_data = [rotAccel_data rotAccel];
    rotAccel_data2 = [rotAccel_data2 rotAccel2];

    %% DMP simulation
    yc = 0; % optional coupling for 'y' state
    zc = 0; % optional coupling for 'z' state
    y = dmp_o.getY(Q);
    z = dmp_o.getZ(rotVel, Q);
    
    tic();
    dmp_o.update(x, y, z, yc, zc);
    
    tau_dot = 0;
    yc_dot = 0; % derivative of coupling for 'y' state
    rotVel = dmp_o.getRotVel(Q);
    rotAccel2 = dmp_o.getRotAccel(Q, tau_dot, yc_dot);

    T1 = [T1 toc()];
    
    tic();
    
    rotAccel = dmp_o.calcRotAccel(x, Q, rotVel, Qg, tau_dot, yc, zc, yc_dot);
    
    T2 = [T2 toc()];
    
    rotAccel
    rotAccel2
    norm(rotAccel-rotAccel2)
    pause


    %% Update phase variable
    dx = dmp_o.phaseDot(x);

    %% Stopping criteria   
    if (t>1.5*t_end)
        warning('Time limit reached... Stopping simulation!');
        break;
    end
    
    eo = DMP_eo.quat2eo(Q, Qg);
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

% figure;
% hold on;
% plot(Time, rotAccel_data(1,:), 'LineWidth',2.5, 'LineStyle','-');
% plot(Time, rotAccel_data2(1,:), 'LineWidth',2.5, 'LineStyle','--');
% legend({'$\dot{\omega}$','$\dot{\omega}_2$'}, 'interpreter','latex', 'fontsize',15);
% axis tight;
% hold off;
% 
% t1_mu = mean(T1);
% t1_std = std(T1);
% T1 = T1(T1<=t1_mu+3*t1_std);
% 
% t2_mu = mean(T2);
% t2_std = std(T2);
% T2 = T2(T2<=t2_mu+3*t2_std);
% 
% t1_total = sum(T1)
% t2_total = sum(T2)
% 
% t1_mean = t1_total/length(T1)
% t2_mean = t2_total/length(T2)
% 
% figure;
% hold on;
% plot(1000*T1, 'LineWidth',2.5, 'LineStyle','-', 'Color','blue');
% plot(1000*T2, 'LineWidth',2.5, 'LineStyle','-', 'Color','magenta');
% legend({'indirect','direct'}, 'interpreter','latex', 'fontsize',15);
% ylabel('time [$ms$]', 'interpreter','latex', 'fontsize',15);
% xlabel('iter \#', 'interpreter','latex', 'fontsize',15);
% axis tight;
% hold off;

% stop

end

