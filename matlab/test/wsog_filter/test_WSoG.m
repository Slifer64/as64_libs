clc;
% close all;
clear;

set_matlab_utils_path();

load('data/train_data.mat', 'Data');

sps = 1.0;
dim = 1;
n1 = 1;
n2 = length(Data.Time);
n_step = 3;
Timed = Data.Time(n1:n_step:n2);
Pd_data0 = Data.Pos(dim, n1:n_step:n2);
dPd_data0 = Data.Vel(dim, n1:n_step:n2);
ddPd_data0 = Data.Accel(dim, n1:n_step:n2);

demo0_data = struct('Time',Timed, 'Pos',Pd_data0, 'Vel',dPd_data0, 'Accel',ddPd_data0, 'Jerk',[]);

fields = fieldnames(demo0_data)
checkStructFields(demo0_data)

return

Ts = Timed(2) - Timed(1);

a = max(abs(Pd_data0));
Pd_data = Pd_data0 + 0*a*0.005*randn(size(Pd_data0));
dPd_data = [diff(Pd_data) 0]/Ts; 
ddPd_data = [diff(dPd_data) 0]/Ts; 
d3Pd_data = [diff(ddPd_data) 0]/Ts; 

train_method = DMP_TRAIN.LS;
can_clock_ptr = CanonicalClock();
% shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
shape_attr_gat_ptr = LinGatingFunction(1.0, 1.0);
% shape_attr_gat_ptr = ExpGatingFunction(1.0, 0.02);
N_kernels = 40;

wsog = WSoG(N_kernels, @shape_attr_gat_ptr.getOutput);
tau = Timed(end);
can_clock_ptr.setTau(tau);
x = can_clock_ptr.getPhase(Timed);
% train_error = wsog.train(train_method, x, Pd_data);
% train_error = wsog.train_qp_constr(x, Pd_data, tau, 0.2, 100.0*Ts);
% train_error = wsog.train_cvx_constr(x, Pd_data, tau, 0.2, 100000.0*Ts);

% w_v = 0.01;
% w_a = 0*0.0001;
% train_error = wsog.train_qp_smooth(x, Pd_data, tau, w_v, w_a);
% 
% [P_data, dP_data, ddP_data] = wsog.simulate(x, 1/tau, 0);
% Time = x*tau;
% plotDemoSim(Time, P_data, dP_data, ddP_data, Timed, Pd_data0, dPd_data0, ddPd_data0);
% 
% abs_dP = abs(dP_data);
% max_v = max(abs_dP);
% min_v = min(abs_dP);
% temp = 1 + (abs_dP-min_v)*(4 - 1)/(max_v-min_v);
% Wv = w_v*diag(1./temp);
% 
% abs_ddP = abs(ddP_data);
% max_a = max(abs_ddP);
% min_a = min(abs_ddP);
% temp = 1 + (abs_ddP-min_a)*(4 - 1)/(max_a-min_a);
% Wa = w_a*diag(1./temp);
% 
% 
% train_error = wsog.train_qp_smooth(x, Pd_data, tau, Wv, Wa);

train_error = wsog.train_cvx(x, Pd_data, dPd_data, ddPd_data, tau, 0.5, 0.5);

train_error

[P_data, dP_data, ddP_data] = wsog.simulate(x, 1/tau, 0);
Time = x*tau;

demo0_data = struct('Time',Timed, 'Pos',Pd_data0, 'Vel',dPd_data0, 'Accel',ddPd_data0, 'Jerk',[]);
demo_data = struct('Time',Timed, 'Pos',Pd_data, 'Vel',dPd_data, 'Accel',ddPd_data, 'Jerk',[]);
sim_data = struct('Time',Timed, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'Jerk',[]);

% plotDemoSim(demo0_data, sim_data);
plotDemoSim(demo_data, sim_data);

%% ====================================
%% =========== simulation =============

% t_end = tau;
% can_clock_ptr.setTau(t_end);
% 
% Time = [];
% P_data = [];
% dP_data = [];
% ddP_data = [];
% t = 0;
% x = 0;
% dx = 0;
% ddx = 0;
% dt = Ts;
% 
% while (t <= t_end)
%    
%     p = wsog.output(x);
%     dp = wsog.outputDot(x, dx);
%     
%     x_next = x + dx*dt;
%     dx_next = can_clock_ptr.getPhaseDot(x_next);
%     dp_next = wsog.outputDot(x_next, dx_next);
%     % ddp = (dp_next - dp) / dt;
%     ddp = wsog.outputDDot(x, dx, ddx);
%     
%     dx = can_clock_ptr.getPhaseDot(x);
%     
%     Time = [Time t];
%     P_data = [P_data p];
%     dP_data = [dP_data dp];
%     ddP_data = [ddP_data ddp];
%     
%     t = t + dt;
%     x = x + dx*dt;
%     
% end

%% =====================================================
%% =====================================================

% plotDemoSim(Time, P_data, dP_data, ddP_data, Timed, Pd_data, dPd_data, ddPd_data);
% plotDemoSim(Time, P_data, dP_data, ddP_data, Timed, Pd_data0, dPd_data0, ddPd_data0);


%% =====================================================
%% =====================================================

function plotDemoSim(demo, sim)

    valid_fields = {'Time', 'Pos', 'Vel', 'Accel', 'Jerk'};
    
    checkStructFields(demo);
    checkStructFields(sim);
    
    if (isempty(demo.Time) || isempty(sim.Time))
        error('Time field must be non-empty!');
    end
    n_plots = 0;
    for i=1:valid_fields
       if () 
    end
    
    figure;
    subplot(3,1,1);
    hold on;
    plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
    plot(Timed, Pd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
    legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);
    axis tight;
    min_y = min([Pd_data]);% P_data]);
    max_y = max([Pd_data]);% P_data]);
    y_off = (max_y - min_y)*0.1;
    ylim([min_y-y_off max_y+y_off]);
    hold off;


    subplot(3,1,2);
    hold on;
    plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
    plot(Timed, dPd_data(1,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
    axis tight;
    min_y = min([dPd_data]);% dP_data]);
    max_y = max([dPd_data]);% dP_data]);
    y_off = (max_y - min_y)*0.1;
    ylim([min_y-y_off max_y+y_off]);
    hold off;

    subplot(3,1,3);
    hold on;
    plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
    plot(Timed, ddPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
    axis tight;
    min_y = min([ddPd_data]);% ddP_data]);
    max_y = max([ddPd_data]);% ddP_data]);
    y_off = (max_y - min_y)*0.1;
    ylim([min_y-y_off max_y+y_off]);
    hold off;
    
end


function checkStructFields(s, fields2)

    fields = fieldnames(s);
    
    for i=1:length(fields)
        k = find( cellfun(@(s) ~isempty(strfind(fields{i}, s)), fields2) );
        if (isempty(k))
            error('[checkStructFields]: Invalid struct field: %s', fields{i});
        end
    end

end

function plotDemoSim2(Time, P_data, dP_data, ddP_data, Timed, Pd_data, dPd_data, ddPd_data)

    figure;
    subplot(3,1,1);
    hold on;
    plot(Time, P_data, 'LineWidth',2.0 , 'Color','blue');
    plot(Timed, Pd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
    legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);
    axis tight;
    min_y = min([Pd_data]);% P_data]);
    max_y = max([Pd_data]);% P_data]);
    y_off = (max_y - min_y)*0.1;
    ylim([min_y-y_off max_y+y_off]);
    hold off;


    subplot(3,1,2);
    hold on;
    plot(Time, dP_data, 'LineWidth',2.0, 'Color','blue');
    plot(Timed, dPd_data(1,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
    axis tight;
    min_y = min([dPd_data]);% dP_data]);
    max_y = max([dPd_data]);% dP_data]);
    y_off = (max_y - min_y)*0.1;
    ylim([min_y-y_off max_y+y_off]);
    hold off;

    subplot(3,1,3);
    hold on;
    plot(Time, ddP_data, 'LineWidth',2.0, 'Color','blue');
    plot(Timed, ddPd_data, 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
    axis tight;
    min_y = min([ddPd_data]);% ddP_data]);
    max_y = max([ddPd_data]);% ddP_data]);
    y_off = (max_y - min_y)*0.1;
    ylim([min_y-y_off max_y+y_off]);
    hold off;
    
end