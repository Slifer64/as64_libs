% The DMP vs GMP variable time scaling
% Simulation both with euler and ode.
classdef TestMPvts < handle
       
    methods (Access = public)
        
        %% Constructor
        function this = TestMPvts()
            
        end
        
        
        %% Loads the training data
        function loadTrainData(this, path)
            
            load(path, 'Data');
            this.Timed = Data.Time;
            this.Pd_data = Data.Pos;
            this.dPd_data = Data.Vel;
            this.ddPd_data = Data.Accel;
            
        end
        

        %% Plots the simulation results
        function plotResults(this, animated)
            
            if (nargin < 2), animated = false; end
            
            %% Reference trajectory (scaled)
            %Timed2 = this.Timed / this.kt;
            Pd_data2 = this.ks*( this.Pd_data-this.yd0 ) + this.y0;
            %dPd_data2 = this.ks*this.dPd_data*this.kt; ??? kt = ?
            %ddPd_data2 = this.ks*this.ddPd_data*this.kt^2;

            [dist, i_pd, i_p] = dtw(Pd_data2, this.P_data);
            N2 = length(i_p);
            dist = dist / N2;

            Time2 = 0:(length(i_p)-1);
            Pd_data2 = Pd_data2(:, i_pd');
            P_data2 = this.P_data(:, i_p');

            figure;
            for i=1:3
                subplot(3,2, (i-1)*2+1);
                hold on;
                plot(Time2, Pd_data2(i,:), 'LineWidth',2, 'Color','blue', 'LineStyle','-');
                plot(Time2, P_data2(i,:), 'LineWidth',2, 'Color','magenta', 'LineStyle',':');
                axis tight;
                hold off;
                subplot(3,2, (i-1)*2+2);
                plot(Time2, P_data2(i,:)-Pd_data2(i,:), 'LineWidth',2, 'Color','red');
                axis tight;
            end


%             figure;
%             subplot(3,1,1);
%             hold on;
%             plot(this.Time, this.x_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
%             plot([this.Time(1) this.Time(end)], [0 this.Time(end)/this.t_end], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
%             ylabel('$x$', 'interpreter','latex', 'fontsize',15);
%             axis tight;
%             hold off;
%             subplot(3,1,2);
%             hold on;
%             plot(this.Time, this.x_dot_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
%             plot([this.Time(1) this.Time(end)], [1/this.t_end 1/this.t_end], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
%             ylabel('$\dot{x}$', 'interpreter','latex', 'fontsize',15);
%             axis tight;
%             hold off;
%             subplot(3,1,3);
%             hold on;
%             plot(this.Time, this.tau_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
%             ylabel('$\tau$', 'interpreter','latex', 'fontsize',15);
%             axis tight;
%             hold off;
            
            %% 3D path
            ax = axes('Parent',figure());
            hold(ax,'on');
            plot3(Pd_data2(1,:), Pd_data2(2,:), Pd_data2(3,:), 'LineWidth',2, 'LineStyle','-', 'Color',[0 0 1 0.5], 'Parent',ax);
            plot3(this.yg(1), this.yg(2), this.yg(3), 'LineWidth',4, 'LineStyle','-', 'Marker','o', 'MarkerSize',10, 'Color','red', 'Parent',ax);
            if (animated)
                pl = plot3(nan, nan, nan, 'LineWidth',2, 'LineStyle',':', 'Color','magenta', 'Parent',ax);
                pl_ee = plot3(nan, nan, nan, 'LineWidth',3, 'LineStyle','-', 'Marker','+', 'MarkerSize',14, 'Color',[0 0.8 0], 'Parent',ax);
                for j=1:size(this.P_data,2)
                   pl.XData = [pl.XData this.P_data(1,j)]; pl.YData = [pl.YData this.P_data(2,j)]; pl.ZData = [pl.ZData this.P_data(3,j)];
                   pl_ee.XData = this.P_data(1,j); pl_ee.YData = this.P_data(2,j); pl_ee.ZData = this.P_data(3,j);
                   drawnow;
                   pause(0.001);
                end
            else
                plot3(this.P_data(1,:), this.P_data(2,:), this.P_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta', 'Parent',ax);       
            end
            hold(ax,'off');
            
            figure;
            subplot(2,1,1);
            plot(this.Time, this.D_data, 'LineWidth',2, 'LineStyle','-');
            title('Damping', 'interpreter','latex', 'fontsize',15);
            xlabel('time [s]', 'interpreter','latex', 'fontsize',15);
            subplot(2,1,2);
            plot(this.Time, this.K_data, 'LineWidth',2, 'LineStyle','-');
            title('Stiffness', 'interpreter','latex', 'fontsize',15);
            xlabel('time [s]', 'interpreter','latex', 'fontsize',15);
            
            
            figure;
            subplot(2,1,1);
            hold on;
            plot(this.Time, this.tau_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
            ylabel('$\tau$', 'interpreter','latex', 'fontsize',15);
            axis tight;
            hold off;
            subplot(2,1,2);
            hold on;
            plot(this.Time, this.tau_dot_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
            ylabel('$\dot{\tau}$', 'interpreter','latex', 'fontsize',15);
            axis tight;
            hold off;
            
            %             figure;
%             subplot(4,1,1);
%             hold on;
%             plot(this.Time, this.P_data(1,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
%             ylabel('$x$', 'interpreter','latex', 'fontsize',15);
%             axis tight;
%             hold off;
%             subplot(4,1,2);
%             hold on;
%             plot(this.Time, this.dP_data(1,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
%             ylabel('$\dot{y}$', 'interpreter','latex', 'fontsize',15);
%             axis tight;
%             hold off;
%             subplot(4,1,3);
%             hold on;
%             plot(this.Time, this.ddP_data(1,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue');
%             ylabel('$\ddot{y}$', 'interpreter','latex', 'fontsize',15);
%             axis tight;
%             hold off;
%             subplot(4,1,4);
%             hold on;
%             plot(this.Time, this.x_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
%             ylabel('$x$', 'interpreter','latex', 'fontsize',15);
%             axis tight;
%             hold off;
%             return
            
        end
        

        %% ODE related functions
        function s_dot = stateTransFun(this, t, s)

            %% Extract state
            [x, x_dot, tau, y, z] = this.extractState(s);
            
            %% Update phase variable
            this.tau_dot = this.a_tau*(this.tau2 - tau);
            this.x_ddot = -this.tau_dot/tau^2;

            %% DMP simulation
            y_c = zeros(this.Dim,1);
            z_c = zeros(this.Dim,1);
%             if (t>0.5 & t<0.55)
%                 z_c = 20*ones(this.Dim,1);
%             end
            [y_dot, z_dot] = this.model.update(x, x_dot, this.x_ddot, y, z, this.yg, tau, this.tau_dot, y_c, z_c);
  
            %% State derivative
            s_dot = zeros(size(s));
            s_dot(1) = x_dot;
            s_dot(2) = this.x_ddot;
            s_dot(3) = this.tau_dot;
            s_dot(4:6) = y_dot;
            s_dot(7:9) = z_dot;

        end

        % Unfolds the ode state
        function [x, x_dot, tau, y, z] = extractState(this, s)

            x = s(1);
            x_dot = s(2);
            tau = s(3);
            y = s(4:6);
            z = s(7:9);

        end
        
        % Logs data between successful ode steps
        function status = logData(this, t,s,flag)

            status = 0;

            if (~isempty(flag)), return; end
            
            % t = t(end);
            % s = s(:,end);
            % Or use: odeset(..., 'Refine',1);

            this.Time = [this.Time t];

            for j=1:size(s,2)
                [x, x_dot, tau, y, z] = this.extractState(s(:,j));

                [y_dot, y_ddot] = this.model.getVelAccel();

                this.P_data = [this.P_data y];
                this.dP_data = [this.dP_data y_dot];
                this.ddP_data = [this.ddP_data y_ddot];
                this.x_data = [this.x_data x];
                this.x_dot_data = [this.x_dot_data x_dot];
                this.tau_data = [this.tau_data tau];
                this.tau_dot_data = [this.tau_dot_data this.tau_dot];
                
                [D_, K_] = this.model.getImpParams(tau, this.tau_dot);
                this.D_data = [this.D_data D_];
                this.K_data = [this.K_data K_];
            end

        end

        % Check whether to stop the ode sim
        function [value, isterminal, direction] = stopSim(this, t, s)

            [x, ~, ~, y, ~] = this.extractState(s);

            value = double(x>=1.1 && norm(y-this.yg)<5e-3);
            isterminal = 1;
            direction = 0;

        end

    end
    
    methods (Access = public, Static)
        
        %% Runs the test for MP with variable time scaling.
        function run(model_type, sim_type)
            
            if (nargin < 2)
                warning('No simulation type specified. Using ''euler'' by default.');
                sim_type = 'euler';
            end
            
            %% set include files
            path = mfilename('fullpath');
            ind = find(path == '/', 1, 'last');
            addpath([path(1:ind) '..']);
            set_matlab_utils_path(); 
            
            
            %% create test object
            this = TestMPvts();
            
            
            %% set params
            this.ks = 1.5; % spatial scale
            kt = 0.01; % temporal scale
            kt_new = 5; % target temporal scale
            this.a_tau = 10;
            dt = 0.002; % euler integration step
            T_max = 8; % maximum simulation time
            train_params = struct('train_method','LS', 'D',30, 'K',100, 'N_kernels',50);
            animate_plot = false;
            
            
            %% set MP model
            if (strcmpi(model_type,'DMP')==1), this.model = DMPvts();
            elseif (strcmpi(model_type,'GMP')==1), this.model = GMPvts();
                elseif (strcmpi(model_type,'DMPimp')==1), this.model = DmpImpVts();
            else, error(['[TestMPvts::run]: Unsupported model type ''' model_type '''...']);
            end
               
            
            %% Load training data
            this.loadTrainData('../data/train_data.mat');
            
            
            %% Model training
            disp('Model training...'); tic
            offline_train_mse = this.model.train(train_params, this.Timed, this.Pd_data, this.dPd_data, this.ddPd_data);
            offline_train_mse
            toc
            
            
            %% init simulation
            this.Dim = size(this.Pd_data,1);
            this.yd0 = this.Pd_data(:,1);
            this.ygd = this.Pd_data(:,end);
            this.y0 = this.yd0;
            this.yg = this.y0 + this.ks*(this.ygd - this.yd0);
            this.t_end = this.Timed(end) / kt;
            this.tau2 = this.Timed(end) / kt_new; % new tau changed during sim
            Time = 0:dt:min(T_max,this.t_end);

            
            %% Run simulation
            
            % set initial values
            y = this.y0;
            z = zeros(this.Dim,1);
            tau = this.t_end;
            x = 0.0;
            x_dot = 1/tau;
            this.x_ddot = 0;
            s0 = [x; x_dot; tau; y; z];

            this.model.setY0(this.y0);
            log_data_fun = @(t,y,flag)logData(this, t,y,flag);
            stop_sim_fun = @(t,s)stopSim(this, t,s);
            
            % run simulation depending on sim_type
            disp('Simulation...'); tic
            if (strcmpi(sim_type,'ode')==1) 
                ode_opt = odeset('OutputFcn', log_data_fun, 'Events', stop_sim_fun, 'Refine',1);
                ode15s(@(t,s)stateTransFun(this, t, s), Time, s0, ode_opt);
            elseif (strcmpi(sim_type,'euler')==1)
                euler_opt = struct('dt',dt, 'log_data_fun',log_data_fun, 'stop_sim_fun',stop_sim_fun);
                TestMPvts.eulerSim(@(t,s)stateTransFun(this, t, s), Time, s0, euler_opt);
            else
                error(['[TestMPvts::run]: Unsupported simulation type ''' sim_type '''...']);
            end
            toc

            
            %% Plot results
            disp('Plotting...');
            this.plotResults(animate_plot);

        end
        
        
        %% Simulate with Euler integration.
        function eulerSim(update_fun, t_span, s0, euler_opt)

            dt = euler_opt.dt;
            log_data_fun = euler_opt.log_data_fun;
            stop_sim_fun = euler_opt.stop_sim_fun;
            
            t0 = t_span(1);
            tf = t_span(end);
            t = t0;
            s = s0;

            while (t <= tf)

                s_dot = update_fun(t,s);
                
                if (~isempty(log_data_fun)), log_data_fun(t,s,[]); end
                if (~isempty(stop_sim_fun) && stop_sim_fun(t,s)), break; end
                
                t = t + dt;
                s = s + s_dot*dt;  
            end

        end
        
        

    end
    
    properties (Access = public)
        
        Time
        P_data
        dP_data
        ddP_data
        x_data
        x_dot_data
        tau_data
        tau_dot_data
        D_data
        K_data
         
        % ==== Training data ===
        Timed
        Pd_data
        dPd_data
        ddPd_data
        
        % ==== MP model ====
        model
        
        % ==== Simulation params =====
        ks % spatial scaling
        % kt % temporal scaling
        yd0 % demo init pos
        ygd % demo goal
        y0 % sim init pos
        yg % sim goal
        t_end % nominal time duration for scaled motion
        Dim % number of DoFs in training data
        tau2 % new value of tau changed during sim
        x_ddot
        tau_dot
        a_tau % exponential convergence rate of tau

    end

end


