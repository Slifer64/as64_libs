% The DMP vs GMP variable time scaling
% Simulation both with euler and ode.
classdef TestMPvts < handle
       
    methods (Access = public)
        
        %% Constructor
        function this = TestMPvts()
            
        end
        
        
        %% Loads the training data
        function loadTrainData(this)
            
            load('data/train_data.mat', 'Data');
            this.Timed = Data.Time;
            this.Pd_data = Data.Pos;
            this.dPd_data = Data.Vel;
            this.ddPd_data = Data.Accel;
            
        end
        
        
        %% Trains a DMP model using the loaded training data
        function [offline_train_mse, elaps_time] = trainDMP(this)
            
            if (isempty(this.Timed)), error('The training data are empty...'); end
            
            a_z = this.D;
            b_z = this.K/this.D;
            can_clock_ptr = CanonicalClock();
            shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
            this.dmp = DMP_pos(DMP_TYPE.STD, this.N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);
            tic
            offline_train_mse = this.dmp.train(this.train_method, this.Timed, this.Pd_data, this.dPd_data, this.ddPd_data);
            elaps_time = toc;
            
        end
        
        
        %% Trains a GMP model using the loaded training data
        function [offline_train_mse, elaps_time] = trainGMP(this)
            
            if (isempty(this.Timed)), error('The training data are empty...'); end

            kernels_std_scaling = 2;
            n_dof = size(this.Pd_data,1);
            this.gmp = GMP_nDoF(n_dof, this.N_kernels, this.D, this.K, kernels_std_scaling);
            tic
            offline_train_mse = this.gmp.train(this.train_method, this.Timed, this.Pd_data);
            elaps_time = toc;
            
        end
        
        
        %% Plots the simulation results
        function plotResults(this)
            
            %% Reference trajectory (scaled)
            %Timed2 = this.Timed / this.kt;
            Pd_data2 = this.ks*( this.Pd_data-this.yd0 ) + this.y0;
            %dPd_data2 = this.ks*this.dPd_data*this.kt;
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

            ax = axes('Parent',figure());
            hold(ax,'on');
            plot3(Pd_data2(1,:), Pd_data2(2,:), Pd_data2(3,:), 'LineWidth',2, 'LineStyle','-', 'Color',[0 0 1 0.5], 'Parent',ax);
            plot3(this.yg(1), this.yg(2), this.yg(3), 'LineWidth',4, 'LineStyle','-', 'Marker','o', 'MarkerSize',10, 'Color','red', 'Parent',ax);
            plot3(this.P_data(1,:), this.P_data(2,:), this.P_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta', 'Parent',ax);
            hold(ax,'off');
            
            figure;
            plot(this.Time, this.D_data, 'LineWidth',2, 'LineStyle','-');
            title('DMP damping', 'interpreter','latex', 'fontsize',15);
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
            
        end
        
        
        %% ODE related functions
        function s_dot = stateTransFun(this, t, s, model_update_fun)

            %% Extract state
            [x, x_dot, tau, y, z] = this.extractState(s);
            
            %% Update phase variable
            this.tau_dot = 20*(this.tau2 - tau);
            this.x_ddot = -this.tau_dot/tau^2;

            %% DMP simulation
            y_c = zeros(this.Dim,1);
            z_c = zeros(this.Dim,1);
            [y_dot, z_dot] = model_update_fun(s, y_c, z_c);
  
            %% State derivative
            s_dot = zeros(size(s));
            s_dot(1) = x_dot;
            s_dot(2) = this.x_ddot;
            s_dot(3) = this.tau_dot;
            s_dot(4:6) = y_dot;
            s_dot(7:9) = z_dot;

        end

        % update function for DMP model
        function [y_dot, z_dot] = dmpUpdate(this, s, y_c, z_c)

            [x, x_dot, tau, y, z] = this.extractState(s);
            
            this.dmp.setTau(tau);
            this.dmp.update(x, y, z, this.yg, y_c, z_c);
            y_dot = this.dmp.getYdot();
            z_dot = this.dmp.getZdot();

        end
        
        % returns the dmp vel and accel (assuming a dmp update is called first).
        function [y_dot, y_ddot] = getDmpVelAccel(this)
            
            y_dot = this.dmp.getYdot();
            y_ddot = this.dmp.getYddot();
            
        end
        
        % update function for GMP model
        function [y_dot, z_dot] = gmpUpdate(this, s, y_c, z_c)

            [x, x_dot, tau, y, z] = this.extractState(s);
            
            xs = [x; x_dot; this.x_ddot];
            this.gmp.setGoal(this.yg);
            this.gmp.update(xs, y, z, y_c, z_c);
            y_dot = this.gmp.getYdot();
            z_dot = this.gmp.getZdot();

        end
        
        % returns the gmp vel and accel (assuming a gmp update is called first).
        function [y_dot, y_ddot] = getGmpVelAccel(this)
            
            y_dot = this.gmp.getYdot();
            y_ddot = this.gmp.getYddot();
            
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
        function status = logData(this,get_vel_accel_fun, t,s,flag)

            status = 0;

            if (~isempty(flag)), return; end
            
            % t = t(end);
            % s = s(:,end);
            % Or use: odeset(..., 'Refine',1);

            this.Time = [this.Time t];

            for j=1:size(s,2)
                [x, x_dot, tau, y, z] = this.extractState(s(:,j));

                [y_dot, y_ddot] = get_vel_accel_fun();

                this.P_data = [this.P_data y];
                this.dP_data = [this.dP_data y_dot];
                this.ddP_data = [this.ddP_data y_ddot];
                this.x_data = [this.x_data x];
                this.x_dot_data = [this.x_dot_data x_dot];
                this.tau_data = [this.tau_data tau];
                this.tau_dot_data = [this.tau_dot_data this.tau_dot];
                
                D_ = (this.dmp.dmp{1}.a_z + this.tau_dot) / tau;
                this.D_data = [this.D_data D_];
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
            
            set_matlab_utils_path(); % set include files
            this = TestMPvts(); % create test object
               
            
            %% Load training data
            this.loadTrainData();
            
            
            %% Model training
            this.train_method = 'LS';
            this.K = 1;
            this.D = 1;
            this.N_kernels = 50;
            disp('Model training...');
            if (strcmpi(model_type,'DMP')==1), [offline_train_mse, elaps_time] = this.trainDMP();
            elseif (strcmpi(model_type,'GMP')==1), [offline_train_mse, elaps_time] = this.trainGMP();
            else, error(['[TestMPvts::run]: Unsupported model type ''' model_type '''...']);
            end
            elaps_time
            offline_train_mse
            
            
            %% set simulation params
            this.ks = 1.5; % spatial scale
            this.kt = 0.1; % temporal scale
            this.Dim = size(this.Pd_data,1);
            this.yd0 = this.Pd_data(:,1);
            this.ygd = this.Pd_data(:,end);
            this.y0 = this.yd0;
            this.yg = this.y0 + this.ks*(this.ygd - this.yd0);
            this.t_end = this.Timed(end) / this.kt;
            this.tau2 = this.Timed(end)/5; % new tau changed during sim
            dt = 0.002; % euler integration step
            Time = 0:dt:this.t_end;

            
            %% Run simulation
            
            % set initial values
            y = this.y0;
            z = zeros(this.Dim,1);
            tau = this.t_end;
            x = 0.0;
            x_dot = 1/tau;
            this.x_ddot = 0;
            s0 = [x; x_dot; tau; y; z];
            
            % set model update function
            if (strcmpi(model_type,'DMP')==1)
                this.dmp.setY0(this.y0);
                model_update_fun = @(s, y_c, z_c)dmpUpdate(this, s, y_c, z_c);
                get_vel_accel_fun = @this.getDmpVelAccel;
            elseif (strcmpi(model_type,'GMP')==1)
                this.gmp.setY0(this.y0);
                model_update_fun = @(s, y_c, z_c)gmpUpdate(this, s, y_c, z_c);
                get_vel_accel_fun = @this.getGmpVelAccel;
            end
            log_data_fun = @(t,y,flag)logData(this,get_vel_accel_fun, t,y,flag);
            stop_sim_fun = @(t,s)stopSim(this, t,s);
            
            % run simulation depending on sim_type
            disp('Simulation...');
            tic
            if (strcmpi(sim_type,'ode')==1) 
                ode_opt = odeset('OutputFcn', log_data_fun, 'Events', stop_sim_fun, 'Refine',1);
                ode45(@(t,s)stateTransFun(this, t, s, model_update_fun), Time, s0, ode_opt);
            elseif (strcmpi(sim_type,'euler')==1)
                euler_opt = struct('dt',dt, 'log_data_fun',log_data_fun, 'stop_sim_fun',stop_sim_fun);
                TestMPvts.eulerSim(@(t,s)stateTransFun(this, t, s, model_update_fun), Time, s0, euler_opt);
            else
                error(['[TestMPvts::run]: Unsupported simulation type ''' sim_type '''...']);
            end
            toc
            
            
            %% Plot results
            this.plotResults();

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
         
        % ==== Training data ===
        Timed
        Pd_data
        dPd_data
        ddPd_data
        
        % ==== Training params ====
        train_method % training method for model
        N_kernels % number of basis functions
        K % model stiffness
        D % model damping
        
        % ==== DMP model ====
        dmp
        
        % ==== GMP model ====
        gmp
        
        
        % ==== Simulation params =====
        ks % spatial scaling
        kt % temporal scaling
        yd0 % demo init pos
        ygd % demo goal
        y0 % sim init pos
        yg % sim goal
        t_end % nominal time duration for scaled motion
        Dim % number of DoFs in training data
        tau2 % new value of tau changed during sim
        x_ddot
        tau_dot

    end

end


