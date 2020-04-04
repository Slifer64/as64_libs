% The DMP vs GMP variable time scaling
% Simulation both with euler and ode.
classdef TestGMPvts < handle
       
    methods (Access = public)
        
        %% Constructor
        function this = TestGMPvts()
            
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
            Timed2 = this.Timed / this.kt;
            Pd_data2 = this.ks*( this.Pd_data-this.yd0 ) + this.y0;
            dPd_data2 = this.ks*this.dPd_data*this.kt;
            ddPd_data2 = this.ks*this.ddPd_data*this.kt^2;


            % Time2 = cell(3,1);
            % P_data2 = cell(3,1);
            % Pd_data2 = cell(3,1);
            % dist = zeros(3,1);
            % n_w = 0;
            % for i=1:3
            %     [dist(i), ix] = fb_dtw(P_data(i,:), Pd_data(i,:));
            %     Pd_data2{i} = Pd_data(i,ix);
            %     P_data2{i} = P_data(i,:);
            %     n_w = length(ix);
            %     Time2{i} = (0:(n_w-1))*Ts;
            %     dist(i) = dist(i) / n_w;
            % end
            % dist
            % 
            % figure;
            % for i=1:3
            % subplot(3,2, (i-1)*2+1);
            % hold on;
            % plot(Time2{i}, Pd_data2{i}, 'LineWidth',2, 'Color','blue', 'LineStyle','-');
            % plot(Time2{i}, P_data2{i}, 'LineWidth',2, 'Color','magenta', 'LineStyle',':');
            % hold off;
            % subplot(3,2, (i-1)*2+2);
            % plot(Time2{i}, P_data2{i}-Pd_data2{i}, 'LineWidth',2, 'Color','red');
            % end


            %% Plot results
            % for i=1:3
            %     figure;
            %     subplot(3,1,1);
            %     hold on;
            %     plot(Time, P_data(i,:), 'LineWidth',2.0 , 'Color','blue');
            %     plot(Timed2, Pd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
            %     ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
            %     title(['temporal scale: $' num2str(temp_s) '$     ,     spatial scale: $' num2str(spat_s) '$'], 'interpreter','latex', 'fontsize',18);
            %     legend({'sim','demo'}, 'interpreter','latex', 'fontsize',15);
            % 
            %     axis tight;
            %     hold off;
            % 
            %     subplot(3,1,2);
            %     hold on;
            %     plot(Time, dP_data(i,:), 'LineWidth',2.0, 'Color','blue');
            %     plot(Timed2, dPd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
            %     ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
            %     axis tight;
            %     hold off;
            % 
            %     subplot(3,1,3);
            %     hold on;
            %     plot(Time, ddP_data(i,:), 'LineWidth',2.0, 'Color','blue');
            %     plot(Timed2, ddPd_data(i,:), 'LineWidth',2.0, 'LineStyle',':', 'Color','magenta');
            %     ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
            %     axis tight;
            %     hold off;
            % end


            if (animated)

                figure;
                ax = subplot(2,1,1);
                hold(ax,'on');
                x_pl = plot(nan, nan, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1], 'Parent',ax);
                plot([this.Time(1) this.Time(end)], [0 1], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
                ylabel('$x$', 'interpreter','latex', 'fontsize',15);
                ax.XLim = [this.Time(1) this.Time(end)];
                ax.YLim = [0 1.2];
                hold(ax,'off');
                ax = subplot(2,1,2);
                hold(ax,'on');
                xdot_pl = plot(nan, nan, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1], 'Parent',ax);
                plot([this.Time(1) this.Time(end)], [this.xd_dot this.xd_dot], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
                ylabel('$\dot{x}$', 'interpreter','latex', 'fontsize',15);
                ax.XLim = [this.Time(1) this.Time(end)];
                ax.YLim = [min(this.x_dot_data) max(this.x_dot_data)] + 0.15*[-1 1];
                hold(ax,'off');

                %% animated 3D path
                ax = axes('Parent',figure());
                hold(ax,'on');
                plot3(Pd_data2(1,:), Pd_data2(2,:), Pd_data2(3,:), 'LineWidth',2, 'LineStyle','-', 'Color','blue', 'Parent',ax);
                plot_animated = true;
                if (plot_animated)
                    pl = plot3(nan, nan, nan, 'LineWidth',4, 'LineStyle','-', 'Marker','o', 'MarkerSize',10, 'Color','red', 'Parent',ax);
                    zc_quiv = quiver3(nan,nan,nan, nan,nan,nan, 'Parent',ax, ...
                        'LineWidth',4, 'LineStyle','-', 'MarkerFaceColor',[0 0.8 0], 'MarkerSize',5, 'MaxHeadSize',0.9, 'AutoScaleFactor',0.02);
                    for j=1:size(this.P_data,2)
                        % pl.XData = [pl.XData P_data(1,j)];
                        % pl.YData = [pl.YData P_data(2,j)];
                        % pl.ZData = [pl.ZData P_data(3,j)]; 
                        pl.XData = [this.P_data(1,j)];
                        pl.YData = [this.P_data(2,j)];
                        pl.ZData = [this.P_data(3,j)];
                        zc_quiv.XData = this.P_data(1,j); zc_quiv.YData = this.P_data(2,j); zc_quiv.ZData = this.P_data(3,j);
                        zc_quiv.UData = this.zc_data(1,j); zc_quiv.VData = this.zc_data(2,j); zc_quiv.WData = this.zc_data(3,j);

                        ax.XLim = [0 0.6000];
                        ax.YLim = [0 1.2000];
                        % ax.ZLim = z_lim;

                        x_pl.XData = [x_pl.XData this.Time(j)];
                        x_pl.YData = [x_pl.YData this.x_data(j)];
                        xdot_pl.XData = [xdot_pl.XData this.Time(j)];
                        xdot_pl.YData = [xdot_pl.YData this.x_dot_data(j)];

                        pause(0.01);
                        drawnow
                    end

                end
                plot3(this.P_data(1,:), this.P_data(2,:), this.P_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta', 'Parent',ax);
                hold(ax,'off');

            else

                figure;
                subplot(2,1,1);
                hold on;
                
                plot(this.Time, this.x_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
                plot([this.Time(1) this.Time(end)], [0 1], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
                ylabel('$x$', 'interpreter','latex', 'fontsize',15);
                hold off;
                subplot(2,1,2);
                hold on;
                plot(this.Time, this.x_dot_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.85 0.33 0.1]);
                plot([this.Time(1) this.Time(end)], [this.xd_dot this.xd_dot], 'LineWidth',2, 'LineStyle','--', 'Color','blue');
                ylabel('$\dot{x}$', 'interpreter','latex', 'fontsize',15);
                hold off;

                ax = axes('Parent',figure());
                hold(ax,'on');
                plot3(Pd_data2(1,:), Pd_data2(2,:), Pd_data2(3,:), 'LineWidth',2, 'LineStyle','-', 'Color',[0 0 1 0.5], 'Parent',ax);
                plot3(this.yg(1), this.yg(2), this.yg(3), 'LineWidth',4, 'LineStyle','-', 'Marker','o', 'MarkerSize',10, 'Color','red', 'Parent',ax);
                plot3(this.P_data(1,:), this.P_data(2,:), this.P_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta', 'Parent',ax);
                hold(ax,'off');

            end
            
        end

        
        %% ODE related functions
        function s_dot = stateTransFun(this, t, s)

            %% Extract state
            [x, x_dot, y, z, fv] = this.extractState(s);
            

            %% Update phase variable
            [x_ddot, fv_dot] = this.phaseVarUpdate(t, x_dot, fv);
         
            
            %% GMP update
            y_c = zeros(this.Dim,1);
            fdist_x = this.disturbance_function(t, this.t_end, 10); 
            fdist_y = this.disturbance_function(t, this.t_end, -8);
            fdist_z = this.disturbance_function(t, this.t_end, -9);
            this.z_c = this.apply_dist*[fdist_x; fdist_y; fdist_z];
            sx = [x; x_dot; x_ddot];
            if (x > 1 || x<0), sx = [round(x); 0; 0]; end
            this.gmp.update(sx, y, z, y_c, this.z_c);
            y_dot = this.gmp.getYdot();
            z_dot = this.gmp.getZdot();
  
            %% State derivative
            s_dot = zeros(size(s));
            s_dot(1) = x_dot;
            s_dot(2) = x_ddot;
            s_dot(3:5) = y_dot;
            s_dot(6:8) = z_dot;
            s_dot(9) = fv_dot;

        end

        % Unfolds the ode state
        function [x, x_dot, y, z, fv] = extractState(this, s)

            x = s(1);
            x_dot = s(2);
            y = s(3:5);
            z = s(6:8);
            fv = s(9);

        end
        
        % Logs data between successful ode steps
        function status = logData(this, t,s,flag)

            status = 0;

            if (~isempty(flag)), return; end

            this.Time = [this.Time t];

            for j=1:size(s,2)
                [x, x_dot, y, z] = this.extractState(s(:,j));

                y_dot = this.gmp.getYdot();
                y_ddot = this.gmp.getYddot();
                    
                this.P_data = [this.P_data y];
                this.dP_data = [this.dP_data y_dot];  
                this.ddP_data = [this.ddP_data y_ddot];
                this.x_data = [this.x_data x];
                this.x_dot_data = [this.x_dot_data x_dot];
                this.zc_data = [this.zc_data this.z_c];
            end

        end

        % Check whether to stop the ode sim
        function [value, isterminal, direction] = stopSim(this, t, s)

            [x, ~, ~, y, ~] = this.extractState(s);

            value = double(x>=1.1 && norm(y-this.yg)<5e-3);
            isterminal = 1;
            direction = 0;

        end

        
        function [x_ddot, fv_dot] = phaseVarUpdate(this, t, x_dot, fv)
            
            if (t < 0.1*this.t_end), fv2 = 5;
            elseif (t < 0.2*this.t_end), fv2 = 20;
            elseif (t < 0.3*this.t_end), fv2 = 0;
            elseif (t < 0.5*this.t_end), fv2 = -25;
            elseif (t < 0.7*this.t_end), fv2 = 10;
            elseif (t < 0.9*this.t_end), fv2 = -25;
            else, fv2 = 10;
            end

            a_v = 5;
            fv_dot = a_v*(fv2 - fv);
            x_ddot = 50*(this.xd_dot - x_dot) + fv;
            
        end
        
        
    end
    
    methods (Access = public, Static)
        
        %% Runs the test for MP with variable time scaling.
        function run(sim_type)
            
            if (nargin < 1)
                warning('No simulation type specified. Using ''euler'' by default.');
                sim_type = 'euler';
            end
            
            %% set include files
            %path = mfilename('fullpath'); ind = find(path == '/', 1, 'last'); addpath([path(1:ind) '..']);
            set_matlab_utils_path(); 
            
            
            %% create test object
            this = TestGMPvts();
            
            
            %% set params
            this.ks = 1.5; % spatial scale
            this.kt = 1.5; % temporal scale
            dt = 0.002; % euler integration step
            T_max = 7.9; % maximum simulation time
            train_method = 'LS'; % gmp train method
            D = 10; % gmp damping
            K = 50; % gmp stiffness
            N_kernels = 50; % gmp number of kernels
            this.apply_dist = true;
            animate_plot = true;
            
            
            %% Load training data
            this.loadTrainData('data/train_data.mat');
            
            
            %% initialize and train GMP
            kernels_std_scaling = 2;
            n_dof = size(this.Pd_data,1);
            this.gmp = GMP_nDoF(n_dof, N_kernels, D, K, kernels_std_scaling);
            tic
            offline_train_mse = this.gmp.train(train_method, this.Timed, this.Pd_data);
            offline_train_mse
            toc
            
            
            %% init simulation
            this.Dim = size(this.Pd_data,1);
            this.yd0 = this.Pd_data(:,1);
            this.ygd = this.Pd_data(:,end);
            this.y0 = this.yd0;
            this.yg = this.y0 + this.ks*(this.ygd - this.yd0);
            this.t_end = this.Timed(end) / this.kt;
            this.xd_dot = 1 / this.t_end; % new tau changed during sim
            Time = 0:dt:T_max;

            
            %% Run simulation
            
            % set initial values
            y = this.y0;
            z = zeros(this.Dim,1);
            fv = 0;
            x = 0.0;
            x_dot = this.xd_dot;
            s0 = [x; x_dot; y; z; fv];

            this.gmp.setY0(this.y0);
            this.gmp.setGoal(this.yg);
            log_data_fun = @(t,y,flag)logData(this, t,y,flag);
            stop_sim_fun = @(t,s)stopSim(this, t,s);
            
            % run simulation depending on sim_type
            disp('Simulation...'); tic
            if (strcmpi(sim_type,'ode')==1) 
                ode_opt = odeset('OutputFcn', log_data_fun, 'Events', stop_sim_fun, 'Refine',1);
                ode15s(@(t,s)stateTransFun(this, t, s), Time, s0, ode_opt);
            elseif (strcmpi(sim_type,'euler')==1)
                euler_opt = struct('dt',dt, 'log_data_fun',log_data_fun, 'stop_sim_fun',stop_sim_fun);
                TestGMPvts.eulerSim(@(t,s)stateTransFun(this, t, s), Time, s0, euler_opt);
            else
                error(['[TestGMPvts::run]: Unsupported simulation type ''' sim_type '''...']);
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
        
        
        function fdist = disturbance_function(t, T, f_max)
    
            t1 = 0.4* 0.45*T;
            t2 = 0.4* 0.55*T;
            t3 = 0.4* 0.75*T;
            t4 = 0.4* 0.85*T;

            if (t < t1), fdist = 0;
            elseif (t < t2), fdist = f_max*(t - t1)/(t2-t1);
            elseif (t < t3), fdist = f_max;
            elseif (t < t4), fdist = f_max - f_max*(t - t3)/(t4-t3);
            else, fdist = 0;
            end

        end

    end
    
    properties (Access = public)
        
        % ==== Data to log ===
        Time
        P_data
        dP_data
        ddP_data
        x_data
        x_dot_data
        zc_data
         
        % ==== Training data ===
        Timed
        Pd_data
        dPd_data
        ddPd_data
        
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
        xd_dot % nominal desired value for \dot{x}
        z_c % GMP acceleration coupling
        apply_dist % flag indicating to apply disturbance

    end

end


