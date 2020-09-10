

classdef GmpEkfTest_x_tau < handle
    
    methods (Static, Access = public)
        
        
        function run(model_type, est_x)
           
            clc;
            %close all;
            
            addpath('../');
            set_matlab_utils_path();
            
            
            
            obj = GmpEkfTest_x_tau(model_type);
            obj.est_x_ = est_x;
            
            obj.train('../data/train_data.mat');
            %obj.plotTrainResults();

            obj.execute();
            obj.plotExecResults();
            
        end
        
    end
    
    methods (Access = public)
        
        function this = GmpEkfTest_x_tau(model_type)

            if (strcmpi(model_type, 'GMP'))
                this.model = GmpModel();
            elseif (strcmpi(model_type, 'DMP'))
                this.model = DmpModel();
            else
                error(['Unsupported model: ''' model_type '''']);
            end
            
        end
        
        function train(this, data_path)
            
            load(data_path, 'Data');
            this.train_data = struct('Time',Data.Time, 'Pos',Data.Pos(1,:), 'Vel',Data.Vel(1,:), 'Accel',Data.Accel(1,:));
            
            this.p0 = Data.Pos(1,1);
            this.g0 = Data.Pos(1,end);
            this.tau0 = Data.Time(end);

            params = struct('N_kernels',30, 'damping',80, 'stiffness',300, 'kernels_std_scaling',1);
            train_method = 'LS';
            
            this.model.init(params);
            offline_train_mse = this.model.train(train_method, this.train_data);
            offline_train_mse

        end
        
        function plotTrainResults(this)
          
            Timed = this.train_data.Time;
            
            [Pos, Vel, Accel] = this.model.simulate(Timed);
            
            Time = Timed;

            figure;
            subplot(3,1,1); hold on;
            plot(Time, Pos, 'LineWidth',2, 'Color','magenta', 'LineStyle','-');
            plot(Timed, this.train_data.Pos, 'LineWidth',2, 'Color','blue', 'LineStyle','--');
            legend({'train','sim'}, 'interpreter','latex', 'fontsize',15);
            ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
            subplot(3,1,2); hold on;
            plot(Time, Vel, 'LineWidth',2, 'Color','magenta', 'LineStyle','-');
            plot(Timed, this.train_data.Vel, 'LineWidth',2, 'Color','blue', 'LineStyle','--');
            ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
            subplot(3,1,3); hold on;
            plot(Time, Accel, 'LineWidth',2, 'Color','magenta', 'LineStyle','-');
            plot(Timed, this.train_data.Accel, 'LineWidth',2, 'Color','blue', 'LineStyle','--');
            ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15);
            xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
            
        end
        
        function execute(this)
            
            rng(0);
            
            est_x = this.est_x_;
            this.g_new = this.p0 + 0.4;
            this.tau_new = 4.5;
            y0 = this.p0;
            dt = 0.002;
            noise_std = 1e-14;
            
            if (est_x)
                theta0 = [this.p0; 7; 0];
            else
                theta0 = [this.p0; 7];
            end
            
            this.N_params = length(theta0);
            this.N_out = length( this.model.msrFun(theta0, struct('t',1, 'dt',1, 'y',0, 'y_dot',0, 'est_x',est_x)) );
            
            Rn = 1 * eye(this.N_out, this.N_out);
            a_p = 1.002;
            num_diff_step = 1e-3*ones(this.N_params,1);
            
            if (est_x)
                P_theta = diag([1e2; 1e2; 1e2]);
                Qn = diag([0.002; 0.02; 0.02]);
                A_c = [0 -1 0; 0 1 0; 0 0 -1; 0 0 1];
                b_c = [0.5; 30; 0; 1];
            else
                P_theta = diag([1e2; 1e2]);
                Qn = diag([0.002; 0.02]);
                A_c = [0 -1; 0 1];
                b_c = [0.5; 30];
            end
            
            this.ekf = EKF(this.N_params, this.N_out, ...
                @(theta, cookie)stateTransFun(this.model, theta, cookie), ...
                @(theta, cookie)msrFun(this.model, theta, cookie));
            this.ekf.setProcessNoiseCov(Qn);
            this.ekf.setMeasureNoiseCov(Rn);
            this.ekf.setFadingMemoryCoeff(a_p);
            this.ekf.theta = theta0;
            this.ekf.P = P_theta;

            this.ekf.enableParamsContraints(true);
            this.ekf.setParamsConstraints(A_c, b_c);
            this.ekf.setPartDerivStep(num_diff_step);

            this.ekf.setStateTransFunJacob(@(theta, cookie)stateTransFunJacob(this.model, theta, cookie));
            %this.ekf.setMsrFunJacob(@(theta, cookie)msrFunJacob(this.model, theta, cookie));
            
            Ln = eye(this.N_out, this.N_out) * sqrt(noise_std);
            
            g = this.g_new;
            tau = this.tau_new;
            
            theta = theta0;
            
            if (est_x)
                g_hat = theta(1:end-2);
                tau_hat = theta(end-1);
                x_hat = theta(end);
            else
                g_hat = theta(1:end-1);
                tau_hat = theta(end);
                x_hat = 0;
            end
            
            this.model.setStart(this.p0);
            this.model.setGoal(g);
            
            t = 0;
            y = y0;
            y_dot = 0;
            
            while (true)

                cookie = struct('t',t, 'dt',dt, 'y',y, 'y_dot',y_dot, 'est_x',est_x);
                
                x = t / tau;  
                if (est_x)
                    theta_ = [g; tau; x];
                else
                    theta_ = [g; tau];
                end
                z = this.model.msrFun(theta_, cookie);
                z_n = z + Ln*randn(this.N_out,1);
                
                z_hat = this.model.msrFun(theta, cookie); 
                
                this.Time = [this.Time t];
                this.y_data = [this.y_data y];
                this.y_dot_data = [this.y_dot_data y_dot];
                this.g_hat_data = [this.g_hat_data g_hat];
                this.tau_hat_data = [this.tau_hat_data tau_hat];
                this.x_hat_data = [this.x_hat_data x_hat];
                this.z_hat_data = [this.z_hat_data z_hat];
                this.tau_data = [this.tau_data tau];
                this.x_data = [this.x_data x];
                this.z_data = [this.z_data z];
                this.z_n_data = [this.z_n_data z_n];
  
                this.ekf.correct(z_n, cookie); 
                this.ekf.predict(cookie);
                
                theta = this.ekf.theta;
                

                t = t + dt;
                if (est_x)
                    g_hat = theta(1:end-2);
                    tau_hat = theta(end-1);
                    x_hat = theta(end);
                else
                    g_hat = theta(1:end-1);
                    tau_hat = theta(end);
                    x_hat = x;
                end
                
                [y, y_dot] = this.model.nextState(x, y, y_dot, tau, dt);
                
                if (t >= tau), break; end
                
            end
            
        end
        
        function plotExecResults(this)
            
%             figure
%             subplot(2,1,1);
%             plot(this.Time, this.y_data, 'LineWidth',2, 'Color','blue');
%             ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15);
%             subplot(2,1,2);
%             plot(this.Time, this.y_dot_data, 'LineWidth',2, 'Color','blue');
%             ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15);
%             xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
            
            
            fig = figure;
            fig.Position(3:4) = [658 844];
            % -------------------------------------------
            ax = subplot(4,1,1); hold(ax, 'on');
            plot(this.Time, this.g_hat_data, 'LineWidth',2, 'Color', 'blue');
            axis tight;
            plot(ax.XLim, [this.g_new this.g_new], 'LineWidth',2, 'Color', 'magenta', 'LineStyle','--');
            legend({'$\hat{g}$', '$g$'}, 'interpreter','latex', 'fontsize',15);
            % -------------------------------------------
            ax = subplot(4,1,2); hold(ax, 'on');
            plot(this.Time, this.tau_hat_data, 'LineWidth',2, 'Color', [0.85 0.33 0.1]);
            plot(this.Time, this.tau_data, 'LineWidth',2, 'Color', 'cyan', 'LineStyle','--');
            axis tight;
            legend({'$\hat{\tau}$', '$\tau$'}, 'interpreter','latex', 'fontsize',15);
            xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
            % -------------------------------------------
            ax = subplot(4,1,3); hold(ax, 'on');
            plot(this.Time, this.x_hat_data, 'LineWidth',2, 'Color', 'blue');
            plot(this.Time, this.x_data, 'LineWidth',2, 'Color', 'magenta', 'LineStyle','--');
            axis tight;
            legend({'$\hat{x}$', '$x$'}, 'interpreter','latex', 'fontsize',15);
            xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
            % -------------------------------------------
            ax = subplot(4,1,4); hold(ax, 'on');
            plot(this.Time, this.z_n_data, 'LineWidth',2, 'Color', 'green', 'LineStyle','-');
            plot(this.Time, this.z_hat_data, 'LineWidth',2, 'Color', 'blue');
            plot(this.Time, this.z_data, 'LineWidth',2, 'Color', 'magenta', 'LineStyle','--');
            axis tight;
            legend({'$z_n$','$\hat{z}$', '$z$'}, 'interpreter','latex', 'fontsize',15);
            xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
            
            
        end

    end
        
        
    properties (Access = private)
    
        est_x_
        
        train_data % training data
        
        N_out
        N_params
        
        p0
        g0
        tau0
        
        % actual sim params
        tau_new
        g_new
        
        model % model encoding the motion
        
        ekf % observer
        
        % sim data
        Time
        y_data
        y_dot_data
        g_hat_data
        tau_hat_data
        x_hat_data
        z_hat_data
        z_data
        z_n_data
        tau_data
        x_data
    
    end
    
    
    
end