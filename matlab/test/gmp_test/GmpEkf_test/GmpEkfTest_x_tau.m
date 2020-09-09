

classdef GmpEkfTest_x_tau < handle
    
    methods (Static, Access = public)
        
        
        function run()
           
            addpath('../');
            set_matlab_utils_path();
            
            clc;
            % close all;
            clear;
            
            obj = GmpEkfTest_x_tau();
            
            obj.train('../data/train_data.mat');
            % obj.plotTrainResults();
            
            obj.simulate();
            obj.plotSimResults();
            
        end
        
    end
    
    methods (Access = public)
        
        function this = GmpEkfTest_x_tau()

        end
        
        function train(this, data_path)
            
            load(data_path, 'Data');
            this.train_data = struct('Time',Data.Time, 'Pos',Data.Pos(1,:), 'Vel',Data.Vel(1,:), 'Accel',Data.Accel(1,:));
            
            this.p0 = Data.Pos(1,1);
            this.g0 = Data.Pos(1,end);
            this.tau0 = Data.Time(end);

            train_method = 'LS';
            N_kernels = 30;
            kernels_std_scaling = 1;
            this.gmp = GMP(N_kernels, 80, 300, kernels_std_scaling);
            offline_train_mse = this.gmp.train(train_method, this.train_data.Time, this.train_data.Pos);
            offline_train_mse

        end
        
        function plotTrainResults(this)
          
            Timed = this.train_data.Time;
            
            Time = Timed;
            tau = Time(end);
            x = Time / tau;
            x_dot = 1 / tau;
            x_ddot = 0;
            
            n_data = length(Time);
            Pos = zeros(1,n_data);
            Vel = zeros(1,n_data);
            Accel = zeros(1,n_data);
            
            for i=1:n_data
               Pos(i) = this.gmp.getYd(x(i));
               Vel(i) = this.gmp.getYdDot(x(i),x_dot);
               Accel(i) = this.gmp.getYdDDot(x(i),x_dot,x_ddot);
            end
            
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
        
        function simulate(this)
            
            rng(0);
            
            this.g_new = this.p0 + 0.8;
            this.tau_new = 12;
            dt = 0.002;
            noise_std = 1e-14;
            
            theta0 = [this.p0; 7; 0];
            
            this.N_params = length(theta0);
            this.N_out = length( this.msrFun(theta0, struct('t',1, 'dt',1)) );
            
            Qn = diag([0.002; 0.02; 0.02]);
            Rn = 0.1 * eye(this.N_out, this.N_out);
            a_p = 1.002;
            P_theta = diag([1e2; 1e2; 1e2]);
            
            num_diff_step = [0.001; 0.001; 0.001];
            
            this.ekf = EKF(this.N_params, this.N_out, @this.stateTransFun, @this.msrFun);
            this.ekf.setProcessNoiseCov(Qn);
            this.ekf.setMeasureNoiseCov(Rn);
            this.ekf.setFadingMemoryCoeff(a_p);
            this.ekf.theta = theta0;
            this.ekf.P = P_theta;

            A_c = [0 -1 0; 0 1 0; 0 0 -1; 0 0 1];
            b_c = [0.5; 30; 0; 1];
            this.ekf.enableParamsContraints(true);
            this.ekf.setParamsConstraints(A_c, b_c);
            this.ekf.setPartDerivStep(num_diff_step);

            this.ekf.setStateTransFunJacob(@this.stateTransFunJacob);
            % this.ekf.setMsrFunJacob(@this.msrFunJacob);
            
            Ln = eye(this.N_out, this.N_out) * sqrt(noise_std);
            
            g = this.g_new;
            tau = this.tau_new;
            
            g_hat = theta0(1:end-2);
            tau_hat = theta0(end-1);
            x_hat = theta0(end);
            
            this.gmp.setY0(this.p0);
            this.gmp.setGoal(g);
            
            t = dt;
            
            while (true)

                x = t / tau;
                z = this.gmp.getYd(x);
                z_n = z + Ln*randn(this.N_out,1);
                
                g0 = this.gmp.getGoal();
                this.gmp.setGoal(g_hat);
                z_hat = this.gmp.getYd(x_hat);
                this.gmp.setGoal(g0);
                
                this.Time = [this.Time t];
                this.g_hat_data = [this.g_hat_data g_hat];
                this.tau_hat_data = [this.tau_hat_data tau_hat];
                this.x_hat_data = [this.x_hat_data x_hat];
                this.z_hat_data = [this.z_hat_data z_hat];
                this.tau_data = [this.tau_data tau];
                this.x_data = [this.x_data x];
                this.z_data = [this.z_data z];
                this.z_n_data = [this.z_n_data z_n];
                
                cookie = struct('t',t, 'dt',dt);
                this.ekf.correct(z_n, cookie); 
                this.ekf.predict(cookie);
                
                theta = this.ekf.theta;

                t = t + dt;
                g_hat = theta(1:end-2);
                tau_hat = theta(end-1);
                x_hat = theta(end);
                
                if (t >= tau), break; end
                
            end
            
        end
        
        function plotSimResults(this)
            
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

        function theta_next = stateTransFun(this, theta, cookie)

            theta_next = theta;
            
            tau = theta(end-1);
            x = theta(end);
            theta_next(end) = x + cookie.dt/tau;
            
        end
        
        function z = msrFun(this, theta, cookie)
            
            g = theta(1:end-2);
            tau = theta(end-1);
            x = theta(end);
            x_dot = 1/tau;
            
            g0 = this.gmp.getGoal(); % store current goal
            this.gmp.setGoal(g);
            y = this.gmp.getYd(x);
            % y_dot = this.gmp.getYdDot(x, x_dot);

            this.gmp.setGoal(g0); % restore previous goal
            
            z = y;
            % z = [y; y_dot];

        end

        function J = stateTransFunJacob(this, theta, cookie)
           
            N = length(theta);
            tau = theta(end-1);
            
            J = eye(N,N);
            J(end,end-1) = -cookie.dt/tau^2;
            
        end
        
        function J = msrFunJacob(this, theta, cookie)
            
            t = cookie.t;
            g = theta(1:end-1);
            tau = theta(end);
            x = t/tau;
            dx_dt = 1/tau;
      
            J = zeros(this.N_out, this.N_params);
            J(:,1) = this.gmp.dYd_dgoal(x, g);
            J(:,end-1) = this.gmp.dYd_dx(x, g) * dx_dt;
            J(:,end) = this.gmp.dYd_dx(x, g);

        end
        
    end
        
        
    properties (Access = private)
    
        train_data % training data
        
        N_out
        N_params
        
        p0
        g0
        tau0
        
        % actual sim params
        tau_new
        g_new
        
        gmp % GMP model
        
        ekf % observer
        
        % sim data
        Time
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