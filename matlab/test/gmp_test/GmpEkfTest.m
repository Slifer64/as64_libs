

classdef GmpEkfTest < handle
    
    methods (Static, Access = public)
        
        
        function run()
           
            set_matlab_utils_path();
            
            clc;
            close all;
            clear;
            
            obj = GmpEkfTest();
            
            obj.train('data/train_data.mat');
            % obj.plotTrainResults();
            
            obj.simulate();
            obj.plotSimResults();
            
        end
        
    end
    
    methods (Access = public)
        
        function this = GmpEkfTest()

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
            
            this.g_new = this.g0;
            this.tau_new = 5;
            dt = 0.005;
            
            theta0 = [this.p0+0.1; 0.01];
            
            this.N_params = length(theta0);
            this.N_out = 1;
            
            Qn = diag([0.002; 0.02]);
            Rn = 1 * eye(this.N_out, this.N_out);
            a_p = 1.0000;
            P_theta = diag([1; 1]);
            
            num_diff_step = [0.001; 0.001];
            
            this.ekf = EKF(this.N_params, this.N_out, @this.stateTransFun, @this.msrFun);
            this.ekf.setProcessNoiseCov(Qn);
            this.ekf.setMeasureNoiseCov(Rn);
            this.ekf.setFadingMemoryCoeff(a_p);
            this.ekf.theta = theta0;
            this.ekf.P = P_theta;

            A_c = [0 -1; 0 1];
            b_c = [0; 1];
            this.ekf.enableParamsContraints(true);
            this.ekf.setParamsConstraints(A_c, b_c);
            this.ekf.setPartDerivStep(num_diff_step);

            this.ekf.setStateTransFunJacob(@this.stateTransFunJacob);
            %this.ekf.setMsrFunJacob(@this.msrFunJacob);
            
            g = this.g_new;
            tau = this.tau_new;
            
            g_hat = theta0(1:end-1);
            x_hat = theta0(end);
            
            theta = [g_hat; x_hat];
            
            this.gmp.setY0(this.p0);
            this.gmp.setGoal(g);
            
            t = dt;
            
            while (true)

                x = t / tau;
                z = this.gmp.getYd(x);
                
                g0 = this.gmp.getGoal();
                this.gmp.setGoal(g_hat);
                z_hat = this.gmp.getYd(x_hat);
                this.gmp.setGoal(g0);
                
                this.Time = [this.Time t];
                this.g_hat_data = [this.g_hat_data g_hat];
                this.x_hat_data = [this.x_hat_data x_hat];
                this.z_hat_data = [this.z_hat_data z_hat];
                this.x_data = [this.x_data x];
                this.z_data = [this.z_data z];
                
                cookie = struct('t',t, 'dt',dt);
%                 this.ekf.correct(z, cookie);
                this.ekf.predict(cookie);
                
                theta = this.ekf.theta;

                t = t + dt;
                g_hat = theta(1:end-1);
                x_hat = theta(end);
                
                if (t >= tau), break; end
                
            end
            
        end
        
        function plotSimResults(this)
            
            fig = figure;
            fig.Position(3:4) = [658 844];
            % -------------------------------------------
            ax = subplot(3,1,1); hold(ax, 'on');
            plot(this.Time, this.g_hat_data, 'LineWidth',2, 'Color', 'blue');
            axis tight;
            plot(ax.XLim, [this.g_new this.g_new], 'LineWidth',2, 'Color', 'magenta', 'LineStyle','--');
            legend({'$\hat{g}$', '$g$'}, 'interpreter','latex', 'fontsize',15);
            % -------------------------------------------
            ax = subplot(3,1,2); hold(ax, 'on');
            plot(this.Time, this.x_hat_data, 'LineWidth',2, 'Color', [0.85 0.33 0.1]);
            plot(this.Time, this.x_data, 'LineWidth',2, 'Color', 'cyan', 'LineStyle','--');
            axis tight;
            legend({'$\hat{x}$', '$x$'}, 'interpreter','latex', 'fontsize',15);
            xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
            % -------------------------------------------
            ax = subplot(3,1,3); hold(ax, 'on');
            plot(this.Time, this.z_hat_data, 'LineWidth',2, 'Color', 'blue');
            plot(this.Time, this.z_data, 'LineWidth',2, 'Color', 'magenta', 'LineStyle','--');
            axis tight;
            legend({'$\hat{z}$', '$z$'}, 'interpreter','latex', 'fontsize',15);
            xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
            
        end

        function theta_next = stateTransFun(this, theta, cookie)
            
            dt = cookie.dt;
            t = cookie.t;
            
            theta_next = theta;
            theta_next(end) = theta(end)*(1 + dt/t);
            
        end
        
        function z = msrFun(this, theta, cookie)
            
            g = theta(1:end-1);
            x = theta(end);
            
            g0 = this.gmp.getGoal(); % store current goal
            
            this.gmp.setGoal(g);
            y = this.gmp.getYd(x);

            z = y;
            
            this.gmp.setGoal(g0); % restore previous goal
            
        end

        function J = stateTransFunJacob(this, theta, cookie)
           
            dt = cookie.dt;
            t = cookie.t;
            
            J = eye(this.N_params, this.N_params);
            J(end, end) = (1 + dt/t);
            
        end
        
        function J = msrFunJacob(this, theta, cookie)
            
            g = theta(1:end-1);
            x = theta(end);
      
            J = zeros(this.N_out, this.N_params);
            J(:,1) = this.gmp.dYd_dgoal(x, g);
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
        x_hat_data
        z_hat_data
        z_data
        x_data
    
    end
    
    
    
end