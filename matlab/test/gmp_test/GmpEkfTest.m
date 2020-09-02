

classdef GmpEkfTest < handle
    
    methods (Static, Access = public)
        
        
        function run()
           
            clc;
            close all;
            clear;
            
            obj = GmpEkfTest();
            
            obj.train('data/train_data.mat');
            obj.plotTrainResults();
            
            obj.initObserver();
            
        end
        
    end
    
    methods (Access = public)
        
        function this = GmpEkfTest()

        end
        
        
        function train(this, data_path)
            
            load(data_path, 'Data');
            this.train_data = struct('Time',Data.Time, 'Pos',Data.Pos(1,:), 'Vel',Data.Vel(1,:), 'Accel',Data.Accel(1,:));

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
        
        function initObserver(this, theta0)
            
            N_params = length(theta0);
            N_out = 1;
            
            Qn = diag([0.02; 0.05]);
            Rn = 1000 * eye(N_out, N_out);
            a_p = 1.002;
            P_theta = 1e5*eye(N_params, N_params);
            
            num_diff_step = [0.001; 0.01];
            
            this.ekf = EKF(N_params, N_out, @pStateTransFun, @pMsrFun);
            this.ekf.setProcessNoiseCov(Qn);
            this.ekf.setMeasureNoiseCov(Rn);
            this.ekf.setFadingMemoryCoeff(a_p);
            this.ekf.theta = theta0;
            this.ekf.P = P_theta;

            %this.ekf.enableParamsContraints(enable_constraints);
            %this.ekf.setParamsConstraints(A_c, b_c);
            this.ekf.setPartDerivStep(num_diff_step);

            %this.ekf.setStateTransFunJacob(@pStateTransFunJacob);
            %this.ekf.setMsrFunJacob(@pMsrFunJacob);

        end
        
        
        
    end
        
        
    properties (Access = private)
    
        train_data % training data
        
        gmp % GMP model
        
        ekf % observer
    
    end
    
    
    
end