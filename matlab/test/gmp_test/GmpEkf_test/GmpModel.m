%% GMP - Model class

classdef GmpModel < Model

    %% ==========================================================
    
    methods (Access = public)
        
        function this = GmpModel()
        
        end
        
        function init(this, params)
           
            this.gmp = GMP(params.N_kernels, params.damping, params.stiffness, params.kernels_std_scaling);

        end
        
        function offline_train_mse = train(this, train_method, train_data)
            
            offline_train_mse = this.gmp.train(train_method, train_data.Time, train_data.Pos);
            
        end
        
        function [Pos, Vel, Accel] = simulate(this, Time)
           
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
            
        end
        
        function setStart(this, y0)
            
            this.gmp.setY0(y0);

        end
        
        function setGoal(this, g)
        
            this.gmp.setGoal(g);
            
        end
        
        % -------------------------------------------------------------
        
        function theta_next = stateTransFun(this, theta, cookie)
                
            theta_next = theta;
            
            if (cookie.est_x)
                tau = theta(end-1);
                x = theta(end);
                theta_next(end) = x + cookie.dt/tau;
            end
            
        end
        
        function z = msrFun(this, theta, cookie)
            
            if (nargin < 3), cookie = []; end
            
            if (cookie.est_x)
                g = theta(1:end-2);
                tau = theta(end-1);
                x = theta(end);
            else
                g = theta(1:end-1);
                tau = theta(end);
                x = cookie.t / tau;
            end
            
            g0 = this.gmp.getGoal(); % store current goal
            this.gmp.setGoal(g);
            y = this.gmp.getYd(x);
            this.gmp.setGoal(g0); % restore previous goal
            
            z = y;

        end

        function J = stateTransFunJacob(this, theta, cookie)
           
            N = length(theta);
            J = eye(N,N);
            
            if (cookie.est_x)
                tau = theta(end-1);
                J(end,end-1) = -cookie.dt/tau^2;
            end
            
        end
        
        function J = msrFunJacob(this, theta, cookie)
            
            if (nargin < 3), cookie = []; end

            if (cookie.est_x)
                g = theta(1:end-2);
                tau = theta(end-1);
                x = theta(end);
            else
                g = theta(1:end-1);
                tau = theta(end);
                x = cookie.t / tau;
            end
            
            dx_dt = 1/tau;
            
            N_params = length(theta);
            N_out = length(this.msrFun(theta,cookie));
      
            J = zeros(N_out, N_params);
            J(:,1) = this.gmp.dYd_dgoal(x, g);
            
            if (cookie.est_x)
                J(:,end-1) = this.gmp.dYd_dx(x, g) * dx_dt;
                J(:,end) = this.gmp.dYd_dx(x, g);
            else
                J(:,end) = this.gmp.dYd_dx(x, g) * dx_dt;
            end

        end
        
        % -------------------------------------------------------------
        
        function [y, y_dot] = nextState(this, x, y, y_dot, tau, dt)
           
            x_dot = 1/tau;
            x = x + x_dot*dt;
            y = this.gmp.getYd(x);
            y_dot = this.gmp.getYdDot(x, x_dot);
            
        end
        
    end
    
    %% ==========================================================
    
    properties (Access = public)
        
        gmp

    end
    
end
