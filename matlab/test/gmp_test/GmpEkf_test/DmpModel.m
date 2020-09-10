%% GMP - Model class

classdef DmpModel < Model

    %% ==========================================================
    
    methods (Access = public)
        
        function this = DmpModel()
        
        end
        
        function init(this, params)
           
            a_z = params.damping;
            b_z = params.stiffness / a_z;
            this.can_clock_ptr = CanonicalClock();
            
            this.dmp = DMP(params.N_kernels, a_z, b_z, this.can_clock_ptr, SigmoidGatingFunction(1.0, 0.5));

        end
        
        function offline_train_mse = train(this, train_method, train_data)
            
            this.y0 = train_data.Pos(:,1);
            this.g = train_data.Pos(:,end);
            
            offline_train_mse = this.dmp.train(train_method, train_data.Time, train_data.Pos, train_data.Vel, train_data.Accel);
            
        end
        
        function [Pos, Vel, Accel] = simulate(this, Time)
           
            tau = Time(end);
            this.can_clock_ptr.setTau(tau);
            this.dmp.setY0(this.y0);
            
            n_data = length(Time);
            Pos = zeros(1,n_data);
            Vel = zeros(1,n_data);
            Accel = zeros(1,n_data);
            
            y = this.y0;
            dy = 0;
            ddy = 0;
            x = 0;
            
            Time = [Time Time(end)]; % to avoid going out of bounds in dt calculation
            
            for j=1:n_data
     
                ddy = this.dmp.calcYddot(x, y, dy, this.g);
                dx = this.can_clock_ptr.getPhaseDot(x);
                
                Pos(j) = y;
                Vel(j) = dy;
                Accel(j) = ddy;
                
                dt = Time(j+1) - Time(j);
                dy = dy + ddy*dt;
                y = y + dy*dt;
                x = x + dx*dt;
     
            end
            
        end
        
        function setStart(this, y0)
            
            this.dmp.setY0(y0);

        end
        
        function setGoal(this, g)
        
            this.g = g;
            
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
                yg = theta(1:end-2);
                tau = theta(end-1);
                x = theta(end);
            else
                yg = theta(1:end-1);
                tau = theta(end);
                x = cookie.t / tau;
            end

            tau0 = this.dmp.getTau();
            this.dmp.setTau(tau);
            z = this.dmp.calcYddot(x, cookie.y, cookie.y_dot, yg);
            this.dmp.setTau(tau0);

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
            
            error('Not implemented!');

        end
            
        % -------------------------------------------------------------
        
        function [y, y_dot] = nextState(this, x, y, y_dot, tau, dt)
           
            tau0 = this.dmp.getTau();
            this.dmp.setTau(tau);
            y_ddot = this.dmp.calcYddot(x, y, y_dot, this.g);
            this.dmp.setTau(tau0);
            
            y_dot = y_dot + y_ddot*dt;
            y = y + y_dot*dt;
            
        end
    end
    
    %% ==========================================================
    
    properties (Access = public)
        
        y0 % initial pos
        g % goal
        dmp
        can_clock_ptr
        shape_attr_gating_ptr

    end
    
end
