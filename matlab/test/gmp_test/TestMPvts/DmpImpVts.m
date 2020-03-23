classdef DmpImpVts < MPvts_
       
    methods (Access = public)
        
        %% Constructor
        function this = DmpImpVts()
            
        end

        
        %% Trains a DMP-imp model using the loaded training data
        function offline_train_mse = train(this, train_params, Timed, Pd_data, dPd_data, ddPd_data)
            
            if (isempty(Timed)), error('The training data are empty...'); end

            this.D = train_params.D;
            this.K = train_params.K;
            this.N_kernels = train_params.N_kernels;
            this.taud = Timed(end);
            
            kernels_std_scaling = 2;
            n_dof = size(Pd_data,1);
            this.gmp = GMP_nDoF(n_dof, this.N_kernels, this.D, this.K, kernels_std_scaling);
            offline_train_mse = this.gmp.train(train_params.train_method, Timed, Pd_data);
       
        end
        

        % update function for DMP-imp model
        function [y_dot, z_dot] = update(this, x, x_dot, x_ddot, y, z, yg, tau, tau_dot, y_c, z_c)
  
            if (x > 1)
                x = 1;
                x_dot = 0;
                x_ddot = 0;
            end
            
            this.gmp.setGoal(yg);
            
            kt = this.taud/tau;

            yd = this.gmp.getYd(x);
            yd_dt1 = this.gmp.getYdDot(x, x_dot) / kt;
            yd_ddt1 = this.gmp.getYdDDot(x, x_dot, 0) / kt^2;
            
            y_ref = yd;
            y_ref_dot = kt*yd_dt1;
            y_ref_ddot = kt^2*yd_ddt1 + this.taud*x_ddot*yd_dt1;
            
            K_ = this.K/tau^2;
            D_ = (this.D + tau_dot)/tau;
            y_dot = z;
            z_dot = y_ref_ddot - D_*(z - y_ref_dot) - K_*(y - y_ref);
            
            this.y_dot = y_dot;
            this.y_ddot = z_dot;

        end
        
        
        % returns the dmp vel and accel (assuming a dmp update is called first).
        function [y_dot, y_ddot] = getVelAccel(this)
            
            y_dot = this.y_dot;
            y_ddot = this.y_ddot;
            
        end
        
        
        % returns the damping of the impedance-like equivalent form
        function [D_, K_] = getImpParams(this, tau, tau_dot)
            
            D_ = (this.D + tau_dot) / tau;
            K_ = this.K / tau^2;
            
        end
        
        
        % set the model's initial position
        function setY0(this, y0)
           
            this.gmp.setY0(y0);
            
        end
        
    end
    
    
    properties (Access = public)

        % ==== Training params ====
        K % model stiffness
        D % model damping
        N_kernels % number of basis functions
        
        taud

        y_dot
        y_ddot
        
        % ==== model ====
        gmp
        
    end

end


