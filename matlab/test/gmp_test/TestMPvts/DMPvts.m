classdef DMPvts < MPvts_
       
    methods (Access = public)
        
        %% Constructor
        function this = DMPvts()
            
        end

        
        %% Trains a DMP model using the loaded training data
        function offline_train_mse = train(this, train_params, Timed, Pd_data, dPd_data, ddPd_data)
            
            if (isempty(Timed)), error('The training data are empty...'); end
            
            this.D = train_params.D;
            this.K = train_params.K;
            this.N_kernels = train_params.N_kernels;
            
            a_z = this.D;
            b_z = this.K/this.D;
            can_clock_ptr = CanonicalClock();
            shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
            this.dmp = DMP_pos(DMP_TYPE.STD, this.N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);
            offline_train_mse = this.dmp.train(train_params.train_method, Timed, Pd_data, dPd_data, ddPd_data);
            
        end
        

        % update function for DMP model
        function [y_dot, z_dot] = update(this, x, x_dot, x_ddot, y, z, yg, tau, tau_dot, y_c, z_c)
  
            this.dmp.setTau(tau);
            this.dmp.update(x, y, z, yg, y_c, tau^2*z_c);
            y_dot = this.dmp.getYdot();
            z_dot = this.dmp.getZdot();

        end
        
        
        % returns the dmp vel and accel (assuming a dmp update is called first).
        function [y_dot, y_ddot] = getVelAccel(this)
            
            y_dot = this.dmp.getYdot();
            y_ddot = this.dmp.getYddot();
            
        end
        
        
        % returns the damping of the impedance-like equivalent form
        function [D_, K_] = getImpParams(this, tau, tau_dot)
            
            D_ = (this.D + tau_dot) / tau;
            K_ = this.K / tau^2;
            
        end
        
        
        % set the model's initial position
        function setY0(this, y0)
           
            this.dmp.setY0(y0);
            
        end
        
    end
    
    
    properties (Access = public)

        % ==== Training params ====
        K % model stiffness
        D % model damping
        N_kernels % number of basis functions
        
        % ==== GMP model ====
        dmp

    end

end


