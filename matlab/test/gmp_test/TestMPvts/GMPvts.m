classdef GMPvts < MPvts_
       
    methods (Access = public)
        
        %% Constructor
        function this = GMPvts()
            
        end

        
        %% Trains a GMP model using the loaded training data
        function offline_train_mse = train(this, train_params, Timed, Pd_data, dPd_data, ddPd_data)
            
            if (isempty(Timed)), error('The training data are empty...'); end

            this.D = train_params.D;
            this.K = train_params.K;
            this.N_kernels = train_params.N_kernels;
            
            kernels_std_scaling = 2;
            n_dof = size(Pd_data,1);
            this.gmp = GMP_nDoF(n_dof, this.N_kernels, this.D, this.K, kernels_std_scaling);
            offline_train_mse = this.gmp.train(train_params.train_method, Timed, Pd_data);
            
        end

        % update function for GMP model
        function [y_dot, z_dot] = update(this, x, x_dot, x_ddot, y, z, yg, tau, tau_dot, y_c, z_c)

            if (x > 1)
                x = 1;
                x_dot = 0;
                x_ddot = 0;
            end
            
            xs = [x; x_dot; x_ddot];
            this.gmp.setGoal(yg);
            this.gmp.update(xs, y, z, y_c, z_c);
            y_dot = this.gmp.getYdot();
            z_dot = this.gmp.getZdot();

        end
        
        % returns the gmp vel and accel (assuming a gmp update is called first).
        function [y_dot, y_ddot] = getVelAccel(this)
            
            y_dot = this.gmp.getYdot();
            y_ddot = this.gmp.getYddot();
            
        end
        
        
        % returns the damping of the impedance-like equivalent form
        function [D_, K_] = getImpParams(this, tau, tau_dot)
            
            D_ = this.D;
            K_ = this.K;
            
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
        
        % ==== GMP model ====
        gmp

    end

end


