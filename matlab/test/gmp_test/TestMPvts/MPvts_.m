classdef MPvts_ < handle
       
    methods (Access = public)
        
        %% Constructor
        function this = MPvts_()
            
        end

    end
    
    methods (Abstract, Access = public)
        
        %% Trains the model using the loaded training data
        offline_train_mse = train(this, train_params, Timed, Pd_data, dPd_data, ddPd_data)

        % update function for model
        [y_dot, z_dot] = update(this, x, x_dot, x_ddot, y, z, yg, tau, tau_dot, y_c, z_c)
        
        % returns the model's vel and accel (assuming a model update is called first).
        [y_dot, y_ddot] = getVelAccel(this)
        
        
        % returns the damping of the impedance-like equivalent form
        [D_, K_] = getImpParams(this, tau, tau_dot)
        
        
        % set the model's initial position
        setY0(this, y0)
        
    end
    

end


