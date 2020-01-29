%% GMP class
%  Generalized movement primitive.
%

classdef GMP < matlab.mixin.Copyable
    
    methods (Access = public)
        
        %% DMP constructor.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] kernels_std_scaling: Scaling for std of kernels (optional, default=2).
        function this = GMP(N_kernels, a_z, b_z, can_clock_ptr, kernels_std_scaling)
                
            if (nargin < 5), kernels_std_scaling = 2.0; end
            
            this.a_z = a_z;
            this.b_z = b_z;
            this.can_clock_ptr = can_clock_ptr;
            this.shape_attr_gating_ptr = SigmoidGatingFunction(1.0, 0.5);
%             this.shape_attr_gating_ptr = LinGatingFunction(1.0, 1.0);
            
            this.wsog = WSoG(N_kernels, kernels_std_scaling);
            this.wsog_opt = this.wsog.deepCopy();
            
            this.setOptTraj(false);
            
            this.setY0(0);
            this.setGoal(1);
            
        end

        
        %% Trains the DMP.
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] yd_data: Row vector with the desired potition.
        function train_error = train(this, train_method, Time, yd_data)
        
            tau = Time(end);
            this.setTau(tau);
            
            x = Time / Time(end);
            train_error = this.wsog.train(train_method, x, yd_data);
            
            this.wsog_opt = this.wsog.deepCopy();
            
        end

        
        %% Constrained optimization.
        %  @param[in] pos_constr: Vector of @GMPConstr position constraints. For no constraints pass '[]'.
        %  @param[in] vel_constr: Vector of @GMPConstr velocity constraints. For no constraints pass '[]'.
        %  @param[in] accel_constr: Vector of @GMPConstr acceleration constraints. For no constraints pass '[]'.
        %  @param[in] opt_set: Object of type @GMPOptSet for setting optimization options.
        function constrOpt(this, pos_constr, vel_constr, accel_constr, opt_set)
        
            ref_data = struct('pos',[], 'vel',[], 'accel',[], 'w_p',[], 'w_v',[], 'w_a',[]);
            
            N = 4000;
            
            x = (0:(N-1))/(N-1);
            this.setOptTraj(false);
            
            if (opt_set.opt_pos)
                ref_data.pos = zeros(1, N);
                for i=1:N
                    ref_data.pos(i) = this.getYd(x(i));
                end
            end
            
            if (opt_set.opt_vel)
                ref_data.vel = zeros(1, N);
                for i=1:N
                    ref_data.vel(i) = this.getYdDot(x(i));
                end
            end
            
            if (opt_set.opt_accel)
                ref_data.accel = zeros(1, N);
                for i=1:N
                    ref_data.accel(i) = this.getYdDDot(x(i));
                end
            end
            
            tau = this.getTau();
            
            this.wsog_opt.constrOpt(x, ref_data, tau, pos_constr, vel_constr, accel_constr, opt_set);
            
        end
            
        %% Updates the weights so that the generated trajectory passes from the given points.
        %  @param[in] x: Vector of timestamps.
        %  @param[in] z: Vector with the desired value for each timestamp.
        %  @param[in] z: Vector with the type of each point (GMP_UPDATE_TYPE).
        %  @param[in] z_var: Vector with the variance of each point (optional, default = 1e-3).
        function updateWeights(this, x, z, type, z_var)
            
            if (nargin < 5), z_var = 1e-3; end
            
            n = length(x);
            
            if (isscalar(z_var)), z_var = z_var*ones(n,1); end
            
            k = this.wsog_.numOfKernels();
            
            H = zeros(n, k);
            z_hat = zeros(n,1);
            
            for i=1:n
                if (type(i) == GMP_UPDATE_TYPE.POS)
                    Hi = this.wsog_.regressVec(x(i))';
                    z_hat(i) = this.wsog_.output(x(i));
                elseif (type(i) == GMP_UPDATE_TYPE.VEL)
                    x_dot = this.phaseDot(x(i));
                    Hi = this.wsog_.regressVecDot(x(i), x_dot)';
                    z_hat(i) = this.wsog_.outputDot(x(i), x_dot);
                elseif (type(i) == GMP_UPDATE_TYPE.ACCEL)
                    x_dot = this.phaseDot(x(i));
                    x_ddot = 0;
                    Hi = this.wsog_.regressVecDDot(x(i), x_dot, x_ddot)';
                    z_hat(i) = this.wsog_.outputDDot(x(i), x_dot, x_ddot);
                end
                H(i,:) = Hi;
            end
            
            e = z - z_hat;
            this.wsog_.updateWeights(H, e, diag(z_var));
            
        end
        
        %% Returns the derivatives of the DMP states.
        %  @param[in] x: phase variable.
        %  @param[in] y: \a y state of the this.
        %  @param[in] z: \a z state of the this.
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[in] y_c: coupling term for the dynamical equation of the \a y state.
        %  @param[in] z_c: coupling term for the dynamical equation of the \a z state.
        %  @param[out] dy: derivative of the \a y state of the this.
        %  @param[out] dz: derivative of the \a z state of the this.
        %  @param[out] dx: derivative of the phase variable of the this.
        function update(this, x, y, z, g, y_c, z_c)

            if (nargin < 6), y_c=0; end
            if (nargin < 7), z_c=0; end
            
            this.setGoal(g);

            tau = this.getTau();
            shape_attr = this.shapeAttractor(x);
            goal_attr = this.goalAttractor(x, y, z, g);
            
            this.dz = ( goal_attr + shape_attr + z_c) / tau;
            this.dy = ( z + y_c) / tau;
            this.dx = this.phaseDot(x);

        end

        function dx = getXdot(this), dx=this.dx; end
        function dy = getYdot(this), dy=this.dy; end
        function dz = getZdot(this), dz=this.dz; end
        
        
        %% Returns the DMP's acceleration.
        function ddy = getYddot(this, tau_dot, yc_dot)
            
            if (nargin < 2), tau_dot = 0; end
            if (nargin < 3), yc_dot = 0; end
            ddy = (this.getZdot() + yc_dot - tau_dot*this.getYdot()) / this.getTau();

        end
        
        
        %% Calculates the DMP's acceleration.
        function ddy = calcYddot(this, x, y, dy, g, tau_dot, yc, zc, yc_dot)

            if (nargin < 6), tau_dot = 0; end
            if (nargin < 7), yc = 0; end
            if (nargin < 8), zc = 0; end
            if (nargin < 9), yc_dot = 0; end

            tau = this.getTau();
            z = dy*tau - yc;

            shape_attr = this.shapeAttractor(x);
            goal_attr = this.goalAttractor(x, y, z, g);
            dz = ( goal_attr + shape_attr + zc) / tau;

            ddy = (dz + yc_dot - tau_dot*dy)/tau;

        end
        
        
        %% Returns the number of kernels.
        function n_ker = numOfKernels(this), n_ker = length(this.wsog_.w); end

        
        %% Sets the initial position.
        function setY0(this, y0)
            
            this.wsog_.setStartValue(y0); 
            
        end
        
        
        %% Set goal position.
        function setGoal(this, g)
            
            this.wsog_.setFinalValue(g); 
            
        end

        
        %% Returns the time scale of the DMP.
        %  @param[out] tau: The time scale of the this.
        function tau = getTau(this), tau = this.can_clock_ptr.getTau(); end
        
        
        %% Sets the time scale of the DMP.
        function setTau(this, tau), this.can_clock_ptr.setTau(tau); end
   
        
        %% Returns the phase variable.
        %  @param[in] t: The time instant.
        %  @param[out] x: The phase variable for time 't'.
        function x = phase(this, t), x = this.can_clock_ptr.getPhase(t); end
   
        
        %% Returns the derivative of the phase variable.
        %  @param[in] x: The phase variable.
        %  @param[out] dx: The derivative of the phase variable.
        function dx = phaseDot(this, x), dx = this.can_clock_ptr.getPhaseDot(x); end

        
        %% Returns the goal attractor of the this.
        %  @param[in] x: The phase variable.
        %  @param[in] y: \a y state of the this.
        %  @param[in] z: \a z state of the this.
        %  @param[in] g: Goal position.
        %  @param[out] goal_attr: The goal attractor of the this.
        function goal_attr = goalAttractor(this, x, y, z, g)

            g_attr_gating = this.goalAttrGating(x);
            goal_attr = g_attr_gating * this.a_z*(this.b_z*(g-y)-z);

        end
        
        
        %% Returns the shape attractor
        function shape_attr = shapeAttractor(this, x)
            
            shape_attr = this.shapeAttrGating(x) * this.forcingTermScaling() * this.forcingTerm(x);
            
        end
        
        %% Creates a deep copy of this object
        function cp_obj = deepCopy(this)
            
            % Make a shallow copy of all properties
            cp_obj = this.copy();
            % Make a deep copy of the pointers
            cp_obj.can_clock_ptr = this.can_clock_ptr.copy();
            cp_obj.shape_attr_gating_ptr = this.shape_attr_gating_ptr.copy();

        end
          
        function p_ref = getYdOpt(this, x)
            
            p_ref = this.wsog_opt.output(x);
            
        end
        
        function p_ref_dot = getYdDotOpt(this, x)
            
            x_dot = this.phaseDot(x);
            p_ref_dot = this.wsog_opt.outputDot(x, x_dot);
            
        end
        
        function p_ref_ddot = getYdDDotOpt(this, x)
            
            x_dot = this.phaseDot(x);
            x_ddot = 0;
            p_ref_ddot = this.wsog_opt.outputDDot(x, x_dot, x_ddot);
            
        end

        function p_ref = getYd(this, x)
            
            p_ref = this.wsog_.output(x);
            
        end
        
        function p_ref_dot = getYdDot(this, x)
            
            x_dot = this.phaseDot(x);
            p_ref_dot = this.wsog_.outputDot(x, x_dot);
            
        end
        
        function p_ref_ddot = getYdDDot(this, x)
            
            x_dot = this.phaseDot(x);
            x_ddot = 0;
            p_ref_ddot = this.wsog_.outputDDot(x, x_dot, x_ddot);
            
        end

        
        %% Enables/Disables the use of the optimized mp-model
        function setOptTraj(this, set)
           
            if (set), this.wsog_ = this.wsog_opt;
            else, this.wsog_ = this.wsog;
            end
            
        end
        
        
    end
    
    methods (Access = protected)
        
        %% Returns the shape attractor gating factor.
        %  @param[in] x: The phase variable.
        function sAttrGat = shapeAttrGating(this, x)

            sAttrGat = this.shape_attr_gating_ptr.getOutput(x);
            sAttrGat(sAttrGat<0) = 0.0;

        end
        
        
        %% Returns the goal attractor gating factor.
        %  @param[in] x: The phase variable.
        function gAttrGat = goalAttrGating(this, x), gAttrGat = 1.0; end

        
        %% Returns the forcing term
        function f = forcingTerm(this, x)
            
            x_dot = this.phaseDot(x);
            x_ddot = 0;
            
            tau = this.getTau();
            
            % get unscaled trajectory
            y0_d = this.wsog_.getStartDemoValue();
            y0 = this.wsog_.getStartValue();
            spat_s = this.wsog_.getSpatialScaling();
            
            yd = (this.wsog_.output(x) + spat_s*y0_d - y0) / spat_s;
            yd_dot = this.wsog_.outputDot(x, x_dot) / spat_s;
            yd_ddot = this.wsog_.outputDDot(x, x_dot, x_ddot) / spat_s;
            
            % yd_dot is already the scaled velocity, i.e. yd_dot = dyd * taud/tau
            % that's why we mutliply do yd_dot*tau = dyd * taud which is
            % what we want below. Accordingly for the yd_ddot
            gd = this.wsog_.getFinalDemoValue();
            f = tau^2*yd_ddot + this.a_z*tau*yd_dot - this.a_z*this.b_z*(gd - yd);
            
        end

        
        %% Returns the forcing term scaling
        function f_scale = forcingTermScaling(this)
           
            f_scale = this.wsog_.getSpatialScaling();
            
        end
        

    end
    
    properties (Access = public)
     
        a_z % parameter 'a_z' relating to the spring-damper system
        b_z % parameter 'b_z' relating to the spring-damper system

        can_clock_ptr % pointer to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor
        
        wsog_ % abstract WSoG object
        
        wsog % WSoG object
        wsog_opt % optimized WSoG object

    end
    
    
    properties (Access = protected)
        
        %% output state
        dy % position derivative
        dz % scaled velocity derivative
        dx % phase variable derivative

    end
    
end
