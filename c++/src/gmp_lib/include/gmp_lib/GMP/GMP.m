%% GMP class
%  Generalized movement primitive.
%

classdef GMP < matlab.mixin.Copyable
    
    methods (Access = public)
        
        %% DMP constructor.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] D: damping.
        %  @param[in] K: stiffness.
        %  @param[in] kernels_std_scaling: Scaling for std of kernels (optional, default=2).
        function this = GMP(N_kernels, D, K, kernels_std_scaling)
                
            if (nargin < 4), kernels_std_scaling = 2.0; end
            
            this.D = D;
            this.K = K;
            this.shape_attr_gating_ptr = SigmoidGatingFunction(1.0, 0.99);
            this.shape_attr_gating_ptr.setSteepness(750);
            
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

            x = Time / Time(end);
            train_error = this.wsog.train(train_method, x, yd_data);
            
            this.wsog_opt = this.wsog.deepCopy();
            
        end

        
        %% Constrained optimization.
        %  @param[in] T: Time duration of the motion.
        %  @param[in] pos_constr: Vector of @GMPConstr position constraints. For no constraints pass '[]'.
        %  @param[in] vel_constr: Vector of @GMPConstr velocity constraints. For no constraints pass '[]'.
        %  @param[in] accel_constr: Vector of @GMPConstr acceleration constraints. For no constraints pass '[]'.
        %  @param[in] opt_set: Object of type @GMPOptSet for setting optimization options.
        function constrOpt(this, T, pos_constr, vel_constr, accel_constr, opt_set)
        
            ref_data = struct('pos',[], 'vel',[], 'accel',[], 'w_p',[], 'w_v',[], 'w_a',[]);
            
            N = 4000;
            
            x = (0:(N-1))/(N-1);
            x_dot = 1/T;
            
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
                    ref_data.vel(i) = this.getYdDot(x(i), x_dot);
                end
            end
            
            if (opt_set.opt_accel)
                ref_data.accel = zeros(1, N);
                for i=1:N
                    ref_data.accel(i) = this.getYdDDot(x(i), x_dot, 0);
                end
            end
            
            this.wsog_opt.constrOpt(x, ref_data, T, pos_constr, vel_constr, accel_constr, opt_set);
            
        end
            
        
        %% Updates the weights so that the generated trajectory passes from the given points.
        %  @param[in] s: Matrix of phase variable states, i.e. s = [x;x_dot; x_ddot].
        %                If s = [x; x_dot], x_ddot is assumed to be 0.
        %  @param[in] z: Vector with the desired value for each timestamp.
        %  @param[in] type: Vector with the type of each point (GMP_UPDATE_TYPE).
        %  @param[in] z_var: Vector with the variance of each point (optional, default = 1e-3).
        function updateWeights(this, s, z, type, z_var)
            
            if (nargin < 5), z_var = 1e-3; end
            
            n = size(s, 2);
            m = size(s,1);
            
            x = s(1,:);
            x_dot = s(2, :);
            x_ddot = zeros(1,n);
            if (m > 2), x_ddot = s(3, :); end
            
            if (isscalar(z_var)), z_var = z_var*ones(n,1); end
            
            k = this.wsog_.numOfKernels();
            
            H = zeros(n, k);
            z_hat = zeros(n,1);
            
            for i=1:n
                if (type(i) == GMP_UPDATE_TYPE.POS)
                    Hi = this.wsog_.regressVec(x(i))';
                    z_hat(i) = this.wsog_.output(x(i));
                elseif (type(i) == GMP_UPDATE_TYPE.VEL)
                    Hi = this.wsog_.regressVecDot(x(i), x_dot(i))';
                    z_hat(i) = this.wsog_.outputDot(x(i), x_dot(i));
                elseif (type(i) == GMP_UPDATE_TYPE.ACCEL)
                    Hi = this.wsog_.regressVecDDot(x(i), x_dot(i), x_ddot(i))';
                    z_hat(i) = this.wsog_.outputDDot(x(i), x_dot(i), x_ddot(i));
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
        %  @param[in] y_c: coupling term for the dynamical equation of the \a y state.
        %  @param[in] z_c: coupling term for the dynamical equation of the \a z state.
        function update(this, s, y, z, y_c, z_c)

            if (nargin < 5), y_c=0; end
            if (nargin < 6), z_c=0; end
            
            this.z_dot = ( this.goalAttractor(y, z) + this.shapeAttractor(s) + z_c);
            this.y_dot = z + y_c;

        end

        
        function y_dot = getYdot(this), y_dot = this.y_dot; end
        function z_dot = getZdot(this), z_dot = this.z_dot; end
        
        
        %% Returns the DMP's acceleration.
        function y_ddot = getYddot(this, yc_dot)
            
            if (nargin < 2), yc_dot = 0; end
            y_ddot = this.getZdot() + yc_dot;

        end
        
        
        %% Calculates the DMP's acceleration.
        function y_ddot = calcYddot(this, s, y, y_dot, yc, zc, yc_dot)

            if (nargin < 5), yc = 0; end
            if (nargin < 6), zc = 0; end
            if (nargin < 7), yc_dot = 0; end

            z = y_dot - yc;
            z_dot = ( this.goalAttractor(y, z) + this.shapeAttractor(s) + zc);

            y_ddot = (z_dot + yc_dot);

        end
        
        
        %% Returns the number of kernels.
        function n_ker = numOfKernels(this), n_ker = length(this.wsog_.w); end

        
        %% Sets the initial position.
        function setY0(this, y0)
            
            this.wsog_.setStartValue(y0); 
            
        end
        
        
        %% Set goal position.
        function setGoal(this, g)
            
            this.g = g;
            this.wsog_.setFinalValue(g); 
            
        end

        
        %% Creates a deep copy of this object
        function cp_obj = deepCopy(this)
            
            % Make a shallow copy of all properties
            cp_obj = this.copy();
            % Make a deep copy of the pointers
            cp_obj.shape_attr_gating_ptr = this.shape_attr_gating_ptr.copy();

        end
          
        
        function p_ref = getYdOpt(this, x)
            
            p_ref = this.wsog_opt.output(x);
            
        end
        
        
        function p_ref_dot = getYdDotOpt(this, x, x_dot)
            
            p_ref_dot = this.wsog_opt.outputDot(x, x_dot);
            
        end
        
        
        function p_ref_ddot = getYdDDotOpt(this, x, x_dot, x_ddot)
            
            if (nargin < 4), x_ddot = 0; end
            
            p_ref_ddot = this.wsog_opt.outputDDot(x, x_dot, x_ddot);
            
        end

        
        function p_ref = getYd(this, x)
            
            p_ref = this.wsog_.output(x);
            
        end
        
        
        function p_ref_dot = getYdDot(this, x, x_dot)
            
            p_ref_dot = this.wsog_.outputDot(x, x_dot);
            
        end
        
        
        function p_ref_ddot = getYdDDot(this, x, x_dot, x_ddot)
            
            if (nargin < 4), x_ddot = 0; end
            
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
                
        
        %% Returns the goal attractor of the this.
        %  @param[in] y: \a y state of the this.
        %  @param[in] z: \a z state of the this.
        %  @param[in] g: Goal position.
        %  @param[out] goal_attr: The goal attractor of the this.
        function goal_attr = goalAttractor(this, y, z)

            goal_attr = this.K*(this.g-y)- this.D*z;

        end
        
        
        %% Returns the shape attractor
        function shape_attr = shapeAttractor(this, s)
            
            x = s(1);
            sAttrGat = this.shape_attr_gating_ptr.getOutput(x);
            if (sAttrGat<0), sAttrGat = 0.0; end
            shape_attr = sAttrGat * this.forcingTerm(s);
            
        end
        
        
        %% Returns the forcing term
        function f = forcingTerm(this, s)
            
            x = s(1);
            x_dot = s(2);
            x_ddot = 0;
            if (length(s) > 2), x_ddot = s(3); end
            
            yd = this.wsog_.output(x);
            yd_dot = this.wsog_.outputDot(x, x_dot);
            yd_ddot = this.wsog_.outputDDot(x, x_dot, x_ddot);
            
            f = yd_ddot + this.D*yd_dot + this.K*(yd - this.g);
            
        end
        

    end
    
    properties (Access = public)
     
        D % damping
        K % stiffness

        shape_attr_gating_ptr % pointer to gating function for the shape attractor
        
        wsog_ % abstract WSoG object
        
        wsog % WSoG object
        wsog_opt % optimized WSoG object

    end
    
    
    properties (Access = protected)
        
        %% output state
        y_dot % position derivative
        z_dot % scaled velocity derivative
        
        g % target/goal

    end
    
end
