%% N-DoF GMP class
%  Generalized movement primitive.
%

classdef GMP_nDoF < matlab.mixin.Copyable
    
    methods (Access = public)
        
        %% DMP constructor.
        %  @param[in] n: number of degrees of freedom.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] D: damping.
        %  @param[in] K: stiffness.
        %  @param[in] kernels_std_scaling: Scaling for std of kernels (optional, default=2).
        function this = GMP_nDoF(n, N_kernels, D, K, kernels_std_scaling)
                
            if (nargin < 5), kernels_std_scaling = 2.0; end
            
            if (length(N_kernels) == 1), N_kernels = ones(n,1)*N_kernels(1); end
            if (length(D) == 1), D = ones(n,1)*D(1); end
            if (length(K) == 1), K = ones(n,1)*K(1); end
            
            this.gmp = cell(n,1);
            for i=1:n, this.gmp{i} = GMP(N_kernels(i), D(i), K(i), kernels_std_scaling); end
            
            this.setY0(zeros(n,1));
            this.setGoal(ones(n,1));
            
            this.y_dot = zeros(n,1);
            this.z_dot = zeros(n,1);
            
        end
        
        
        %% Returns the number of DoFs.
        function n = length(this)
           
            n = length(this.gmp);
            
        end

        
        %% Trains the DMP.
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] yd_data: Row vector with the desired potition.
        function train_error = train(this, train_method, Time, yd_data)

            n = this.length();
            if (nargout > 0)
                train_error = zeros(n,1);
                for i=1:n, train_error(i) = this.gmp{i}.train(train_method, Time, yd_data(i,:)); end
            else
                for i=1:n, this.gmp{i}.train(train_method, Time, yd_data(i,:)); end
            end
            
        end

        
        %% Constrained optimization.
        %  @param[in] T: Time duration of the motion.
        %  @param[in] pos_constr: Cell array where the i-th entry is a vector of @GMPConstr position constraints for the i-th gmp. For no constraints pass '[]'.
        %  @param[in] vel_constr: Cell array where the i-th entry is a vector of @GMPConstr velocity constraints for the i-th gmp. For no constraints pass '[]'.
        %  @param[in] accel_constr: Cell array where the i-th entry is a vector of @GMPConstr acceleration constraints for the i-th gmp. For no constraints pass '[]'.
        %  @param[in] opt_set: Cell array where the i-th entry is an object of type @GMPOptSet for setting optimization options for the i-th gmp.
        function constrOpt(this, T, pos_constr, vel_constr, accel_constr, opt_set)
        
            n = this.length();
            for i=1:n
               this.constrOpt(T, pos_constr{i}, vel_constr{i}, accel_constr{i}, opt_set{i});
            end
 
        end
            
        
        %% TODO
        %% Updates the weights so that the generated trajectory passes from the given points.
        % function updateWeights(this, s, z, type, z_var)
        
        
        %% Returns the derivatives of the DMP states.
        %  @param[in] x: phase variable.
        %  @param[in] y: \a y state of the this.
        %  @param[in] z: \a z state of the this.
        %  @param[in] y_c: coupling term for the dynamical equation of the \a y state.
        %  @param[in] z_c: coupling term for the dynamical equation of the \a z state.
        function update(this, s, y, z, y_c, z_c)

            n = this.length();
            
            if (nargin < 5), y_c = zeros(n,1); end
            if (nargin < 6), z_c = zeros(n,1); end
            
            if (length(y_c) == 1), y_c = ones(n,1)*y_c(1); end
            if (length(z_c) == 1), z_c = ones(n,1)*z_c(1); end
            
            for i=1:n
                this.gmp{i}.update(s, y(i), z(i), y_c(i), z_c(i));
                this.y_dot(i) = this.gmp{i}.getYdot();
                this.z_dot(i) = this.gmp{i}.getZdot();
            end

        end

        
        function y_dot = getYdot(this), y_dot = this.y_dot; end
        function z_dot = getZdot(this), z_dot = this.z_dot; end
        
        
        %% Returns the DMP's acceleration.
        function y_ddot = getYddot(this, yc_dot)
            
            n = this.length();
            if (nargin < 2), yc_dot = zeros(n,1); end
            if (length(yc_dot)==1), yc_dot = ones(n,1)*yc_dot(1); end
            
            y_ddot = zeros(n,1);
            for i=1:n, y_ddot(i) = this.gmp{i}.getYddot(yc_dot(i)); end

        end
        
        
        %% Calculates the DMP's acceleration.
        function y_ddot = calcYddot(this, s, y, y_dot, yc, zc, yc_dot)

            n = this.length();
            if (nargin < 5), yc = zeros(n,1); end
            if (nargin < 6), zc = zeros(n,1); end
            if (nargin < 7), yc_dot = zeros(n,1); end
            
            if (length(yc)==1), yc = ones(n,1)*yc(1); end
            if (length(zc)==1), zc = ones(n,1)*zc(1); end
            if (length(yc_dot)==1), yc_dot = ones(n,1)*yc_dot(1); end

            y_ddot = zeros(n,1);
            for i=1:n, y_ddot(i) = this.gmp{i}.calcYddot(s, y(i), y_dot(i), yc(i), zc(i), yc_dot(i)); end

        end
        
        
        %% Returns the number of kernels for the i-th gmp.
        function n_ker = numOfKernels(this, i)
            
            n_ker = this.gmp{i}.numOfKernels();
            
        end

        
        %% Sets the initial position.
        function setY0(this, y0)
            
            n = this.length();
            for i=1:n, this.gmp{i}.setY0(y0(i)); end
            
        end
        
        
        %% Set goal position.
        function setGoal(this, g)

            n = this.length();
            for i=1:n, this.gmp{i}.setGoal(g(i)); end
          
        end

        
        %% Creates a deep copy of this object
        function cp_obj = deepCopy(this)
            
            % Make a shallow copy of all properties
            cp_obj = this.copy();
            % Make a deep copy of the pointers
            n = this.length();
            cp_obj.gmp = cell(n,1);
            for i=1:n, cp_obj.gmp{i} = this.deepCopy(); end

        end

        
        function p_ref = getYd(this, x)
            
            n = this.length();
            p_ref = zeros(n,1);
            for i=1:n, p_ref(i) = this.gmp{i}.getYd(x); end
            
        end
        
        
        function p_ref_dot = getYdDot(this, x, x_dot)
            
            n = this.length();
            p_ref_dot = zeros(n,1);
            for i=1:n, p_ref_dot(i) = this.gmp{i}.getYdDot(x, x_dot); end
            
        end
        
        
        function p_ref_ddot = getYdDDot(this, x, x_dot, x_ddot)
            
            if (nargin < 4), x_ddot = 0; end
            
            n = this.length();
            p_ref_ddot = zeros(n,1);
            for i=1:n, p_ref_ddot(i) = this.gmp{i}.getYdDDot(x, x_dot, x_ddot); end
            
        end
        
        
    end
    
    
    properties (Access = public)

        gmp

    end
    
    
    properties (Access = protected)
        
        %% output state
        y_dot % position derivative
        z_dot % scaled velocity derivative

    end
    
end
