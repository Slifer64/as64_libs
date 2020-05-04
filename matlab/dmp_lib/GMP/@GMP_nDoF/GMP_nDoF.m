%% N-DoF GMP class
%  Generalized movement primitive.
%

classdef GMP_nDoF < matlab.mixin.Copyable
    
    methods (Access = public)
        
        %% GMP constructor.
        %  @param[in] n: number of degrees of freedom.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] D: damping.
        %  @param[in] K: stiffness.
        %  @param[in] kernels_std_scaling: Scaling for std of kernels (optional, default=1).
        %  \note: Each of the arguments 'N_kernels', 'D', 'K' can be scalar or a nx1 vector.
        function this = GMP_nDoF(n, N_kernels, D, K, kernels_std_scaling)
                
            if (nargin < 5), kernels_std_scaling = 1.0; end
            
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
        %  return: number of DoFs.
        function n = length(this)
           
            n = length(this.gmp);
            
        end

        
        %% Trains the GMP.
        %  @param[in] train_method: the training method to use, as a string ('LWR', 'LS').
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] yd_data: Matrix with the desired potition for each DoF in each row.
        %  @param[out] train_error: The training error expressed as the mse error.
        function train_error = train(this, train_method, Time, yd_data)

            n = this.length();
            if (nargout > 0)
                train_error = zeros(n,1);
                for i=1:n, train_error(i) = this.gmp{i}.train(train_method, Time, yd_data(i,:)); end
            else
                for i=1:n, this.gmp{i}.train(train_method, Time, yd_data(i,:)); end
            end
            
        end


        %% Calculates the time derivatives of the GMP's states.
        %  @param[in] s: Vector with the phase variable state, i.e. s = [x; x_dot; x_ddot].
        %  @param[in] y: 'y' state of the GMP.
        %  @param[in] z: 'z' state of the GMP.
        %  @param[in] y_c: coupling term for the dynamical equation of the 'y' state (optional, default=0).
        %  @param[in] z_c: coupling term for the dynamical equation of the 'z' state (optional, default=0).
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

        
        %% Returns the 'y' state time derivative.
        %  Call after @update.
        %  @return: time derivative of 'y' state.
        function y_dot = getYdot(this)
            y_dot = this.y_dot; 
        end
        
        
        %% Returns the 'z' state time derivative.
        %  Call after @update.
        %  @return: time derivative of 'z' state.
        function z_dot = getZdot(this)
            z_dot = this.z_dot; 
        end
        
        
        %% Returns the GMP's acceleration.
        %  Call after @update.
        %  @param[in] yc_dot: time derivative of 'y' state coupling (optional, default=0).
        %  @return: acceleration.
        function y_ddot = getYddot(this, yc_dot)
            
            n = this.length();
            if (nargin < 2), yc_dot = zeros(n,1); end
            if (length(yc_dot)==1), yc_dot = ones(n,1)*yc_dot(1); end
            
            y_ddot = zeros(n,1);
            for i=1:n, y_ddot(i) = this.gmp{i}.getYddot(yc_dot(i)); end

        end
        
        
        %% Calculates the GMP's acceleration.
        %  @param[in] s: Vector with the phase variable state, i.e. s = [x; x_dot; x_ddot].
        %  @param[in] y: 'y' state of the GMP.
        %  @param[in] y_dot: time derivative of 'y' state.
        %  @param[in] y_c: coupling term for the dynamical equation of the 'y' state (optional, default=0).
        %  @param[in] z_c: coupling term for the dynamical equation of the 'z' state (optional, default=0).
        %  @param[in] yc_dot: time derivative of the 'y' state coupling (optional, default=0).
        %  @return: acceleration.
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
        %  @param[in] i: index for the i-th DoF.
        %  @return: number of kernels for the i-th gmp.
        function n_ker = numOfKernels(this, i)
            
            n_ker = this.gmp{i}.numOfKernels();
            
        end

        
        %% Sets the initial position.
        %  @param[in] y0: initial position.
        function setY0(this, y0)
            
            n = this.length();
            for i=1:n, this.gmp{i}.setY0(y0(i)); end
            
        end
        
        
        %% Set goal position.
        %  @param[in] g: goal position.
        function setGoal(this, g)

            n = this.length();
            for i=1:n, this.gmp{i}.setGoal(g(i)); end
          
        end

        
        %% Returns a deep copy of this object.
        %  @return: deep copy of this object.
        function cp_obj = deepCopy(this)
            
            % Make a shallow copy of all properties
            cp_obj = this.copy();
            % Make a deep copy of the pointers
            n = this.length();
            cp_obj.gmp = cell(n,1);
            for i=1:n, cp_obj.gmp{i} = this.deepCopy(); end

        end

        
        %% Returns the scaled desired position.
        %  @param[in] x: phase variable.
        %  @return: scaled desired position.
        function p_ref = getYd(this, x)
            
            n = this.length();
            p_ref = zeros(n,1);
            for i=1:n, p_ref(i) = this.gmp{i}.getYd(x); end
            
        end
        
        
        %% Returns the scaled desired velocity.
        %  @param[in] x: phase variable.
        %  @param[in] x_dot: 1st time derivative of the phase variable.
        %  @return: scaled desired velocity.
        function p_ref_dot = getYdDot(this, x, x_dot)
            
            n = this.length();
            p_ref_dot = zeros(n,1);
            for i=1:n, p_ref_dot(i) = this.gmp{i}.getYdDot(x, x_dot); end
            
        end
        
        
        %% Returns the scaled desired acceleration.
        %  @param[in] x: phase variable.
        %  @param[in] x_dot: 1st time derivative of the phase variable.
        %  @param[in] x_ddot: 2nd time derivative of the phase variable.
        %  @return: scaled desired acceleration.
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
