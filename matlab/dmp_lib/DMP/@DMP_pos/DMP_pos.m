%% DMP_pos class
%  For encoding Cartesian Orientation.
%


classdef DMP_pos < matlab.mixin.Copyable

    methods  (Access = public)
        %% DMP_pos constructor.
        %  @param[in] N_kernels: 3x3 matrix where each row contains the kernels for position,
        %                       velocity and acceleration and each row for x, y and z coordinate.
        %                       If 'N_kernels' is a column vector then every coordinate will have the
        %                       same number of kernels.
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] shape_attr_gating_ptr: Pointer to a DMP gating function object.
        %
        function this = DMP_pos(dmp_type, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr)
            
            if (nargin < 5), can_clock_ptr = CanonicalClock(); end
            if (nargin < 6), shape_attr_gating_ptr=SigmoidGatingFunction(1.0, 0.5); end
            
            if (isscalar(N_kernels)), N_kernels = ones(3,1)*N_kernels; end
            if (isscalar(a_z)), a_z = ones(3,1)*a_z; end
            if (isscalar(b_z)), b_z = ones(3,1)*b_z; end

            this.can_clock_ptr = can_clock_ptr;
            this.shape_attr_gating_ptr = shape_attr_gating_ptr;

            for i=1:3
                if (dmp_type == DMP_TYPE.STD)
                    this.dmp{i} = DMP(N_kernels(i), a_z(i), b_z(i), can_clock_ptr, shape_attr_gating_ptr);
                elseif (dmp_type == DMP_TYPE.BIO)
                    this.dmp{i} = DMP_bio(N_kernels(i), a_z(i), b_z(i), can_clock_ptr, shape_attr_gating_ptr);
                else
                    error('[DMP_pos::DMP_pos]: Unsopported DMP type!');
                end
            end

        end
        
        
        %% Trains the DMP_pos.
        %  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
        %  @param[in] Time: 1xN row vector with the timestamps of the training data points.
        %  @param[in] Pd_data: 3xN matrix with desired position at each column for each timestep.
        %  @param[in] dPd_data: 3xN matrix with desired velocity at each column for each timestep.
        %  @param[in] ddPd_data: 3xN matrix with desired acceleration at each column for each timestep.
        %  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
        function train_err = train(this, train_method, Time, Pd_data, dPd_data, ddPd_data)
            
            tau = Time(end);
            this.setTau(tau);

            if (nargout > 0)
                train_err = zeros(3,1);
                for i=1:3
                    train_err(i) = this.dmp{i}.train(train_method, Time, Pd_data(i,:), dPd_data(i,:), ddPd_data(i,:));
                end
            else
                for i=1:3
                    this.dmp{i}.train(train_method, Time, Pd_data(i,:), dPd_data(i,:), ddPd_data(i,:));
                end
            end

        end
        
        %% Calculates the derivatives of the DMP states. The derivatives can then be
        %% retrieved with 'getXdot', 'getYdot' and 'getZdot'.
        %  @param[in] x: phase variable.
        %  @param[in] Y: 'y' state of the DMP.
        %  @param[in] Z: 'z' state of the DMP.
        %  @param[in] Y0: Initial position.
        %  @param[in] Yg: Goal position.
        %  @param[in] Y_c: Coupling term for the dynamical equation of the 'y' state.
        %  @param[in] Z_c: Coupling term for the dynamical equation of the 'z' state.
        function update(this, x, Y, Z, Y0, Yg, Y_c, Z_c)
            
            if (nargin < 7), Y_c=zeros(3,1); end
            if (nargin < 8), Z_c=zeros(3,1); end
            
            for i=1:3, this.dmp{i}.update(x, Y(i), Z(i), Y0(i), Yg(i), Y_c(i), Z_c(i)); end
            
            this.dZ = zeros(3,1);
            this.dY = zeros(3,1);
            for i=1:3
                this.dY(i) = this.dmp{i}.getYdot();
                this.dZ(i) = this.dmp{i}.getZdot();    
            end
            this.dx = this.phaseDot(x);

        end
        
        function ddY = calcYddot(this, x, Y, dY, Y0, Yg, tau_dot, Yc, Zc, Yc_dot)
            
            if (nargin < 7), tau_dot = 0; end
            if (nargin < 8), Yc = zeros(3,1); end
            if (nargin < 9), Zc = zeros(3,1); end
            if (nargin < 10), Yc_dot = zeros(3,1); end

            ddY = zeros(3,1);
            for i=1:3
               ddY(i) = this.dmp{i}.calcYddot(x, Y(i), dY(i), Y0(i), Yg(i), tau_dot, Yc(i), Zc(i), Yc_dot(i)); 
            end
            
        end

        
        %% Returns the acceleration for the given input state defined by the timestamp,
        %  the orientation, the angular velocity and acceleration, the initial and target orientation
        %  and an optinal coupling term.
        %  @param[in] x: phase variable.
        %  @param[in] P: Current position.
        %  @param[in] dP: Current velocity.
        %  @param[in] P0: Initial position.
        %  @param[in] Pg: Goal position.
        %  @param[in] Z_c: Coupling term. (optional, default=arma::vec().zeros(3))
        %  @return ddP: Acceleration.
        %
        function Yddot = getYddot(this, tau_dot, Yc_dot)
            
            if (nargin < 2), tau_dot = 0; end
            if (nargin < 3), Yc_dot = zeros(3,1); end
            
            Yddot = zeros(3,1);
            for i=1:3
                Yddot(i) = (this.dmp{i}.getZdot() + Yc_dot - tau_dot*this.dmp{i}.getYdot()) / this.dmp{i}.getTau(); 
            end
            
        end
        

        %% Returns the time scaling factor.
        %  @return: The time scaling factor.
        %
        function tau = getTau(this)

            tau = this.can_clock_ptr.getTau();

        end
        
        
        %% Sets the time scaling factor.
        %  @param[in] tau: The time scaling factor.
        %
        function setTau(this, tau)

            this.can_clock_ptr.setTau(tau);

        end
        
        
        %% Returns the phase variable corresponding to the given time instant.
        %  @param[in] t: The time instant.
        %  @return: The phase variable for time 't'.
        %
        function x = phase(this, t)
            
            x = this.can_clock_ptr.getPhase(t);

        end
        
        
        %% Returns the derivative of the phase variable.
        %  @param[in] x: The phase variable.
        %  @return: The derivative of the phase variable.
        %
        function dx = phaseDot(this, x)
            
            dx = this.can_clock_ptr.getPhaseDot(x);

        end
        
        function dx = getXdot(this), dx=this.dx; end
        function dY = getYdot(this), dY=this.dY; end
        function dZ = getZdot(this), dZ=this.dZ; end
        
        function can_clock_ptr = getCanClockPtr(this), can_clock_ptr = this.can_clock_ptr; end
        
    end
    
    
    properties  (Access = private)
        
        dmp % 3x1 vector of DMP producing the desired trajectory for each x, y and z dimensions.

        can_clock_ptr % handle (pointer) to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor
        
        dZ
        dY
        dx
    end
    
end
