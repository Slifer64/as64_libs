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
        function this = GMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr)

            % if (nargin < 4), can_clock_ptr = CanonicalClock(); end
            % if (nargin < 5), shape_attr_gating_ptr=SigmoidGatingFunction(1.0, 0.5); end
                
            this.a_z = a_z;
            this.b_z = b_z;
            this.can_clock_ptr = can_clock_ptr;
            this.shape_attr_gating_ptr = shape_attr_gating_ptr;
            
            this.wsog = WSoG(N_kernels);
            
            this.setY0(0.0);
        end

        
        %% Trains the DMP.
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] yd_data: Row vector with the desired potition.
        function train_error = train(this, train_method, Time, yd_data)
        
            this.taud = Time(end);
            this.y0d = yd_data(1);
            this.gd = yd_data(end);
            
            x = Time / Time(end);
            train_error = this.wsog.train(train_method, x, yd_data);
            
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

            tau = this.getTau();
            shape_attr = this.shapeAttractor(x, g);
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

            shape_attr = this.shapeAttractor(x, g);
            goal_attr = this.goalAttractor(x, y, z, g);
            dz = ( goal_attr + shape_attr + zc) / tau;

            ddy = (dz + yc_dot - tau_dot*dy)/tau;

        end
        
        
        %% Returns the number of kernels.
        function n_ker = numOfKernels(this), n_ker = length(this.wsog.w); end

        
        %% Sets the initial position.
        function setY0(this, y0), this.y0 = y0; end

        
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
        function shape_attr = shapeAttractor(this, x, g)
            
            shape_attr = this.shapeAttrGating(x) * this.forcingTermScaling(g) * this.forcingTerm(x);
            
        end
        
        
        %% Returns the partial derivative of the DMP's acceleration wrt to the goal and tau.
        %  @param[in] t: current timestamp.
        %  @param[in] y: position.
        %  @param[in] dy: velocity.
        %  @param[in] y0: initial position.
        %  @param[in] x_hat: phase variable estimate.
        %  @param[in] g_hat: goal estimate.
        %  @param[in] tau_hat: time scale estimate.
        %  @param[out] dC_dtheta: partial derivative of the DMP's acceleration wrt to the goal and tau.
        function dC_dtheta = getAcellPartDev_g_tau(this, t, y, dy, y0, x, g, tau)

            dC_dtheta = zeros(2,1);

            K_dmp = this.a_z*this.b_z;
            D_dmp = this.a_z;
            psi = this.kernelFunction(x);
            sum_psi = sum(psi) + this.zero_tol;
            sum_w_psi = psi'*this.w;
            shape_attr_gat = this.shapeAttrGating(x);

            theta1 = g;
            theta2 = 1/tau;

            dshape_attr_gat_dtheta2 = this.shape_attr_gating_ptr.getPartDev_1oTau(t,x);

            dPsidtheta2 = -2*t*this.h.*(theta2*t-this.c).*psi;
            sum_w_dPsidtheta2 = this.w'*dPsidtheta2;
            dSumWPsi_dtheta2 = (sum_w_dPsidtheta2*sum_psi - sum_w_psi*sum(dPsidtheta2) ) / sum_psi^2;

            dC_dtheta(1) = (K_dmp + shape_attr_gat*sum_w_psi/sum_psi)*theta2^2;

            dC_dtheta(2) = 2*theta2* (K_dmp*(theta1-y) + shape_attr_gat*(theta1-y0)*sum_w_psi/sum_psi) + ...
                -D_dmp*dy + theta2^2*(theta1-y0)*( dshape_attr_gat_dtheta2*sum_w_psi/sum_psi + shape_attr_gat*dSumWPsi_dtheta2 );
            dC_dtheta(2) = dC_dtheta(2)*(-1/tau^2);

        end

        
        %% Creates a deep copy of this object
        function cp_obj = deepCopy(this)
            
            % Make a shallow copy of all properties
            cp_obj = this.copy();
            % Make a deep copy of the pointers
            cp_obj.can_clock_ptr = this.can_clock_ptr.copy();
            cp_obj.shape_attr_gating_ptr = this.shape_attr_gating_ptr.copy();

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
            
            yd = this.wsog.output(x);
            yd_dot = this.wsog.outputDot(x, x_dot);
            yd_ddot = this.wsog.outputDDot(x, x_dot, x_ddot);
            
            f = this.taud^2*yd_ddot + this.a_z*this.taud*yd_dot - this.a_z*this.b_z*(this.gd - yd);
            
        end

        %% Returns the forcing term scaling
        function f_scale = forcingTermScaling(this, g)
           
            f_scale = (g - this.y0) / (this.gd - this.y0d);
            
        end
        
    end
    
    properties (Access = public)
     
        a_z % parameter 'a_z' relating to the spring-damper system
        b_z % parameter 'b_z' relating to the spring-damper system

        can_clock_ptr % pointer to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor
        
        wsog % WSoG object

    end
    
    
    properties (Access = protected)

        y0 % initial position
        y0d % initial demo position
        gd % initial demo goal
        
        taud % time duration of demo
        
        %% output state
        dy % position derivative
        dz % scaled velocity derivative
        dx % phase variable derivative

    end
    
end
