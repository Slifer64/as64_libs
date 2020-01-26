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
            
            this.wsog = WSoG(N_kernels, kernels_std_scaling);
            
            this.setY0(0);
            this.setGoal(1);
            
        end

        
        %% Trains the DMP.
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] yd_data: Row vector with the desired potition.
        function train_error = train(this, train_method, Time, yd_data)
        
            this.taud = Time(end);
            
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
        function n_ker = numOfKernels(this), n_ker = length(this.wsog.w); end

        
        %% Sets the initial position.
        function setY0(this, y0)
            
            this.wsog.setStartValue(y0); 
            
        end
        
        
        %% Set goal position.
        function setGoal(this, g)
            
            this.wsog.setFinalValue(g); 
            
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
          
        
        function p_ref = getYd(this, x)
            
            p_ref = this.wsog.output(x);
            
        end
        
        function p_ref_dot = getYdDot(this, x)
            
            x_dot = this.phaseDot(x);
            p_ref_dot = this.wsog.outputDot(x, x_dot);
            
        end
        
        function p_ref_ddot = getYdDDot(this, x)
            
            x_dot = this.phaseDot(x);
            x_ddot = 0;
            p_ref_ddot = this.wsog.outputDDot(x, x_dot, x_ddot);
            
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
            y0_d = this.wsog.getStartDemoValue();
            y0 = this.wsog.getStartValue();
            spat_s = this.wsog.getSpatialScaling();
            
            yd = (this.wsog.output(x) + spat_s*y0_d - y0) / spat_s;
            yd_dot = this.wsog.outputDot(x, x_dot) / spat_s;
            yd_ddot = this.wsog.outputDDot(x, x_dot, x_ddot) / spat_s;
            
            % yd_dot is already the scaled velocity, i.e. yd_dot = dyd * taud/tau
            % that's why we mutliply do yd_dot*tau = dyd * taud which is
            % what we want below. Accordingly for the yd_ddot
            gd = this.wsog.getFinalDemoValue();
            f = tau^2*yd_ddot + this.a_z*tau*yd_dot - this.a_z*this.b_z*(gd - yd);
            
        end

        
        %% Returns the forcing term scaling
        function f_scale = forcingTermScaling(this)
           
            f_scale = this.wsog.getSpatialScaling();
            
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
        
        taud % time duration of demo
        
        %% output state
        dy % position derivative
        dz % scaled velocity derivative
        dx % phase variable derivative

    end
    
end
