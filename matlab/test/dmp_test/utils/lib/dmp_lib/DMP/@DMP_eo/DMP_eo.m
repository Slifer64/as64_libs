%% DMP_eo class
%  For encoding Cartesian Orientation.
%


classdef DMP_eo < matlab.mixin.Copyable
    
    methods  (Access = public)
        %% DMP_eo constructor.
        %  @param[in] N_kernels: 3x3 matrix where each row contains the kernels for position,
        %                       velocity and acceleration and each row for x, y and z coordinate.
        %                       If 'N_kernels' is a column vector then every coordinate will have the
        %                       same number of kernels.
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] shape_attr_gating_ptr: Pointer to a DMP gating function object.
        %
        function this = DMP_eo(dmp_type, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr)

            if (nargin < 5), can_clock_ptr = CanonicalClock(); end
            if (nargin < 6), shape_attr_gating_ptr = SigmoidGatingFunction(1.0, 0.5); end
            
            if (isscalar(N_kernels)), N_kernels = ones(3,1)*N_kernels; end
            if (isscalar(a_z)), a_z = ones(3,1)*a_z; end
            if (isscalar(b_z)), b_z = ones(3,1)*b_z; end

            this.can_clock_ptr = can_clock_ptr;
            this.shape_attr_gating_ptr = shape_attr_gating_ptr;
            
            this.Q0 = [1 0 0 0]';
            this.Qg = [1 0 0 0]';
            
            if (dmp_type == DMP_TYPE.STD)
                for i=1:3
                    this.dmp{i} = DMP(N_kernels(i), a_z(i), b_z(i), can_clock_ptr, shape_attr_gating_ptr);
                end
            elseif (dmp_type == DMP_TYPE.BIO)
                for i=1:3
                    this.dmp{i} = DMP_bio(N_kernels(i), a_z(i), b_z(i), can_clock_ptr, shape_attr_gating_ptr);
                end
            else
                error('[DMP_eo::DMP_eo]: Unsupported DMP type!');
            end

        end
        
        %% Trains the DMP_eo.
        %  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
        %  @param[in] Time: 1xN row vector with the timestamps of the training data points.
        %  @param[in] Quat_data: 3xN matrix with desired position at each column for each timestep.
        %  @param[in] rotVel_data: 3xN matrix with desired velocity at each column for each timestep.
        %  @param[in] rotAccel_data: 3xN matrix with desired acceleration at each column for each timestep.
        %  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
        train_err = train(this, train_method, Time, Quat_data, rotVel_data, rotAccel_data)

            
        function setQ0(this, Q0)
            
            this.Q0 = Q0; 
            Y0 = DMP_eo.quat2eo(this.Q0, this.Qg);
            for i=1:length(this.dmp), this.dmp{i}.setY0(Y0(i)); end
            
        end
        
        
        function [Y, Z] = setQg(this, Qg, Q, Y, Z, Yc)
            
            if (nargin < 3), Q = [1 0 0 0]'; end
            if (nargin < 4), Y = []; end
            if (nargin < 5), Z = []; end
            if (nargin < 6), Yc = zeros(3,1); end
             
            if (nargout>0)
                
                if (isempty(Y)), Y = DMP_eo.quat2eo(Q, Qg); end
                    
                eo_old = Y;
                Qe_old = quatExp(eo_old);
                
                eo = DMP_eo.quat2eo(Q, Qg);
                Y = eo;
                
                if (nargout>1)
                    
                    if (isempty(Z)), Z = zeros(3,1); end
                    
                    deo_old = (Z + Yc) / this.getTau();
                    vRot = DMP_eo.deo2rotVel(deo_old, Qe_old);
                    deo = DMP_eo.rotVel2deo(vRot, quatExp(eo));
                    Z = this.getTau()*deo - Yc;
                end
                
            end
            
            this.Qg = Qg;
            Y0 = DMP_eo.quat2eo(this.Q0, this.Qg);
            for i=1:length(this.dmp), this.dmp{i}.setY0(Y0(i)); end
            
        end
        
            
        %% Calculates the derivatives of the DMP states. The derivatives can then be
        %% retrieved with 'getDx', 'getdeo' and 'getddeo'.
        %  @param[in] x: phase variable.
        %  @param[in] Y: 'y' state of the DMP.
        %  @param[in] Z: 'z' state of the DMP.
        %  @param[in] Y0: Initial position.
        %  @param[in] Yc: Coupling term for the deonamical equation of the 'y' state.
        %  @param[in] Zc: Coupling term for the deonamical equation of the 'z' state.
        update(this, x, Y, Z, Yc, Zc)
        
        function dx = getXdot(this), dx=this.dx; end
        function dy = getYdot(this), dy=this.dY; end
        
        ddy = getYddot(this, tau_dot, yc_dot)
        
        function dz = getZdot(this), dz=this.dZ; end
        
        rotVel = getRotVel(this, Q, Qg)
        
        rotAccel = getRotAccel(this, Q, Qg, tau_dot, yc_dot);
        
        rotAccel = calcRotAccel(this, x, Q, rotVel, Qg, Q0, tau_dot, yc, zc, yc_dot)
        
        y = getY(this, Q, Qg)
          
        z = getZ(this, rotVel, Q, Qg)
        
        
        %% Returns the time scaling factor.
        %  @return: The time scaling factor.
        %
        function tau = getTau(this), tau = this.can_clock_ptr.getTau(); end
        
        
        %% Sets the time scaling factor.
        %  @param[in] tau: The time scaling factor.
        %
        function setTau(this, tau), this.can_clock_ptr.setTau(tau); end
        
        
        %% Returns the phase variable corresponding to the given time instant.
        %  @param[in] t: The time instant.
        %  @return: The phase variable for time 't'.
        %
        function x = phase(this, t), x = this.can_clock_ptr.getPhase(t); end
        
        
        %% Returns the derivative of the phase variable.
        %  @param[in] x: The phase variable.
        %  @return: The derivative of the phase variable.
        %
        function dx = phaseDot(this, x), dx = this.can_clock_ptr.getPhaseDot(x); end

        
    end
    
    methods (Static)
        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        Qe = quatError(Q, Qg)

        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        eo = quat2eo(Q, Qg)
        
        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        Q = eo2quat(eo, Qg)
        
        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        deo = rotVel2deo(rotVel, Qe)
        
        
        %% Given a quaternion, a target quaternion and the quaternion error velocity w.r.t the
        %% target, returns the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] deo: Quaternion error velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The rotational velocity of the quaternion.
        %
        rotVel = deo2rotVel(deo, Qe)
        
        
        %% Given a quaternion, its rotational velocity and acceleration and a target quaternion,
        %% returns the second derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotAccel: The quaternion's rotational acceleration.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The second derivative of the orientation error.
        %
        ddeo = rotAccel2ddeo(rotAccel, rotVel, Qe)
        
        
        %% Given a quaternion, its rotational velocity and acceleration and a target quaternion,
        %% returns the second derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotAccel: The quaternion's rotational acceleration.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The second derivative of the orientation error.
        %
        rotAccel = ddeo2rotAccel(ddeo, rotVel, Qe)
        
        
        %% Returns the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @return: Jacobian from derivative of orientation error to quaternion derivative.
        %
        J_dQ_deo = jacobDquatDeo(Qe)
        
        
        %% Returns the Jacobian from the quaternion derivative to the derivative of the orientation error.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @return: Jacobian from quaternion derivative to derivative of orientation error.
        %
        J_deo_dQ = jacobDeoDquat(Qe)
       
        
        %% Returns the time derivative of the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @param[in] deo: The derivative of the orientation error.
        %  @return: The derivative of the Jacobian from derivative of orientation error to quaternion derivative.
        %
        dJ_deo_dQ = jacobDotDeoDquat(Qe, rotVel)
        
        
        %% Returns the time derivative of the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @param[in] deo: The derivative of the orientation error.
        %  @return: The derivative of the Jacobian from derivative of orientation error to quaternion derivative.
        %
        dJ_dQ_deo = jacobDotDquatDeo(Qe, rotVel)
        
        
    end
    
    
    properties (Constant)  
        
          zero_tol = 1e-8;
          
    end
    
    properties  (Access = public)
        
        dmp % 3x1 vector of DMP producing the desired trajectory for each x, y and z axes of the orientation error.

        can_clock_ptr % handle (pointer) to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor
        
    end
       
    properties  (Access = protected)
   
        Q0
        Qg
        
        dZ % second derivative of the orientation error
        dY % first derivative of the orientation error
        dx % pahse variable derivative
        
    end
    
end
