%% DMPo class
%  For encoding Cartesian Orientation.
%


classdef DMPo < matlab.mixin.Copyable
    
    methods  (Access = public)
        %% DMPo constructor.
        %  @param[in] N_kernels: 3x3 matrix where each row contains the kernels for position,
        %                       velocity and acceleration and each row for x, y and z coordinate.
        %                       If 'N_kernels' is a column vector then every coordinate will have the
        %                       same number of kernels.
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] shape_attr_gating_ptr: Pointer to a DMP gating function object.
        %
        function this = DMPo(dmp_type, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr)

            if (nargin < 5), can_clock_ptr = CanonicalClock(); end
            if (nargin < 6), shape_attr_gating_ptr = SigmoidGatingFunction(1.0, 0.5); end
            
            if (isscalar(N_kernels)), N_kernels = ones(3,1)*N_kernels; end
            if (isscalar(a_z)), a_z = ones(3,1)*a_z; end
            if (isscalar(b_z)), b_z = ones(3,1)*b_z; end

            this.can_clock_ptr = can_clock_ptr;
            this.shape_attr_gating_ptr = shape_attr_gating_ptr;
            
            this.Q0 = [1 0 0 0]';
            
            if (dmp_type == DMP_TYPE.STD)
                for i=1:3
                    this.dmp{i} = DMP(N_kernels(i), a_z(i), b_z(i), can_clock_ptr, shape_attr_gating_ptr);
                end
            elseif (dmp_type == DMP_TYPE.BIO)
                for i=1:3
                    this.dmp{i} = DMP_bio(N_kernels(i), a_z(i), b_z(i), can_clock_ptr, shape_attr_gating_ptr);
                end
            else
                error('[DMPo::DMPo]: Unsupported DMP type!');
            end
            
            for i=1:length(this.dmp), this.dmp{i}.setY0(0); end

        end
        
        
        %% Trains the DMPo.
        %  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
        %  @param[in] Time: 1xN row vector with the timestamps of the training data points.
        %  @param[in] Quat_data: 3xN matrix with desired position at each column for each timestep.
        %  @param[in] rotVel_data: 3xN matrix with desired velocity at each column for each timestep.
        %  @param[in] rotAccel_data: 3xN matrix with desired acceleration at each column for each timestep.
        %  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
        function train_err = train(this, train_method, Time, Quat_data, rotVel_data, rotAccel_data)

            tau = Time(end);
            this.setTau(tau);

            n_data = length(Time);
            q_data = zeros(3, n_data);
            qdot_data = zeros(3, n_data);
            qddot_data = zeros(3, n_data);

            Q0 = Quat_data(:,1);
            
            this.setQ0(Q0);

            for j=1:n_data
               Q1 = DMPo.quatTf(Quat_data(:,j), Q0);
               q_data(:,j) = quatLog(Q1);
               qdot_data(:,j) = DMPo.rotVel2qdot(rotVel_data(:,j), Q1);
               qddot_data(:,j) = DMPo.rotAccel2qddot(rotAccel_data(:,j), rotVel_data(:,j), Q1);
            end

            if (nargout > 0)
                train_err = zeros(3,1);
                for i=1:3
                    train_err(i) = this.dmp{i}.train(train_method, Time, q_data(i,:), qdot_data(i,:), qddot_data(i,:));
                end
            else
                for i=1:3
                    this.dmp{i}.train(train_method, Time, q_data(i,:), qdot_data(i,:), qddot_data(i,:));
                end
            end

        end
      

        function setQ0(this, Q0)
            
            this.Q0 = Q0; 
            
        end

        
        function y = getY(this, Q), y = DMPo.quat2q(Q, this.Q0); end

        
        function z = getZ(this, rotVel, Q)

            Q1 = DMPo.quatTf(Q, this.Q0);
            qdot = DMPo.rotVel2qdot(rotVel, Q1);
            dy = qdot;

            z = this.getTau()*dy; 

        end

        
        %% Calculates the derivatives of the DMP states. The derivatives can then be
        %% retrieved with 'getDx', 'getdeo' and 'getddeo'.
        %  @param[in] x: phase variable.
        %  @param[in] Y: 'y' state of the DMP.
        %  @param[in] Z: 'z' state of the DMP.
        %  @param[in] Y0: Initial position.
        %  @param[in] Yc: Coupling term for the deonamical equation of the 'y' state.
        %  @param[in] Zc: Coupling term for the deonamical equation of the 'z' state.
        function update(this, x, Y, Z, G, Yc, Zc)

            if (nargin < 6), Yc = zeros(3,1); end
            if (nargin < 7), Zc = zeros(3,1); end

            if (isscalar(Yc)), Yc = ones(3,1)*Yc; end
            if (isscalar(Zc)), Zc = ones(3,1)*Zc; end

            for i=1:3, this.dmp{i}.update(x, Y(i), Z(i), G(i), Yc(i), Zc(i)); end

            this.dY = zeros(3,1);
            this.dZ = zeros(3,1);
            for i=1:3
                this.dY(i) = this.dmp{i}.getYdot();
                this.dZ(i) = this.dmp{i}.getZdot();
            end
            this.dx = this.phaseDot(x);

        end
        
        function dx = getXdot(this), dx=this.dx; end
        function dy = getYdot(this), dy=this.dY; end
        
        function ddy = getYddot(this, tau_dot, yc_dot)

            if (nargin < 3), tau_dot=0; end
            if (nargin < 3), yc_dot=0; end
            ddy = (this.getZdot() - tau_dot*this.getYdot() + yc_dot)/this.getTau(); 

        end
        
        function dz = getZdot(this), dz=this.dZ; end

        function rotVel = getRotVel(this, Q)

            Q1 = DMPo.quatTf(Q, this.Q0);
            qdot = this.getYdot(); 
            rotVel = DMPo.qdot2rotVel(qdot, Q1);

        end
        
        function rotAccel = getRotAccel(this, Q, tau_dot, yc_dot)
            
            if (nargin < 3), tau_dot=0; end
            if (nargin < 4), yc_dot=0; end

            Q1 = DMPo.quatTf(Q, this.Q0);
            qddot = this.getYddot(tau_dot, yc_dot);
            rotVel = this.getRotVel(Q);
            rotAccel = DMPo.qddot2rotAccel(qddot, rotVel, Q1);

        end
        
        function rotAccel = calcRotAccel(this, x, Q, rotVel, Qg, tau_dot, yc, zc, yc_dot)
            
            if (nargin < 6), tau_dot = 0; end
            if (nargin < 7), yc = zeros(3,1); end
            if (nargin < 8), zc = zeros(3,1); end
            if (nargin < 9), yc_dot = zeros(3,1); end

            a_z = [this.dmp{1}.a_z; this.dmp{2}.a_z; this.dmp{3}.a_z];
            b_z = [this.dmp{1}.b_z; this.dmp{2}.b_z; this.dmp{3}.b_z];
            tau = this.getTau();

            qg = DMPo.quat2q(Qg, this.Q0);

            Q1 = DMPo.quatTf(Q, this.Q0);
            q = DMPo.quat2q(Q, this.Q0);
            invQ1 = quatInv(Q1);

            JQq = DMPo.jacobQq(Q1);
            JQq_dot = DMPo.jacobDotQq(Q1, rotVel);

            fo = zeros(3,1);
            for i=1:3, fo(i) = this.dmp{i}.shapeAttractor(x, qg(i)); end

            qdot = DMPo.rotVel2qdot(rotVel, Q1);
            qddot = (a_z.*b_z.*(qg - q) - tau*a_z.*qdot + a_z.*yc + fo + tau*yc_dot - tau*tau_dot*qdot + zc)/tau^2;
            
            rotAccel = 2*quatProd(JQq_dot*qdot + JQq*qddot, invQ1);
            rotAccel = rotAccel(2:4);

        end

        
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
        %  @param[in] Q0: The initial quaternion.
        %  @return: The derivative of the orientation error.
        %
        function Q1 = quatTf(Q, Q0), Q1 = quatProd(Q, quatInv(Q0)); end

        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        function q = quat2q(Q, Q0), q = quatLog(DMPo.quatTf(Q,Q0)); end
        
        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        function Q = q2quat(q, Q0), Q = quatProd( quatExp(q), Q0); end
        
        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        function qdot = rotVel2qdot(rotVel, Q1)
            
            JqQ = DMPo.jacobqQ(Q1);
            qdot = 0.5*JqQ * quatProd([0; rotVel], Q1);

        end
        
        
        %% Given a quaternion, a target quaternion and the quaternion error velocity w.r.t the
        %% target, returns the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] deo: Quaternion error velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The rotational velocity of the quaternion.
        %
        function rotVel = qdot2rotVel(qdot, Q1)
            
            JQq = DMPo.jacobQq(Q1);
            rotVel = 2 * quatProd( JQq*qdot, quatInv(Q1) );
            rotVel = rotVel(2:4);

        end
        
        
        %% Given a quaternion, its rotational velocity and acceleration and a target quaternion,
        %% returns the second derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotAccel: The quaternion's rotational acceleration.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The second derivative of the orientation error.
        %
        function qddot = rotAccel2qddot(rotAccel, rotVel, Q1)
            
            rotVelQ = [0; rotVel];
            rotAccelQ = [0; rotAccel];

            J = DMPo.jacobqQ(Q1);
            Jdot = DMPo.jacobDotqQ(Q1, rotVel);

            qddot = 0.5 * (Jdot * quatProd(rotVelQ, Q1) + J * quatProd( rotAccelQ+0.5*quatProd(rotVelQ,rotVelQ), Q1 ) );

        end
        
        
        %% Given a quaternion, its rotational velocity and acceleration and a target quaternion,
        %% returns the second derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotAccel: The quaternion's rotational acceleration.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The second derivative of the orientation error.
        %
        function rotAccel = qddot2rotAccel(qddot, rotVel, Q1)

            qdot = DMPo.rotVel2qdot(rotVel, Q1);
            invQ1 = quatInv(Q1);
            J = DMPo.jacobQq(Q1);
            Jdot = DMPo.jacobDotQq(Q1, rotVel);

            rotAccel = 2 * ( quatProd( Jdot*qdot+J*qddot, invQ1 ) );
            rotAccel = rotAccel(2:4);

        end
        
        
        %% Returns the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @return: Jacobian from derivative of orientation error to quaternion derivative.
        %
        function JQq = jacobQq(Q1)

            if (abs(Q1(1)-1) <= DMPo.zero_tol)
                JQq = [zeros(1, 3); eye(3,3)];
                return;
            end

            w = Q1(1);
            v = Q1(2:4);
            norm_v = norm(v);
            eta = v / norm_v;
            s_th = norm_v;
            c_th = w;
            th = atan2(s_th, c_th);
            Eta = eta*eta';

            JQq = zeros(4,3);
            JQq(1,:) = -0.5 * s_th * eta';
            JQq(2:4,:) = 0.5 * ( (eye(3,3) - Eta)*s_th/th + c_th*Eta );

        end
        
        
        %% Returns the Jacobian from the quaternion derivative to the derivative of the orientation error.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @return: Jacobian from quaternion derivative to derivative of orientation error.
        %
        function JqQ = jacobqQ(Q1)
            
            if (abs(Q1(1)-1) <= DMPo.zero_tol)
                JqQ = [zeros(3,1) eye(3,3)];
                return;
            end

            w = Q1(1);
            v = Q1(2:4);
            norm_v = norm(v);
            eta = v / norm_v;
            s_th = norm_v;
            c_th = w;
            th = atan2(s_th, c_th);

            JqQ = zeros(3,4);
            JqQ(:,1) = 2*eta*(th*c_th - s_th)/s_th^2;
            JqQ(:,2:4) = 2*eye(3,3)*th/s_th;

        end
       
        
        %% Returns the time derivative of the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @param[in] deo: The derivative of the orientation error.
        %  @return: The derivative of the Jacobian from derivative of orientation error to quaternion derivative.
        %
        function JqQ_dot = jacobDotqQ(Q1, rotVel)

            qdot = DMPo.rotVel2qdot(rotVel, Q1);

            if (abs(Q1(1)-1) <= DMPo.zero_tol)
                JqQ_dot = [-qdot/3 zeros(3,3)];
                return;
            end

            w = Q1(1);
            v = Q1(2:4);
            norm_v = norm(v);
            eta = v / norm_v;
            s_th = norm_v;
            c_th = w;
            th = atan2(s_th, c_th);
            Eta = eta*eta';
            temp = (th*c_th-s_th)/s_th^2;

            JqQ_dot = zeros(3,4);
            JqQ_dot(:,1) = ((-th/s_th - 2*c_th*temp/s_th)*Eta + temp*(eye(3,3)-Eta)/th)*qdot;
            JqQ_dot(:,2:4) = -temp*dot(eta,qdot)*eye(3,3);

        end
        
        
        %% Returns the time derivative of the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @param[in] deo: The derivative of the orientation error.
        %  @return: The derivative of the Jacobian from derivative of orientation error to quaternion derivative.
        %
        function JQq_dot = jacobDotQq(Q1, rotVel)
            
            qdot = DMPo.rotVel2qdot(rotVel, Q1);

            if (abs(Q1(1)-1) <= DMPo.zero_tol)
                JQq_dot = [-qdot'/4; zeros(3,3)];
                return;
            end

            w = Q1(1);
            v = Q1(2:4);
            norm_v = norm(v);
            eta = v / norm_v;
            s_th = norm_v;
            c_th = w;
            th = atan2(s_th, c_th);
            Eta = eta*eta';
            I_eta = eye(3,3) - Eta;
            temp = ((th*c_th-s_th)/th^2);

            JQq_dot = zeros(4,3);
            JQq_dot(1,:) = -0.25 * qdot' * (c_th*Eta + (s_th/th)*I_eta);
            JQq_dot(2:4,:) = 0.25*dot(eta,qdot)*( temp*I_eta - s_th*Eta ) + 0.25*temp*( eta*(qdot'*I_eta) + (I_eta*qdot)*eta' );

        end
        
        
    end
    
    
    properties (Constant)  
        
          zero_tol = 1e-15;
          
    end
    
    properties  (Access = public)
        
        dmp % 3x1 vector of DMP producing the desired trajectory for each x, y and z axes of the orientation error.

        can_clock_ptr % handle (pointer) to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor
        
    end
       
    properties  (Access = protected)
        
        Q0
        
        dZ % second derivative of the orientation error
        dY % first derivative of the orientation error
        dx % pahse variable derivative
        
    end
    
end
