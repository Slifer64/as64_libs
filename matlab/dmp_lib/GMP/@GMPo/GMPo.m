%% GMPo class
%  For encoding Cartesian Orientation.
%


classdef GMPo < GMP_nDoF
    
    methods  (Access = public)
        %% GMPo constructor.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] D: damping.
        %  @param[in] K: stiffness.
        %  @param[in] kernels_std_scaling: Scaling for std of kernels (optional, default=2).
        %  \note: Each of the arguments 'N_kernels', 'D', 'K' can be scalar or a 3x1 vector.
        function this = GMPo(N_kernels, D, K, kernels_std_scaling)

            this = this@GMP_nDoF(3, N_kernels, D, K, kernels_std_scaling);

            this.setQ0([1 0 0 0]');

        end
        
        
        %% Trains the GMPo.
        %  @param[in] train_method: the training method to use, as a string ('LWR', 'LS').
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] Quat_data: Matrix with the desired unit quaternion (4x1 vector) in each column.
        %  @param[out] train_error: The training error expressed as the mse error.
        function train_error = train(this, train_method, Time, Quat_data)

            n_data = length(Time);
            this.setQ0(Quat_data(:,1));
            
            qd_data = zeros(3, n_data);
            for j=1:n_data
               Q1 = GMPo.quatTf(Quat_data(:,j), this.Q0);
               qd_data(:,j) = quatLog(Q1);
            end
             
            if (nargout > 0), train_error = train@GMP_nDoF(this, train_method, Time, qd_data);
            else, train@GMP_nDoF(this, train_method, Time, qd_data); end
            
        end
        
        
        %% Calculates the derivatives of the DMP states. 
        %  The derivatives can then be retrieved with 'getXdot', 'getYdot' and 'getZdot'.
        %  @param[in] x: phase variable.
        %  @param[in] Y: 'y' state of the DMP: y=log(Q*Q0^{-1}).
        %  @param[in] Z: 'z' state of the DMP (the scaled ydot state).
        %  @param[in] G: 'g' goal/target of the DMP: y=log(Qg*Q0^{-1}).
        %  @param[in] Yc: Coupling term for the deonamical equation of the 'y' state.
        %  @param[in] Zc: Coupling term for the deonamical equation of the 'z' state.
        % function update(this, s, y, z, y_c, z_c)
        
        
        %% Returns the 'y' state time derivative.
        %  Call after @update.
        %  @return: time derivative of 'y' state.
        % function y_dot = getYdot(this)
        
        
        %% Returns the 'z' state time derivative.
        %  Call after @update.
        %  @return: time derivative of 'z' state.
        % function z_dot = getZdot(this)

        
        %% Returns the GMP's acceleration.
        %  Call after @update.
        %  @param[in] yc_dot: time derivative of 'y' state coupling (optional, default=0).
        %  @return: acceleration.
        % function y_ddot = getYddot(this, yc_dot)
        
        
        %% Calculates the GMP's acceleration.
        %  @param[in] s: Vector with the phase variable state, i.e. s = [x; x_dot; x_ddot].
        %  @param[in] y: 'y' state of the GMP.
        %  @param[in] y_dot: time derivative of 'y' state.
        %  @param[in] y_c: coupling term for the dynamical equation of the 'y' state (optional, default=0).
        %  @param[in] z_c: coupling term for the dynamical equation of the 'z' state (optional, default=0).
        %  @param[in] yc_dot: time derivative of 'y' state coupling (optional, default=0).
        %  @return: acceleration.
        % function y_ddot = calcYddot(this, s, y, y_dot, yc, zc, yc_dot)

        
        %% Returns the number of kernels for the i-th gmp.
        %  @param[in] i: index for the i-th DoF.
        %  @return: number of kernels for the i-th gmp.
        % function n_ker = numOfKernels(this, i)
        
        
        %% Sets the initial orientation.
        %  @param[in] Q0: Initial orientation (as unit quaternion).
        function setQ0(this, Q0)
            
            this.Q0 = Q0;
            
        end

        
        %% Sets the target orientation.
        %  @param[in] Q0: Initial orientation (as unit quaternion).
        function setQg(this, Qg)
            
            qg = GMPo.quat2q(Qg, this.Q0);

            n = this.length();
            for i=1:n, this.gmp{i}.setGoal(qg(i)); end
            
        end

        
        %% Returns a deep copy of this object.
        %  @return: deep copy of this object.
        % function cp_obj = deepCopy(this)
            
        
        %% Returns the scaled desired position.
        %  @param[in] x: phase variable.
        %  @return: scaled desired position.
        % function p_ref = getYd(this, x)
        
        
        %% Returns the scaled desired velocity.
        %  @param[in] x: phase variable.
        %  @param[in] x_dot: 1st time derivative of the phase variable.
        %  @return: scaled desired velocity.
        % function p_ref_dot = getYdDot(this, x, x_dot)
        
        
        %% Returns the scaled desired acceleration.
        %  @param[in] x: phase variable.
        %  @param[in] x_dot: 1st time derivative of the phase variable.
        %  @param[in] x_ddot: 2nd time derivative of the phase variable.
        %  @return: scaled desired acceleration.
        % function p_ref_ddot = getYdDDot(this, x, x_dot, x_ddot)
        
        
        %% Returns the rotational velocity.
        %  Call @update first!
        %  @param[in] Q: the current orientation.
        %  @return: the rotational velocity.
        function rotVel = getRotVel(this, Q)

            Q1 = GMPo.quatTf(Q, this.Q0);
            qdot = this.getYdot(); 
            rotVel = GMPo.qdot2rotVel(qdot, Q1);

        end
        
        
        %% Returns the rotational acceleration.
        %  Call @update first!
        %  @param[in] Q: the current orientation.
        %  @param[in] yc_dot: time derivative of 'yc' coupling term (optional, default=0).
        %  @return: the rotational acceleration.
        function rotAccel = getRotAccel(this, Q, yc_dot)

            if (nargin < 3), yc_dot=0; end

            Q1 = GMPo.quatTf(Q, this.Q0);
            qddot = this.getYddot(yc_dot);
            rotVel = this.getRotVel(Q);
            rotAccel = GMPo.qddot2rotAccel(qddot, rotVel, Q1);

        end
        
        
        %% Calculates the rotational acceleration based on the current input variables.
        %  @param[in] x: phase variable.
        %  @param[in] Q: the current orientation.
        %  @param[in] rotVel: the rotational velocity.
        %  @param[in] Qg: the target orientation.
        %  @param[in] yc: Coupling term fo 'y' state diff-equation (optional, default=0).
        %  @param[in] zc: Coupling term fo 'z' state diff-equation (optional, default=0).
        %  @param[in] yc_dot: time derivative of 'yc' coupling term (optional, default=0).
        %  @return: the rotational acceleration.
        function rotAccel = calcRotAccel(this, s, Q, rotVel, Qg, yc, zc, yc_dot)
            
            if (nargin < 6), yc = zeros(3,1); end
            if (nargin < 7), zc = zeros(3,1); end
            if (nargin < 8), yc_dot = zeros(3,1); end

            D = [this.gmp{1}.D; this.gmp{2}.D; this.gmp{3}.D];
            K = [this.gmp{1}.K; this.gmp{2}.K; this.gmp{3}.K];

            qg = GMPo.quat2q(Qg, this.Q0);

            Q1 = GMPo.quatTf(Q, this.Q0);
            q = GMPo.quat2q(Q, this.Q0);
            invQ1 = quatInv(Q1);

            JQq = GMPo.jacobQq(Q1);
            JQq_dot = GMPo.jacobDotQq(Q1, rotVel);

            fo = zeros(3,1);
            for i=1:3, fo(i) = this.gmp{i}.shapeAttractor(s); end

            qdot = GMPo.rotVel2qdot(rotVel, Q1);
            qddot = K.*(qg - q) - D.*(qdot + yc) + fo + yc_dot + zc;
            
            rotAccel = 2*quatProd(JQq_dot*qdot + JQq*qddot, invQ1);
            rotAccel = rotAccel(2:4);

        end

        
        %% Returns the 'y' state of the DMP based on the current orientation.
        %  @param[in] Q: Current orientation (as unit quaternion).
        function y = getY(this, Q)
            
            y = GMPo.quat2q(Q, this.Q0); 
        
        end

        
        %% Returns the 'z' state of the DMP based on the current rotational velocity and orientation.
        %  @param[in] rotVel: Current rotational velocity.
        %  @param[in] Q: Current orientation (as unit quaternion).
        function z = getZ(this, rotVel, Q)

            Q1 = GMPo.quatTf(Q, this.Q0);
            qdot = GMPo.rotVel2qdot(rotVel, Q1);
            z = qdot; 

        end

        
    end
    
    methods (Static)
        
        %% Expresses a given quaternion w.r.t. the initial orientation. 
        %  @param[in] Q: Orientation as unit quaternion.
        %  @param[in] Q0: Initial quaternion.
        %  @return: Q1 = Orientation w.r.t. Q0, i.e. Q1 = Q*Q0^{-1}.
        function Q1 = quatTf(Q, Q0), Q1 = quatProd(Q, quatInv(Q0)); end

        
        %% Returns the log of a given orientation w.r.t. the initial orientation.
        %  @param[in] Q: Orientation as unit quaternion.
        %  @param[in] Q0: Initial quaternion.
        %  @return: The logarithm of the Q w.r.t. Q0, i.e. q = log(Q*Q0^{-1}).
        function q = quat2q(Q, Q0), q = quatLog(GMPo.quatTf(Q,Q0)); end
        
        
        %% Returns the quaternion Q given the initial orientation Q0 and the log of Q w.r.t. to Q0.
        %  @param[in] q: Logarithm of orientation w.r.t. the initial orientation.
        %  @param[in] Q0: Initial orientation.
        %  @return: The orientation corresponding to log, i.e. Q = exp(q)*Q0
        function Q = q2quat(q, Q0), Q = quatProd( quatExp(q), Q0); end
        

        %% Returns derivative of log given the rotational velocity and orientation (expressed w.r.t. the initial orientation)
        %  @param[in] rotVel: Rotational velocity.
        %  @param[in] Q1: Orientation expressed w.r.t. the initial orientation.
        %  @return: Derivative of log.
        function qdot = rotVel2qdot(rotVel, Q1)
            
            JqQ = GMPo.jacobqQ(Q1);
            qdot = 0.5*JqQ * quatProd([0; rotVel], Q1);

        end
        
        
        function rotVel = qdot2rotVel(qdot, Q1)
            
            JQq = GMPo.jacobQq(Q1);
            rotVel = 2 * quatProd( JQq*qdot, quatInv(Q1) );
            rotVel = rotVel(2:4);

        end
        

        function qddot = rotAccel2qddot(rotAccel, rotVel, Q1)
            
            rotVelQ = [0; rotVel];
            rotAccelQ = [0; rotAccel];

            J = GMPo.jacobqQ(Q1);
            Jdot = GMPo.jacobDotqQ(Q1, rotVel);

            qddot = 0.5 * (Jdot * quatProd(rotVelQ, Q1) + J * quatProd( rotAccelQ+0.5*quatProd(rotVelQ,rotVelQ), Q1 ) );

        end
        

        function rotAccel = qddot2rotAccel(qddot, rotVel, Q1)

            qdot = GMPo.rotVel2qdot(rotVel, Q1);
            invQ1 = quatInv(Q1);
            J = GMPo.jacobQq(Q1);
            Jdot = GMPo.jacobDotQq(Q1, rotVel);

            rotAccel = 2 * ( quatProd( Jdot*qdot+J*qddot, invQ1 ) );
            rotAccel = rotAccel(2:4);

        end
        
        
        %% Returns the Jacobian from the derivative of log to the derivative of Q.
        %  @param[in] Q1: The orientation w.r.t. the initial orientation.
        %  @return: Jacobian.
        function JQq = jacobQq(Q1)

            if ( (1-abs(Q1(1))) <= GMPo.zero_tol)
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
        
        
        %% Returns the Jacobian from the derivative of Q to the derivative of log.
        %  @param[in] Q1: The orientation w.r.t. the initial orientation.
        %  @return: Jacobian.
        function JqQ = jacobqQ(Q1)
            
            if ( (1-abs(Q1(1))) <= GMPo.zero_tol)
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
       
        
        %% Returns the time derivative of the Jacobian from the derivative of log to the derivative of Q.
        %  @param[in] Q1: The orientation w.r.t. the initial orientation.
        %  @return: Jacobian time derivative.
        function JqQ_dot = jacobDotqQ(Q1, rotVel)

            qdot = GMPo.rotVel2qdot(rotVel, Q1);

            if ( (1-abs(Q1(1))) <= GMPo.zero_tol)
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
        
        
        %% Returns the time derivative of the Jacobian from the derivative of Q to the derivative of log.
        %  @param[in] Q1: The orientation w.r.t. the initial orientation.
        %  @return: Jacobian time derivative.
        function JQq_dot = jacobDotQq(Q1, rotVel)
            
            qdot = GMPo.rotVel2qdot(rotVel, Q1);

            if ( (1-abs(Q1(1))) <= GMPo.zero_tol)
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
    
       
    properties  (Access = protected)
        
        Q0 % initial orientation
        
    end
    
end