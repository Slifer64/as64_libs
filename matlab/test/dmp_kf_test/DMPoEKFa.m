%% EKF class
% Implementation of the discrete Extended Kalman Filter algorithm.
% - The Jacobian of the state transition and measurement functions can either
% be provided or approximated numerically.
% - A fading memory coefficient can be defined.
% - Enforcement of linear constraints can be imposed on the estimated parameteres.
%


classdef DMPoEKFa < matlab.mixin.Copyable
    
    methods
        %% Extended Kalman Filter constructor
        %  @param[in] N_params: Number of states to estimate.
        %  @param[in] N_out: Number of outputs.
        %  @param[in] stateTransFun_ptr: Pointer to state transition function. It should accept a vector (the parameters) and 
        %                                  a void pointer (cookie) to pass optinally extra arguments to the state transition function.
        %  @param[in] msrFun_ptr: Pointer to measurement function. It should accept a vector (the parameters) and 
        %                                  a void pointer (cookie) to pass optinally extra arguments to the measurement function.
        function this = DMPoEKFa(dmp, Ts)
            
            this.Ts = Ts;
            this.dmp = dmp;
            this.Az = diag([dmp.dmp{1}.a_z, dmp.dmp{2}.a_z, dmp.dmp{3}.a_z]);
            this.Bz = diag([dmp.dmp{1}.b_z, dmp.dmp{2}.b_z, dmp.dmp{3}.b_z]);
            
            N_params = 10;
            N_out = 6;
            
            this.theta = zeros(N_params);
            this.P = zeros(N_params, N_params);
            
            this.setProcessNoiseCov(eye(N_params,N_params)*1e-5);
            this.setMeasureNoiseCov(eye(N_out,N_out)*0.05);
            
            this.setFadingMemoryCoeff(1.0);
            
            this.enableParamsContraints(false);
            this.setParamsConstraints([],[]);
            
            this.setPartDerivStep(0.001);
            
            % this.stateTransFun_ptr = @this.stateTransFun;
            % this.msrFun_ptr = @this.msrFun;
            
            % this.msrFunJacob_ptr = @this.msrFunJacob;
            % this.stateTransFunJacob_ptr = @this.stateTransFunJacob;

        end
        
        
        %% Sets the fading memory coefficient. 
        %  Note if the continuous time fading memory coefficient is 'a' then 
        %  the discrete one is 'a_p = exp(a*Ts)' where Ts is the sampling period.
        %  @param[in] a_p: Fading memory coefficient.
        function setFadingMemoryCoeff(this, a_p)
            
            this.a_p = a_p;
            
        end
        
        
        %% Enables/Disables constraints in the estimation parameters.
        %  Note that the constraints must be set with 'setParamsConstraints' first.
        %  @param[in] enable_contraints: Flag that is true/false for enabling/disabling the constraints.
        function enableParamsContraints(this, enable_contraints)
           
            this.enable_constraints = enable_contraints;
            
        end
           
        
        %% Sets linear constraints in the estimation parameters.
        %  The constraints are such that D*theta <= d
        %  @param[in] A_c: Constraint matrix.
        %  @param[in] b_c: Constraints bounds.
        function setParamsConstraints(this, A_c, b_c)
            
            this.A_c = A_c;
            this.b_c = b_c;
            
        end

        
        %% Sets the covariance matrix of the process noise.
        %  Note that if the continuous time process noise is R_c the discrete
        %  one is 'R = Q_c/Ts' where Ts is the sampling period.
        %  @param[in] Q: Process noise covariance matrix.
        function setProcessNoiseCov(this, Qn)
            
            this.Qn = Qn;
            
        end
        
        
        %% Sets the covariance matrix of the measurement noise.
        %% Note that if the continuous time process noise is Q_c the discrete
        %% one is 'Q = Q_c*Ts' where Ts is the sampling period.
        %  @param[in] R: Measurement noise covariance matrix.
        function setMeasureNoiseCov(this, Rn)
            
            this.Rn = Rn;
            
        end
            
        
        %% Sets the step for computing the partial derivatives of the state
        %% transition and/or measurement function w.r.t. the estimation parameters.
        %  @param[in] dtheta: Scalar or vector of parameters step size.
        function setPartDerivStep(this, dtheta)
            
            if (isscalar(dtheta))
                this.dtheta = ones(length(this.theta),1)*dtheta;
            else
                this.dtheta = dtheta;
            end

        end
        
        
        %% Performs the EKF prediction (time update).
        %  @params[in] cookie: Void pointer to additional arguments needed by the 
        %                      state transition function and its Jacobian (default = []).
        function predict(this, cookie)
            
            if (nargin < 2), cookie=[]; end
                    
            if (~isempty(cookie))
                this.F_k = this.stateTransFunJacob(this.theta, cookie);             
                this.theta = this.stateTransFun(this.theta, cookie);
            else
                this.F_k = this.stateTransFunJacob(this.theta);             
                this.theta = this.stateTransFun(this.theta);
            end
 
            % FP = this.P;
            % FP(1:3,:) = this.F_k(1:3,:)*this.P;
            % FP(4:6,:) = FP(4:6,:)+ this.P(1:3,:)*this.Ts;
            % this.P = this.a_p^2*FP*this.F_k' + this.Q;
            this.P = this.a_p^2*this.F_k*this.P*this.F_k' + this.Qn;

        end
        
        
        %% Performs the EKF correction (measurement update).
        %  @param[in] z: Groundtruth measurement.
        %  @params[in] cookie: Void pointer to additional arguments needed by the 
        %                      measurement function and its Jacobian (default = []).
        function correct(this, z, cookie)

            if (nargin < 3), cookie=[]; end
            
            % =====  Retrive the measurement function Jacobian  ===== 
            this.H_k = this.msrFunJacob(this.theta, cookie);
            z_hat = this.msrFun(this.theta, cookie);

            % =====  Correction estimates ===== 
            Kg = this.P*this.H_k'/(this.H_k*this.P*this.H_k' + this.Rn);
            % Kg = this.P*this.H_k'/(this.P(1:6,1:6) + this.Rn);
            this.theta = this.theta + Kg * (z - z_hat);
            
            % =====  Apply projection if enabled  ===== 
            proj_flag = false;
            D = []; % active contraints
            d = [];
            if ( this.enable_constraints && ~isempty(this.b_c) )
                ind = find(this.A_c*this.theta > this.b_c);
                if (~isempty(ind))
                    proj_flag = true;
                    D = this.A_c(ind,:);
                    d = this.b_c(ind);
                end     
            end
            
            N_params = length(this.theta);
            I = eye(N_params, N_params);
            
            if (proj_flag)
                % Kg = ( I - this.P*D'/(D*this.P*D')*D ) * Kg;
                % this.theta = this.theta - this.P*D'/(D*this.P*D')*(D*this.theta-d); 
                Kg = ( I - D'/(D*D')*D ) * Kg;
                this.theta = this.theta - D'/(D*D')*(D*this.theta-d); 
            end

            % =====  Calculate new covariance  =====
%             KC = Kg()
            this.P = (I - Kg*this.H_k) * this.P * (I - Kg*this.H_k)' + Kg*this.Rn*Kg';
            this.K = Kg;

        end
        
        
        %% Performs the EKF correction (measurement update).
        function theta_next = stateTransFun(this, theta, cookie)
            
            vRot = theta(1:3);
            eQ = theta(4:6);
            eg = theta(7:9);
            tau = theta(10);
            
            Q = quatExp(eQ);
            Qg = quatExp(eg);
            
            x = cookie.t/tau;
            tau0 = this.dmp.getTau();
            this.dmp.setTau(tau);
            dvRot = this.dmp.calcRotAccel(x, Q, vRot, Qg);
            deQ = this.rotVel2deo(vRot, Q);
            this.dmp.setTau(tau0);
            
            theta_next = zeros(10,1);
            theta_next(1:3) = vRot + dvRot*this.Ts;
            theta_next(4:6) = eQ + deQ*this.Ts;
            theta_next(7:10) = theta(7:10);
            
        end
        
        
        %% Performs the EKF correction (measurement update).
        function z = msrFun(this, theta, cookie)
            
            if (nargin < 3), cookie=[]; end
            
            z = theta(1:6);
            
        end
        
        
        %% Computes numerically the state transition function's Jacobian.
        %  @param[in] theta: Parameters around which the Jacobian is computed.
        %  @params[in] cookie: Void pointer to additional arguments needed by the 
        %                      state transition function's Jacobian (default = []).
        %  @return F_k: The state transition function's Jacobian.
        function F_k = stateTransFunJacob(this, theta, cookie)
 
            t = cookie.t;
            
            F_k = zeros(10,10);
            F_k(1:3,:) = this.calcF1Jacob(theta, t);
            F_k(4:6,:) = this.calcF2Jacob(theta);
            
            F_k = eye(10,10) + F_k*this.Ts;
            
        end
        
        function F1 = calcF1Jacob(this, theta, t)
           
            vRot = theta(1:3);
            eQ = theta(4:6);
            eg = theta(7:9);
            tau = theta(10);
            
            Q = quatExp(eQ);
            Qg = quatExp(eg);
            
            x = t/tau;
            
            tau0 = this.dmp.getTau();
            
            this.dmp.setTau(tau);
            
            F1 = zeros(3,10);
            
            dvRot_step = 1e-3;
            for i=1:3
                dvRot = zeros(3,1);
                dvRot(i) = dvRot_step;
                dvRot2 = this.dmp.calcRotAccel(x, Q, vRot + dvRot, Qg);
                dvRot1 = this.dmp.calcRotAccel(x, Q, vRot - dvRot, Qg);
                F1(:,i) = (dvRot2 - dvRot1) / (2*dvRot_step);
            end
            
            deQ_step = 1e-3;
            for i=1:3
                deQ = zeros(3,1);
                deQ(i) = deQ_step;
                dvRot2 = this.dmp.calcRotAccel(x, quatExp(eQ+deQ), vRot, Qg);
                dvRot1 = this.dmp.calcRotAccel(x, quatExp(eQ-deQ), vRot, Qg);
                F1(:,3+i) = (dvRot2 - dvRot1) / (2*deQ_step);
            end
            
            deg_step = 1e-3;
            for i=1:3
                deQ = zeros(3,1);
                deQ(i) = deg_step;
                dvRot2 = this.dmp.calcRotAccel(x, Q, vRot, quatExp(eg+deQ));
                dvRot1 = this.dmp.calcRotAccel(x, Q, vRot, quatExp(eg-deQ));
                F1(:,6+i) = (dvRot2 - dvRot1) / (2*deg_step);
            end
            
            dtau = 1e-3;
            this.dmp.setTau(tau+dtau);
            dvRot2 = this.dmp.calcRotAccel(t/(tau+dtau), Q, vRot, Qg);
            this.dmp.setTau(tau-dtau);
            dvRot1 = this.dmp.calcRotAccel(t/(tau-dtau), Q, vRot, Qg);
            F1(:,10) = (dvRot2 - dvRot1) / (2*dtau);
            
            this.dmp.setTau(tau0);
            
        end
        
        function F2 = calcF2Jacob(this, theta)
           
            vRot = theta(1:3);
            eQ = theta(4:6);
            Q = quatExp(eQ);
            
            F2 = zeros(3,10);
            
            dvRot_step = 1e-3;
            for i=1:3
                dvRot = zeros(3,1);
                dvRot(i) = dvRot_step;
                deo2 = this.rotVel2deo(vRot + dvRot, Q);
                deo1 = this.rotVel2deo(vRot - dvRot, Q);
                F2(:,i) = (deo2 - deo1) / (2*dvRot_step);
            end
            
            % J_eQ_Q = DMP_eo.jacobDeoDquat(Q);
            % J_Q_eQ = DMP_eo.jacobDquatDeo(Q);
            % F2(:,1:3) = 0.5*J_eQ_Q*this.quat2mat(vRot)*J_Q_eQ;
            
            deQ_step = 1e-3;
            for i=1:3
                deQ = zeros(3,1);
                deQ(i) = deQ_step;
                deo2 = this.rotVel2deo(vRot, quatExp(eQ+deQ));
                deo1 = this.rotVel2deo(vRot, quatExp(eQ-deQ));
                F2(:,3+i) = (deo2 - deo1) / (2*deQ_step);
            end
            
        end
        
        function deo = rotVel2deo(this, rotVel, Q)
            
            J_deo_dQ = DMP_eo.jacobDeoDquat(Q);
            deo = 0.5*J_deo_dQ * quatProd([0; rotVel], Q);
            
        end
        
        
        %% Computes numerically the measurement function's Jacobian.
        %  @param[in] theta: Parameters around which the Jacobian is computed.
        %  @params[in] cookie: Void pointer to additional arguments needed by the  
        %                      measurement function's Jacobian (default = []).
        %  @return H_k: The measurement function's Jacobian.
        function H_k = msrFunJacob(this, theta, cookie)
            
            if (nargin < 3), cookie=[]; end
            
            H_k = [eye(6,6) zeros(6,4)];
            
        end

        
    end
    
    methods (Static, Access = private)
       
        function ssMat = vec2ssMat(p)

            ssMat(1,1) = 0;
            ssMat(2,2) = 0;
            ssMat(3,3) = 0;
            ssMat(1,2) = -p(3);
            ssMat(2,1) = p(3);
            ssMat(1,3) = p(2);
            ssMat(3,1) = -p(2);
            ssMat(3,2) = p(1);
            ssMat(2,3) = -p(1);

        end
        
        function qmat = quat2mat(Q)

            qmat = zeros(4,4);

            w = Q(1);
            v = Q(2:4);

            qmat(:,1) = Q;
            qmat(1,2:4) = -v;
            qmat(2:4,2:4) = w*eye(3,3) + this.vec2ssMat(v);

        end
        
    end

    properties
        
        Ts % sampling time
        dmp
        Az
        Bz
        
        F_k % state transition function Jacobian
        H_k % measurement function Jacobian
        K % Kalman gain
        Qn % process noise covariance
        Rn % measurement noise covariance
        
        a_p % fading memory coefficient
        theta % parameters estimate
        P % parameters estimation error covariance
        
        % Apply projection so that:
        % A_c * theta <= b_c
        enable_constraints
        A_c % constraint matrix
        b_c % constraints bounds
        
        apply_cov_sat
        theta_sigma_min
        theta_sigma_max
        
        stateTransFun_ptr % state transition function pointer
        msrFun_ptr % measurement function pointer
        
        stateTransFunJacob_ptr % state transition function Jacobian pointer
        msrFunJacob_ptr % measurement function Jacobian pointer
               
        dtheta % Parameters step size for numerical approximation of transition and measurement function Jacobian
    end
    
end
