%% EKF class
% Implementation of the discrete Extended Kalman Filter algorithm.
% - The Jacobian of the state transition and measurement functions can either
% be provided or approximated numerically.
% - A fading memory coefficient can be defined.
% - Enforcement of linear constraints can be imposed on the estimated parameteres.
%


classdef DMPpEKFa < handle
    
    methods
        %% Extended Kalman Filter constructor
        %  @param[in] N_params: Number of states to estimate.
        %  @param[in] N_out: Number of outputs.
        %  @param[in] stateTransFun_ptr: Pointer to state transition function. It should accept a vector (the parameters) and 
        %                                  a void pointer (cookie) to pass optinally extra arguments to the state transition function.
        %  @param[in] msrFun_ptr: Pointer to measurement function. It should accept a vector (the parameters) and 
        %                                  a void pointer (cookie) to pass optinally extra arguments to the measurement function.
        function this = DMPpEKFa(dmp, Ts)
            
            this.Ts = Ts;
            this.dmp = dmp;
            this.Az = diag([dmp{1}.a_z, dmp{2}.a_z, dmp{3}.a_z]);
            this.Bz = diag([dmp{1}.b_z, dmp{2}.b_z, dmp{3}.b_z]);
            
            N_params = 10;
            
            this.theta = zeros(N_params);
            this.P = zeros(N_params, N_params);
            
            this.setProcessNoiseCov(eye(N_params,N_params)*1e-5);
            this.setMeasureNoiseCov(eye(N_out,N_out)*0.05);
            
            this.setFadingMemoryCoeff(1.0);
            
            this.enableParamsContraints(false);
            this.setParamsConstraints([],[]);
            
            this.setPartDerivStep(0.001);
            
            this.stateTransFun_ptr = @this.stateTransFun;
            this.msrFun_ptr = @this.msrFun;
            
            this.msrFunJacob_ptr = @this.msrFunJacob;
            this.stateTransFunJacob_ptr = @this.stateTransFunJacob;

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
        function setProcessNoiseCov(this, Q)
            
            this.Q = Q;
            
        end
        
        
        %% Sets the covariance matrix of the measurement noise.
        %% Note that if the continuous time process noise is Q_c the discrete
        %% one is 'Q = Q_c*Ts' where Ts is the sampling period.
        %  @param[in] R: Measurement noise covariance matrix.
        function setMeasureNoiseCov(this, R)
            
            this.R = R;
            
        end
            
        
        %% Sets the state transition function Jacobian.
        %  @param[in] stateTransFunJacob_ptr: Pointer to the state transition function Jacobian. It should accept a vector (the parameters)
        %                                        and a void pointer (cookie) to pass extra arguments to the function.
        function setStateTransFunJacob(this, stateTransFunJacob_ptr)
            
            this.stateTransFunJacob_ptr = stateTransFunJacob_ptr;
            
        end
        
        
        %% Sets the measurement function Jacobian.
        %  @param[in] msrFunJacob_ptr: Pointer to the measurement function Jacobian. It should accept a vector (the parameters)
        %                                        and a void pointer (cookie) to pass extra arguments to the function.
        function setMsrFunJacob(this, msrFunJacob_ptr)
            
            this.msrFunJacob_ptr = msrFunJacob_ptr;
            
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
                this.F_k = this.stateTransFunJacob_ptr(this.theta, cookie);             
                this.theta = this.stateTransFun_ptr(this.theta, cookie);
            else
                this.F_k = this.stateTransFunJacob_ptr(this.theta);             
                this.theta = this.stateTransFun_ptr(this.theta);
            end
 
            this.P = this.a_p^2*this.F_k*this.P*this.F_k' + this.Q;
            
%             FP = this.P;
%             FP(1:3,:) = this.F_k(1:3,:)*this.P;
%             FP(4:6,:) = FP(4:6,:)+ this.P(1:3,:)*this.Ts;
%             this.P = this.a_p^2*FP*this.F_k' + this.Q;
    
        end
        
        
        %% Performs the EKF correction (measurement update).
        %  @param[in] z: Groundtruth measurement.
        %  @params[in] cookie: Void pointer to additional arguments needed by the 
        %                      measurement function and its Jacobian (default = []).
        function correct(this, z, cookie)

            if (nargin < 3), cookie=[]; end
            
            % =====  Retrive the measurement function Jacobian  ===== 
            this.H_k = this.msrFunJacob_ptr(this.theta, cookie);
            z_hat = this.msrFun_ptr(this.theta, cookie);

            % =====  Correction estimates ===== 
%             CPC = this.P(1:6,1:6);
%             CPC == this.H_k*this.P*this.H_k'
            Kg = this.P*this.H_k'/(this.H_k*this.P*this.H_k' + this.R);
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
            this.P = (I - Kg*this.H_k) * this.P * (I - Kg*this.H_k)' + Kg*this.R*Kg';
            this.K = Kg;

        end
        
        
        %% Performs the EKF correction (measurement update).
        function theta_next = stateTransFun(this, theta, cookie)
            
            p_dot = theta(1:3);
            p = theta(4:6);
            pg = theta(7:9);
            tau = theta(10);
            
            x = cookie.t/tau;
            p_ddot = this.dmp.calcYddot(x, p, p_dot, pg);
 
            
            theta_next = zeros(10,1);
            theta_next(1:3) = p_ddot;
            theta_next(4:6) = p_dot * this.Ts;
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
            
            p_dot = theta(1:3);
            p = theta(4:6);
            pg = theta(7:9);
            tau = theta(10);
            x = cookie.t/tau;
            
            F_k = eye(10,10);
            F_k(4:6,1:3) = eye(3,3)*this.Ts;
            F_k(1:3,1:3) = eye(3,3) - this.Az*this.Ts/tau;
            F_k(1:3,4:6) = -this.Az*this.Bz*this.Ts/tau^2;
            F_k(1:3,7:10) = this.dmp.getAcellPartDev_g_tau(this, cookie.t, p, p_dot, cookie.p0, x, pg, tau)*this.Ts;
            
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

    properties
        
        Ts % sampling time
        dmp
        Az
        Bz
        
        F_k % state transition function Jacobian
        H_k % measurement function Jacobian
        K % Kalman gain
        Q % process noise covariance
        R % measurement noise covariance
        
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
