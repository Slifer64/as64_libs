%% DMP_eo2 class
%  For encoding Cartesian Orientation.
%


classdef DMP_eo2 < matlab.mixin.Copyable
    
    methods  (Access = public)
        %% DMP_eo2 constructor.
        %  @param[in] N_kernels: 3x3 matrix where each row contains the kernels for position,
        %                       velocity and acceleration and each row for x, y and z coordinate.
        %                       If 'N_kernels' is a column vector then every coordinate will have the
        %                       same number of kernels.
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] shape_attr_gating_ptr: Pointer to a DMP gating function object.
        %
        function this = DMP_eo2(dmp_type, N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr)

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
                error('[DMP_eo2::DMP_eo2]: Unsupported DMP type!');
            end

        end
        
        
        %% Trains the DMP_eo2.
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
            eo_data = zeros(3, n_data);
            deo_data = zeros(3, n_data);
            ddeo_data = zeros(3, n_data);

            Qg = Quat_data(:,end);

            for j=1:n_data
               Qe = DMP_eo2.quatProd(Qg, quatInv(Quat_data(:,j)) );
               eo_data(:,j) = quatLog(Qe);
               deo_data(:,j) = DMP_eo2.rotVel2deo(rotVel_data(:,j), Qe);
               ddeo_data(:,j) = DMP_eo2.rotAccel2ddeo(rotAccel_data(:,j), rotVel_data(:,j), Qe);
            end

            if (nargout > 0)
                train_err = zeros(3,1);
                for i=1:3
                    train_err(i) = this.dmp{i}.train(train_method, Time, eo_data(i,:), deo_data(i,:), ddeo_data(i,:));
                end
            else
                for i=1:3
                    this.dmp{i}.train(train_method, Time, eo_data(i,:), deo_data(i,:), ddeo_data(i,:));
                end
            end

        end
      

            
        function setQ0(this, Q0)
            
            this.Q0 = Q0; 
            Y0 = DMP_eo2.quat2eo(this.Q0, this.Qg);
            for i=1:length(this.dmp), this.dmp{i}.setY0(Y0(i)); end
            
        end
        
        
        function setQg(this, Qg)
            
            this.Qg = Qg; 
            Y0 = DMP_eo2.quat2eo(this.Q0, this.Qg);
            for i=1:length(this.dmp), this.dmp{i}.setY0(Y0(i)); end
            
        end
        
        
        function y = getY(this, Q), y = DMP_eo2.quat2eo(Q, this.Qg); end

        
        function z = getZ(this, rotVel, Q)

            Qe = DMP_eo2.quatError(Q, this.Qg);
            deo = DMP_eo2.rotVel2deo(rotVel, Qe);
            dy = deo;

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
        function update(this, x, Y, Z, Yc, Zc)

            if (nargin < 6), Yc = zeros(3,1); end
            if (nargin < 7), Zc = zeros(3,1); end

            if (isscalar(Yc)), Yc = ones(3,1)*Yc; end
            if (isscalar(Zc)), Zc = ones(3,1)*Zc; end

            for i=1:3, this.dmp{i}.update(x, Y(i), Z(i), 0, Yc(i), Zc(i)); end

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

            Qe = DMP_eo2.quatError(Q, this.Qg);
            deo = this.getYdot(); 
            rotVel = DMP_eo2.deo2rotVel(deo, Qe);

        end
        
        function rotAccel = getRotAccel(this, Q, tau_dot, yc_dot)
            
            if (nargin < 3), tau_dot=0; end
            if (nargin < 4), yc_dot=0; end

            Qe = DMP_eo2.quatError(Q, this.Qg);
            ddeo = this.getYddot(tau_dot, yc_dot);
            rotVel = this.getRotVel(Q);
            rotAccel = DMP_eo2.ddeo2rotAccel(ddeo, rotVel, Qe);

        end
        
        function rotAccel = calcRotAccel(this, x, Q, rotVel, Qg, tau_dot, yc, zc, yc_dot)

            if (nargin < 6), tau_dot = 0; end
            if (nargin < 7), yc = zeros(3,1); end
            if (nargin < 8), zc = zeros(3,1); end
            if (nargin < 9), yc_dot = zeros(3,1); end

            a_z = [this.dmp{1}.a_z; this.dmp{2}.a_z; this.dmp{3}.a_z];
            b_z = [this.dmp{1}.b_z; this.dmp{2}.b_z; this.dmp{3}.b_z];
            tau = this.getTau();


            Qg_prev = this.Qg;
            this.setQg(Qg);

            Qe = DMP_eo2.quatError(Q, Qg);
            eo = DMP_eo2.quat2eo(Q, Qg);
            invQe = quatInv(Qe);

            rotVelQ = [0; rotVel];
            QeRotVel = quatProd(Qe,rotVelQ);

            J_deo_dQ = DMP_eo2.jacobDeoDquat(Qe);
            J_dQ_deo = DMP_eo2.jacobDquatDeo(Qe);
            dJ_dQ_deo = DMP_eo2.jacobDotDquatDeo(Qe, rotVel);

            fo = zeros(3,1);
            for i=1:3, fo(i) = this.dmp{i}.shapeAttractor(x, 0); end

            deo = DMP_eo2.rotVel2deo(rotVel, Qe);
            ddeo = (-a_z.*b_z.*eo - tau*a_z.*deo + a_z.*yc + fo + tau*yc_dot - tau*tau_dot*deo + zc)/tau^2;

            rotAccel1 = quatProd(invQe, dJ_dQ_deo*J_deo_dQ*QeRotVel);
            % rotAccel2 = 2*quatProd(invQe, J_dQ_deo* (a_z.*b_z.*eo - 0.5*tau*a_z.*(J_deo_dQ*QeRotVel) - fo) ) / tau^2;
            rotAccel2 = 2*quatProd(invQe, J_dQ_deo*-ddeo);   
            rotAccel = rotAccel1 + rotAccel2;  
            rotAccel = rotAccel(2:4);

            this.setQg(Qg_prev);

            % norm(rotAccel-rotAccel_2)

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
        function Qe = quatError(Q, Q0), Qe = quatProd(Q, quatInv(Q0)); end

        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        function eo = quat2eo(Q, Q0), eo = quatLog(DMP_eo2.quatError(Q,Q0)); end
        
        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        function Q = eo2quat(eo, Q0), Q = quatProd( quatExp(eo), Q0); end
        
        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        function deo = rotVel2deo(rotVel, Qe)
            
            J_deo_dQ = DMP_eo2.jacobDeoDquat(Qe);
            deo = -0.5*J_deo_dQ * quatProd(Qe, [0; rotVel]);

        end
        
        
        %% Given a quaternion, a target quaternion and the quaternion error velocity w.r.t the
        %% target, returns the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] deo: Quaternion error velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The rotational velocity of the quaternion.
        %
        function rotVel = deo2rotVel(deo, Qe)
            
            J_dQ_deo = DMP_eo2.jacobDquatDeo(Qe);
            rotVel = -2 * quatProd( quatInv(Qe), J_dQ_deo*deo );
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
        function ddeo = rotAccel2ddeo(rotAccel, rotVel, Qe)
            
            rotVelQ = [0; rotVel];
            rotAccelQ = [0; rotAccel];

            J = DMP_eo2.jacobDeoDquat(Qe);
            dJ = DMP_eo2.jacobDotDeoDquat(Qe, rotVel);

            ddeo = -0.5 * (dJ * quatProd(Qe, rotVelQ) + J * quatProd( Qe, rotAccelQ-0.5*quatProd(rotVelQ,rotVelQ) ) );

        end
        
        
        %% Given a quaternion, its rotational velocity and acceleration and a target quaternion,
        %% returns the second derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotAccel: The quaternion's rotational acceleration.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The second derivative of the orientation error.
        %
        function rotAccel = ddeo2rotAccel(ddeo, rotVel, Qe)

            deo = DMP_eo2.rotVel2deo(rotVel, Qe);
            invQe = quatInv(Qe);
            J = DMP_eo2.jacobDquatDeo(Qe);
            dJ = DMP_eo2.jacobDotDquatDeo(Qe, rotVel);

            % rotVelQ = [0; rotVel];

            % rotAccel = -quatProd(quatProd(rotVelQ, invQe), J*deo) - 2*quatProd(invQe, dJ*deo) - 2*quatProd(invQe, J*ddeo);
            % rotAccel2 = 0.5*quatProd(rotVelQ, rotVelQ) - 2*quatProd(invQe, dJ_dQ_deo*deo) - 2*quatProd(invQe, J_dQ_deo*ddeo);
            rotAccel = - 2 * (quatProd(invQe, dJ*deo) + quatProd(invQe, J*ddeo));
            rotAccel = rotAccel(2:4);

        end
        
        
        %% Returns the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @return: Jacobian from derivative of orientation error to quaternion derivative.
        %
        function J_dQ_deo = jacobDquatDeo(Qe)

            if (abs(Qe(1)-1) <= DMP_eo2.zero_tol)
                J_dQ_deo = [zeros(1, 3); eye(3,3)];
                return;
            end

            w = Qe(1);
            v = Qe(2:4);
            norm_v = norm(v);
            eta = v / norm_v;
            s_th = norm_v;
            c_th = w;
            th = atan2(s_th, c_th);
            Eta = eta*eta';

            J_dQ_deo = zeros(4,3);
            J_dQ_deo(1,:) = -0.5 * s_th * eta';
            J_dQ_deo(2:4,:) = 0.5 * ( (eye(3,3) - Eta)*s_th/th + c_th*Eta );

        end
        
        
        %% Returns the Jacobian from the quaternion derivative to the derivative of the orientation error.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @return: Jacobian from quaternion derivative to derivative of orientation error.
        %
        function J_deo_dQ = jacobDeoDquat(Qe)
            
            if (abs(Qe(1)-1) <= DMP_eo2.zero_tol)
                J_deo_dQ = [zeros(3,1) eye(3,3)];
                return;
            end

            w = Qe(1);
            v = Qe(2:4);
            norm_v = norm(v);
            eta = v / norm_v;
            s_th = norm_v;
            c_th = w;
            th = atan2(s_th, c_th);

            J_deo_dQ = zeros(3,4);
            J_deo_dQ(:,1) = 2*eta*(th*c_th - s_th)/s_th^2;
            J_deo_dQ(:,2:4) = 2*eye(3,3)*th/s_th;

        end
       
        
        %% Returns the time derivative of the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @param[in] deo: The derivative of the orientation error.
        %  @return: The derivative of the Jacobian from derivative of orientation error to quaternion derivative.
        %
        function dJ_deo_dQ = jacobDotDeoDquat(Qe, rotVel)

            deo = DMP_eo2.rotVel2deo(rotVel, Qe);

            if (abs(Qe(1)-1) <= DMP_eo2.zero_tol)
                dJ_deo_dQ = [-deo/3 zeros(3,3)];
                return;
            end

            w = Qe(1);
            v = Qe(2:4);
            norm_v = norm(v);
            eta = v / norm_v;
            s_th = norm_v;
            c_th = w;
            th = atan2(s_th, c_th);
            Eta = eta*eta';
            temp = (th*c_th-s_th)/s_th^2;

            dJ_deo_dQ = zeros(3,4);
            dJ_deo_dQ(:,1) = ((-th/s_th - 2*c_th*temp/s_th)*Eta + temp*(eye(3,3)-Eta)/th)*deo;
            dJ_deo_dQ(:,2:4) = -temp*dot(eta,deo)*eye(3,3);

        %             dQe = -0.5*quatProd(Qe, [0; rotVel]);
        %             dJ = getQToLogJacobianAcceleration( Qe, dQe );
        %             dJ_deo_dQ2 = 2*dJ;
        %             dJ_deo_dQ-dJ_deo_dQ2;
        end
        
        
        %% Returns the time derivative of the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @param[in] deo: The derivative of the orientation error.
        %  @return: The derivative of the Jacobian from derivative of orientation error to quaternion derivative.
        %
        function dJ_dQ_deo = jacobDotDquatDeo(Qe, rotVel)
            
            deo = DMP_eo2.rotVel2deo(rotVel, Qe);

            if (abs(Qe(1)-1) <= DMP_eo2.zero_tol)
                dJ_dQ_deo = [-deo'/4; zeros(3,3)];
                return;
            end

            w = Qe(1);
            v = Qe(2:4);
            norm_v = norm(v);
            eta = v / norm_v;
            s_th = norm_v;
            c_th = w;
            th = atan2(s_th, c_th);
            Eta = eta*eta';
            I_eta = eye(3,3) - Eta;
            temp = ((th*c_th-s_th)/th^2);

            dJ_dQ_deo = zeros(4,3);
            dJ_dQ_deo(1,:) = -0.25 * deo' * (c_th*Eta + (s_th/th)*I_eta);
            dJ_dQ_deo(2:4,:) = 0.25*dot(eta,deo)*( temp*I_eta - s_th*Eta ) + 0.25*temp*( eta*(deo'*I_eta) + (I_eta*deo)*eta' );

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
        Qg
        
        dZ % second derivative of the orientation error
        dY % first derivative of the orientation error
        dx % pahse variable derivative
        
    end
    
end
