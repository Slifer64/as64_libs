%% Tracker of quaternion trajectory with 5th order polynomial trajectory in the quaternion error space.
%

classdef Quat5thPolyTrajTracker < SysModel


    %% ====  Methods  =====
    methods
            
        %% Constructor.
        function this = Quat5thPolyTrajTracker(Q0, Qg, T, dt)
        
            this@SysModel(dt);
            
            this.Qd_prev = Q0;
            this.eo_0 = quatLog( quatProd(Qg, quatInv(Q0) ) );
            this.eo_g = [0 0 0]';
            this.T = T;
            this.dt = dt;
            
            this.D = [2; 2; 2];
            this.K = [10; 10; 10];

        end

        %% State transition function.
        function x_next = stateTransFun(this, x, cookie) 

            Q = x(1:4);
            vRot = x(5:7);
            
            Qd = this.getTrajectory(this.t);
            
            dvRot = -this.D.*vRot - this.K.*quatLog( quatProd(Q, quatInv(Qd)) );

            % Euler integration of continuous-time dynamics x_dot=f(x) with sample time dt
            Q = quatProd(quatExp(vRot*this.dt), Q);
            vRot = vRot + dvRot*this.dt;
            
            x_next = [Q; vRot];

        end
        
        
        %% Measurement function.
        function y = msrFun(this, x, cookie)
        
            Qd = this.getTrajectory(this.t);
            
            Q = x(1:4);
            vRot = x(5:7);
            
            y = [Q; vRot];
            
            % y = quatProd( quatExp(vRot/this.dt), Q); % Q_{k+1}
            % y = -this.D.*vRot - this.K.*quatLog( quatProd(Q, quatInv(Qd)) );


        end

        
        %% Measurement function Jacobian.
        function H = msrFunJacob(this, x, cookie)
        
            error('[Quat5thPolyTrajTracker::msrFunJacob]: Unimplemented function!');

        end

        
        %% State transition function Jacobian.
        function F = stateTransFunJacob(this, x, cookie)

            error('[Quat5thPolyTrajTracker::stateTransFunJacob]: Unimplemented function!');

        end

        
        %% Get reference trajectory.
        function Qd = getTrajectory(this, t)
    
            tn = t/this.T;   
            eo_d = this.eo_0 + (this.eo_g - this.eo_0)*( 10*tn^3 - 15*tn^4 + 6*tn^5 );    
            Qd = quatExp(eo_d);
            
            if ( dot(Qd,this.Qd_prev) < 0), Qd = -Qd; end
            this.Qd_prev = Qd;
            
        end
        
    end

    properties (Access = private)

        eo_0
        eo_g
        T
        Qd_prev
        
        D
        K
        
    end
    
end
