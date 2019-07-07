%% Falling object model.
%

classdef FallingObject < SysModel


    %% ====  Methods  =====
    methods
            
        %% Constructor.
        function this = FallingObject(dt, x_n)
        
            this@SysModel(dt);
            
            if (nargin < 2), x_n = ones(3,1); end
            
            this.x_n = x_n;
            this.po = 2;     % lb-sec^2 / ft^4
            this.g = 32.2;   % ft / sec^2
            this.k = 20000;  % ft
            this.M = 100000; % ft
            this.a = 100000; % ft

        end

        %% State transition function.
        function x_next = stateTransFun(this, x, cookie) 

            x = x .* this.x_n;
            
            dx = zeros(3,1);
            dx(1) = x(2);
            dx(2) = 0.5*this.po*exp(-x(1)/this.k)*x(2)^2*x(3) - this.g;
            dx(3) = 0;
            
            x_next = x + dx*this.dt;
            
            x_next = x_next ./ this.x_n;

        end
        
        
        %% Measurement function.
        function y = msrFun(this, x, cookie)

            y = sqrt(this.M^2 + (x(1)-this.a)^2 );

        end

        
        %% Measurement function Jacobian.
        function H = msrFunJacob(this, x, cookie)
        
            H = zeros(1,3);
            H(1) = (x(1) - this.a) / sqrt(this.M^2 + (x(1)-this.a)^2 );

        end

        
        %% State transition function Jacobian.
        function F = stateTransFunJacob(this, x, cookie)

            x = x .* this.x_n;
            
            F = zeros(3,3);
            
            F(1,2) = 1 * this.x_n(2);
            F(2,1) = -0.5*this.po*exp(-x(1)/this.k)*x(2)^2*x(3)/this.k * this.x_n(1);
            F(2,2) = this.po*exp(-x(1)/this.k)*x(2)*x(3) * this.x_n(2);
            F(2,3) = 0.5*this.po*exp(-x(1)/this.k)*x(2)^2 * this.x_n(3);
            
            

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

        x_n % normalizatiom value for each state
        po
        g
        k
        M
        a
        
    end
    
end
