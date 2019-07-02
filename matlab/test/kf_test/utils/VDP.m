%% Van der Pol
%

classdef VDP < handle


    %% ====  Methods  =====
    methods
            
        %% Evaluate the van der Pol ODEs for mu = 1.
        function this = VDP(dt)
        
            this.dt = dt;

        end
        
        %% Evaluate the van der Pol ODEs for mu = 1.
        function dxdt = stateTransFunCont(this, x)
        
            dxdt = [x(2); (1-x(1)^2)*x(2)-x(1)];

        end

        
        %% State transition function.
        % @param[in] x: state at time k.
        % @param[out] x_next: state at time k+1.
        %
        function x_next = stateTransFun(this, x, cookie) 

            % Euler integration of continuous-time dynamics x'=f(x) with sample time dt
            x_next = x + this.stateTransFunCont(x)*this.dt;

        end
        
        
        %% Measurement function.
        % @param[in] xk: state at time k.
        % @param[out] yk: measurements at time k.
        %
        function y = msrFun(this, x, cookie)
        
%             y = x(1);
            y = x(1)*x(1);
%             y = x(1)*log(abs(x(1)));
%             y = x(2);
%             y = 0.5*x(1)^3 + 0.2*x(1)^2 + 0.8*x(1) + 0.5;

        end

        
        %% Measurement function Jacobian.
        % @param[in] xk: x[k], states at time k
        % @param[out] yk: y[k], measurements at time k
        %
        function H = msrFunJacob(this, x, cookie)
        
            H = zeros(1,2);
            H(1) = 0;
            H(2) = 1;

        end

        
        %% State transition function Jacobian.
        function F = stateTransFunJacob(this, x, cookie)

            F = zeros(2,2);
            F(1,1) = 1;
            F(1,2) = this.dt;
            F(2,1) = -this.dt*(1+2*(1-x(1)));
            F(2,2) = 1 + x(2).*(1-x(1)).^2*this.dt;

        end
 
    end
    
    properties (Access = private)
       
        dt
        
    end
    
end
