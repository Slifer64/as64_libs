%% Van der Pol oscilator.
%

classdef VDP < SysModel


    %% ====  Methods  =====
    methods
            
        %% Constructor.
        function this = VDP(dt)
        
            this@SysModel(dt);

        end

        
        %% State transition function.
        function x_next = stateTransFun(this, x, cookie) 

            dxdt = [x(2); (1-x(1)^2)*x(2)-x(1)];
            
            % Euler integration of continuous-time dynamics x_dot=f(x) with sample time dt
            x_next = x + dxdt*this.dt;

        end
        
        
        %% Measurement function.
        function y = msrFun(this, x, cookie)
        
%             y = x(1);
            y = x(1)^2;
%             y = x(1)*log(abs(x(1)));
%             y = x(2);
%             y = 0.5*x(1)^3 + 0.2*x(1)^2 + 0.8*x(1) + 0.5;

        end

        
        %% Measurement function Jacobian.
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
    
end
