%% Mackey-Glass chaotic time series with time delay = 30
%

classdef MackeyGlassTS < SysModel


    %% ====  Methods  =====
    methods
            
        %% Constructor.
        function this = MackeyGlassTS(dt)
        
            this@SysModel(dt);
            
            % deltat = 5;
            % a = 0.15;
            % b = 0.05;
            % tau = 30;
            % x0 = 1.2;
            % MackeyGlassTS.trainNN(a, b, tau, x0, deltat);
            % return
            
        end
        
        
        %% State transition function.
        function x_next = stateTransFun(this, x, cookie) 

            xin = x';
            xout  = MackeyGlassTS.mackeyGlassNN(xin);
            x_next = [x(2:end); xout(1)];

        end
        
        
        %% Measurement function.
        function y = msrFun(this, x, cookie)
        
            y = x(end);

        end

        
        %% Measurement function Jacobian.
        function H = msrFunJacob(this, x, cookie)
        
            error('[MackeyGlassTS::msrFunJacob]: Unimplemented function!');

        end

        
        %% State transition function Jacobian.
        function F = stateTransFunJacob(this, x, cookie)

            error('[MackeyGlassTS::stateTransFunJacob]: Unimplemented function!');

        end
 
        
    end
    
    methods (Static, Access = private)
        
        Y = mackeyGlassNN(X)
        
        trainNN(a, b, tau, x0, deltat)
 
    end
    
    properties (Access = private)
        
    end
    
end
