%% System model
%

classdef SysModel < handle


    methods
        
        %% Constructor.
        % @param[in] dt: sampling time.
        function this = SysModel(dt)
            
            this.dt = dt;
            
        end
        
        function setExtParams(this, t)
            
            this.t = t;
            
        end
        
    end
    
    %% ====  Abstract Methods  =====
    methods (Abstract)
            

        %% State transition function.
        % @param[in] x: state at time k.
        % @param[in] cookie: optional arguments.
        % @param[out] x_next: state at time k+1.
        %
        x_next = stateTransFun(this, x, cookie)
        
        
        %% Measurement function.
        % @param[in] x: state at time k.
        % @param[in] cookie: optional arguments.
        % @param[out] y: measurements at time k.
        %
        y = msrFun(this, x, cookie)

        
        %% Measurement function Jacobian.
        % @param[in] x: state at time k
        % @param[in] cookie: optional arguments.
        % @param[out] H: measurement function Jacobian.
        %
        H = msrFunJacob(this, x, cookie)

        
        %% State transition function Jacobian.
        % @param[in] x: state at time k
        % @param[in] cookie: optional arguments.
        % @param[out] F: state transition function Jacobian.
        %
        F = stateTransFunJacob(this, x, cookie)

 
    end

    properties (Access = protected)
       
        dt % sampling time
        t  % current timestamp
        
    end
    
end
