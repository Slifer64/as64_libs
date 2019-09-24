%% Inequality constraints abstract class
%

classdef IneqConstr < handle
       
    methods (Access = public)
        %% Constructor.
        function this = IneqConstr()

        end
        
    end
    
    methods (Abstract, Access = public)
        
        %% Returns the value of the inequality constraints f_i(x).
        %  @param[in] x: State vector.
        %  @return: The value of the inequality constraints f_i(x).
        yi = fi(this, x)
            
        %% Returns the log barrier value, gradient and hessian.
        %  @param[out] phi: log barrier function value. (optional)
        %  @param[out] grad_phi: log barrier function gradient. (optional)
        %  @param[out] hess_phi: log barrier function hessian. (optional)
        [phi, grad_phi, hess_phi] = logBarFun(this, x)
        
        %% Returns the log barrier value, gradient and hessian for the phase1 problem.
        %  @param[out] phi: log barrier function value. (optional)
        %  @param[out] grad_phi: log barrier function gradient. (optional)
        %  @param[out] hess_phi: log barrier function hessian. (optional)
        [phi, grad_phi, hess_phi] = ph1LogBarFun(this, x)

    end
    
end