%% Objective function class
%

classdef Objective
       
    methods (Access = public)
        %% Objective function class constructor.
        function this = Objective()

        end
        
    end
    
    methods (Abstract, Access = public)
        %% Calculates the value of the objective function.
        %  @param[in] x: Point at which to evaluate the objective.
        %  @param[out] J: Objective function value at the point 'x'.
        J = fun(this, x)
        
        %% Calculates the gradient of the objective function.
        %  @param[in] x: Point at which to evaluate the gradient.
        %  @param[out] dJ: Gradient of the objective function value at the point 'x'.
        dJ = gradFun(this, x)   
        
    end
    

end

