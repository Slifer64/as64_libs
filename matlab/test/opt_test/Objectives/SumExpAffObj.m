%% Sum of Exponetials of affine functions class
%

classdef SumExpAffObj < Objective
       
    methods (Access = public)
        %% Sum of Exponetials of affine functions class constructor.
        %  @param[in] n: The problem dimension. 
        %  @param[in] P: The positive definite matrix in x'*P*x term.
        %  @param[in] q: The vector in q'*x term.
        %  @param[in] r: The constant term.
        function this = SumExpAffObj()

            this.a1 = [1; 3];
            this.b1 = -0.1;
            this.a2 = [1; -3];
            this.b2 = -0.1;
            this.a3 = [-1; 0];
            this.b3 = -0.1;

        end

        %% Calculates the value of the objective function.
        %  @param[in] x: Point at which to evaluate the objective.
        %  @param[out] J: Objective function value at the point 'x'.
        function J = fun(this, x)
            
            J = exp(this.a1'*x + this.b1) + exp(this.a2'*x + this.b2) + exp(this.a3'*x + this.b3);
            
        end
        
        %% Calculates the gradient of the objective function.
        %  @param[in] x: Point at which to evaluate the gradient.
        %  @param[out] dJ: Gradient of the objective function value at the point 'x'.
        function dJ = gradFun(this, x)
            
            dJ = exp(this.a1'*x + this.b1)*this.a1 + exp(this.a2'*x + this.b2)*this.a2 + exp(this.a3'*x + this.b3)*this.a3;
            
        end
        
    end
    
    properties (Access = private)
       
        a1
        b1
        a2
        b2
        a3
        b3
        
    end

end