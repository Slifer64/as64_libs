%% Linear minus the sum of the log of affine inequalities class
%

classdef LinearMinusSumLogAffIneqObj < Objective
       
    methods (Access = public)
        %% Sum of Exponetials of affine functions class constructor.
        %  @param[in] n: The problem dimension. 
        %  @param[in] m: Number of affine inequalities.
        function this = LinearMinusSumLogAffIneqObj(n, m)

            this.c = rand(n,1);
            
            this.A = rand(m, n);
            
            x0 = rand(n,1);
            
            this.b = this.A * x0;

        end

        %% Calculates the value of the objective function.
        %  @param[in] x: Point at which to evaluate the objective.
        %  @param[out] J: Objective function value at the point 'x'.
        function J = fun(this, x)
            
            J = this.c'*x - sum( log(this.b-this.A*x) );
            
        end
        
        %% Calculates the gradient of the objective function.
        %  @param[in] x: Point at which to evaluate the gradient.
        %  @param[out] dJ: Gradient of the objective function value at the point 'x'.
        function dJ = gradFun(this, x)
            
            dJ = this.c + this.A' * ( ones(size(this.b)) ./ (this.b - this.A*x) );
            
        end
        
        %% Calculates the hessian of the objective function.
        %  @param[in] x: Point at which to evaluate the gradient.
        %  @param[out] H: Hessian of the objective function value at the point 'x'.
        function H = hessianFun(this, x)

            H = this.A' * diag( ( ones(size(this.b)) ./ (this.b - this.A*x) ).^2 ) * this.A;
            
        end
        
        %% Generates a point that lies in the domain of the problem.
        %  @param[out] x0: Point that is inside the domain of the problem.
        function x0 = genPointInDom(this)
            
            x0 = this.A \ (this.b - abs(this.b));
            
        end
        
    end
    
    properties (Access = private)
       
        c
        A
        b
        
    end

end