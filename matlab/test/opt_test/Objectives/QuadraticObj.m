%% Quadratic Objective function class
%

classdef QuadraticObj < Objective
       
    methods (Access = public)
        %% Objective function class constructor.
        %  @param[in] n: The problem dimension. 
        %  @param[in] P: The positive definite matrix in x'*P*x term.
        %  @param[in] q: The vector in q'*x term.
        %  @param[in] r: The constant term.
        function this = QuadraticObj(n, P, q, r)

            if (nargin < 2), P = diag(10*abs(rand(n,1))); end
            if (nargin < 3), q = zeros(n,1); end
            if (nargin < 4), r = 0; end

            this.P = P;
            this.q = q;
            this.r = r;

        end

        %% Calculates the value of the objective function.
        %  @param[in] x: Point at which to evaluate the objective.
        %  @param[out] J: Objective function value at the point 'x'.
        function J = fun(this, x)
            
            J = x'*this.P*x + dot(this.q, x) + this.r;
            
        end
        
        %% Calculates the gradient of the objective function.
        %  @param[in] x: Point at which to evaluate the gradient.
        %  @param[out] dJ: Gradient of the objective function value at the point 'x'.
        function dJ = gradFun(this, x)
            
            dJ = 2*this.P*x + this.q;
            
        end
        
    end
    
    methods (Static, Access = public)
       
        %% Generates a random positive definite matrix.
        %  @param[in] n: The matrix dimension.
        %  $param[out] P: The generated matrix.
        function P = genRandP(n)
            
            L = zeros(n,n);
            for i=1:n, L(i,1:i) = rand(1,i); end
            for i=1:n
               L(i,i) = abs(L(i,i));
               if (L(i,i) < 0.01), L(i,i) = 0.01; end
            end
            P = L*L';
            
        end
        
    end
    
    properties (Access = private)
       
        P % positive definite matrix in x'*P*x term
        q % vector in q'*x term
        r % constant term
        
    end

end