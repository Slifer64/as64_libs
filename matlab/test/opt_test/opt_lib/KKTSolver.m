%% KKT solver
%

classdef KKTSolver < handle
       
    properties (Constant)
        
        FULL_INV = 0 % Invert the entire KKT matrix, O((m+n)^3)
        BLOCK_ELIM = 1 % Solve KKT system with block elimination, O(n*m^2)
        EQ_ELIM = 2 % Eliminate equality constraints and solve unconstrained problem, O(n*(n-m)^2)
        
    end
    
    methods (Access = public)
        
        %% Constructor.
        %  @param[in] kkt_solve_method: enum of @KKTSolver. (optional, default=KKTSolver.FULL_INV)
        function this = KKTSolver(kkt_solve_method)
            
            if (nargin < 1), kkt_solve_method = KKTSolver.FULL_INV; end
            
            this.setMethod(kkt_solve_method);
            
        end
        
        %% Solves the KKT system using the method specified by @setMethod.
        %  @param[in] H: The hessian of the objective.
        %  @param[in] A: The equality constraints matrix.
        %  @param[in] g: The gradient of the objective.
        %  @param[in] h: The equality constraints residual, i.e. A*x-b.
        %  @param[out] v: The dual variable. (optional)
        %  @return x: The primal variable.
        function [dx, v] = solve(this, H, A, g, h)
            
            [dx, v] = this.solveKKT_ptr(H, A, g, h);
            
        end
        
        %% Sets the method for solving the KKT system.
        %  @param[in] kkt_solve_method: Enum of type @KKTSolveMethod, specifying the method for solving the KKT system.
        %  @param[in] A: Equality constraints matrix (required only for @KKTSolver.EQ_ELIM method).
        function setMethod(this, kkt_solve_method, A)

            if (nargin < 3), A = []; end
            
            % solveKKT_ptr: pointer to KKT solve function
            if (kkt_solve_method == KKTSolver.FULL_INV)
                this.solveKKT_ptr = @KKTSolver.solveKKTFullInv; 
            elseif (kkt_solve_method == KKTSolver.BLOCK_ELIM)
                this.solveKKT_ptr = @KKTSolver.solveKKTBlockElim; 
            elseif (kkt_solve_method == KKTSolver.EQ_ELIM)
                if (nargin < 3), error('[KKTSolver::setMethod]: EQ_ELIM method requires the equality constraints matrix...'); end
                Q = qr(A'); % [Q, R] = qr(this.A');
                m = size(A,1);
                % Q1 = Q(:,1:m);
                Q2 = Q(:,m+1:end);
                % R = R(1:m,:);
                % x_hat = Q1 * this.lowTriangSolve(R',this.b);
                F = Q2;
                this.solveKKT_ptr = @(H, A, g, h)KKTSolver.solveKKTEqElim(F, H, A, g, h); 
            else
                error('[KKTSolver::setMethod]: Unsupported KKT solve method...');
            end
            
            this.kkt_solve_method = kkt_solve_method;
            
        end
   
    end
    
    methods (Static, Access = private)
        
        %% Solves the KKT system using full inversion of the KKT matrix.
        function [dx, v] = solveKKTFullInv(H, A, g, h)
            
            [m, n] = size(A);
            
            H1 = [H A'; A zeros(m,m)];
            
            size(H1)
            rank(H1)
            pause
            
            z = [H A'; A zeros(m,m)] \ -[g; h];
            dx = z(1:n);
            v = z(n+1:end);
            
        end
        
        %% Solves the KKT system using variable elimination.
        function [dx, w] = solveKKTBlockElim(H, A, g, h)
            
            [m,n] = size(A);
            
%             if (rank(H) < n) % (cond(H) > 1e7) or sigma_min/sigma_max<1e-6 
%                 H = H + A'*A;
%                 g = g + A'*h;
%             end
            
%             L = chol(H,'lower');
%             temp = this.upperTriangSolve(L', this.lowTriangSolve(L,[A' g]));
            temp = H \ [A' g];
            
            invH_transA = temp(:,1:m);
            invH_g = temp(:,m+1);
            w = (A*invH_transA) \ (h - A*invH_g);

%             lowSolve = dsp.LowerTriangularSolver;
%             upSolve = dsp.UpperTriangularSolver;
%             dx = upSolve(L', lowSolve(L, (-A'*w - g) ));
            % dx = H \ (-A'*w - g);
            dx = -(invH_transA*w + invH_g);
            % dx = this.upperTriangSolve(L', this.lowTriangSolve(L, (-A'*w - g) ));

        end
        
        %% Solves the KKT system by eliminating the equality constraints and solving an unconstrained system.
        function [dx, w] = solveKKTEqElim(F, H, A, g, h)
            
            dz = -(F'*H*F) \ (F'*g);
            dx = F * dz;
            w = [];

        end

    end
    
    properties (Access = private)
        
        kkt_solve_method
        solveKKT_ptr
        
        d % damping value
    end

end

