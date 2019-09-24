%% Gradient Descent class
%

classdef NewtonDescent < handle
       
    methods (Access = public)
        %% Gradient Descent class constructor.
        %  @param[in] objFun_ptr: Pointer to objective function.
        %  @param[in] gradObjFun_ptr: Pointer to gradient objective function.
        function this = NewtonDescent(N_var, objFun_ptr, gradObjFun_ptr, hessianObjFun_ptr)

            this.setLineSearch(BackTrackLineSearch(objFun_ptr, 0.01, 0.5));
            this.setKKTsolveMethod(KKTSolveMethod.FULL_INV);
            this.clearEqConstr();
            this.clearInEqConstr();
            this.setStopThreshold(1e-5);
            this.setMaxIters(100);
            this.setEqualCheckTol(1e-8);
  
            this.N_var = N_var;
            this.objFun_ptr = objFun_ptr;
            this.gradObjFun_ptr = gradObjFun_ptr;
            this.hessianObjFun_ptr = hessianObjFun_ptr;
            
            this.lowTriangSolve = dsp.LowerTriangularSolver;
            this.upperTriangSolve = dsp.UpperTriangularSolver;

        end
        
        %% Calculates the minimum of unconstrained convex problem.
        %  @param[in] x0: Initial point.
        %  @param[out] x: Optimal primal variable solution.
        %  @param[out] v: Optimal dual variable solution.
        %  @param[out] x_data: Matrix with the intermediate primal variable calculations (optional).
        %  @param[out] exit_code: Indicates the reason why the optimization stopped.
        function [x, v, x_data, exit_code] = solve(this, x0)
            
            if (this.eq_constr_flag)
                
                if (this.kkt_solve_method == KKTSolveMethod.EQ_ELIM)
                    [Q, R] = qr(this.A');
                    m = size(this.A,1);
                    Q1 = Q(:,1:m);
                    Q2 = Q(:,m+1:end);
                    R = R(1:m,:);

                    this.x_hat = Q1 * this.lowTriangSolve(R',this.b);
                    this.F = Q2;
                end
            
                if (this.ineq_constr_flag)
                    [x, v, x_data, exit_code] = this.solveEqIneqInterPoint(x0);
                else
                    [x, v, x_data, exit_code] = this.solveEq(x0);
                end
            else
                if (this.ineq_constr_flag)
                    [x, v, x_data, exit_code] = this.solveIneqInterPoint(x0);
                else
                    [x, v, x_data, exit_code] = this.solveUnconstr(x0);
                end
            end
        end
        
        %% Calculates the minimum of unconstrained convex problem.
        %  @param[in] x0: Initial point.
        %  @param[out] x: Optimal primal variable solution.
        %  @param[out] v: Optimal dual variable solution.
        %  @param[out] x_data: Matrix with the intermediate primal variable calculations (optional).
        %  @param[out] exit_code: Indicates the reason why the optimization stopped.
        function [x, v, x_data, exit_code] = solveUnconstr(this, x0)
            
            v = 0;
            
            x = x0;
            df = this.gradObjFun_ptr(x);
            ddf = this.hessianObjFun_ptr(x);
            
            iter = 1;
            
            x_data = [x];

            while (true)
                
                if (iter > this.max_iter)
                    warning('[NewtonDescent::solveUnconstr]: Exiting due to maximum iterations reached...');
                    exit_code = NewtonDescent.MAX_ITERS_REACHED;
                    break;
                end
                iter = iter + 1;
  
                dx = - ddf\df;
                t = this.line_search.run(x, dx, df);
                x = x + t*dx;
                
                if (nargout > 1), x_data = [x_data x]; end

                df = this.gradObjFun_ptr(x);
                ddf = this.hessianObjFun_ptr(x);

                lambda2 = -0.5*df'*dx;
                if (lambda2 < this.eps)
                    exit_code = NewtonDescent.TOLERANCE_REACHED;
                    break; 
                end
                    
            end

        end
        
        %% Calculates the minimum of convex problem with equality constraints.
        %  @param[in] x0: Initial point.
        %  @param[out] x: Optimal primal variable solution.
        %  @param[out] v: Optimal dual variable solution.
        %  @param[out] x_data: Matrix with the intermediate primal variable calculations (optional).
        %  @param[out] exit_code: Indicates the reason why the optimization stopped.
        function [x, v, x_data, exit_code] = solveEq(this, x0)
            
            if (~this.eq_constr_flag)
                error('[NewtonDescent::solveEq]: No equalitiy constraints are set...');
            end
            
            m = size(this.A,1);
            
            x = x0;
            g = this.gradObjFun_ptr(x);
            h = zeros(m,1);
            H = this.hessianObjFun_ptr(x);
            
            iter = 1;
            
            x_data = [x];

            while (true)
                
                if (iter > this.max_iter)
                    warning('[NewtonDescent::solveEq]: Exiting due to maximum iterations reached...');
                    exit_code = NewtonDescent.MAX_ITERS_REACHED;
                    break;
                end
                iter = iter + 1;

                [dx, v] = this.solveKKT_ptr(H, this.A, g, h);
              
                t = this.line_search.run(x, dx, g);
                x = x + t*dx;
                
                if (nargout > 2), x_data = [x_data x]; end

                g = this.gradObjFun_ptr(x);
                H = this.hessianObjFun_ptr(x);

                lambda2 = 0.5*dx'*H*dx;
                if (lambda2 < this.eps)
                    exit_code = NewtonDescent.TOLERANCE_REACHED;
                    break; 
                end
                    
            end

            if (nargout > 1), v = -this.A'\this.gradObjFun_ptr(x); end

        end
        
        %% Calculates the minimum of convex problem with equality constraints. The starting point needs not be feasible.
        %  @param[in] x0: Initial point.
        %  @param[out] x: Optimal primal variable solution.
        %  @param[out] v: Optimal dual variable solution.
        %  @param[out] x_data: Matrix with the intermediate primal variable calculations (optional).
        %  @param[out] exit_code: Indicates the reason why the optimization stopped.
        function [x, v, x_data, exit_code] = solveEqInfeasStart(this, x0)

            if (~this.eq_constr_flag)
                error('[NewtonDescent::solveEqInfeasStart]: No equalitiy constraints are set...');
            end
            
            m = size(this.A,1);
            
            if (this.kkt_solve_method == KKTSolveMethod.EQ_ELIM)
                error('[NewtonDescent::solveEqInfeasStart]: KKTSolveMethod.EQ_ELIM is not supported for this function.');
            end
            
            x = x0;
            v = zeros(m,1);
            
            iter = 1;
            
            x_data = [x];
            
            r_fun = @(x,v) norm([this.gradObjFun_ptr(x) + this.A'*v; this.A*x-this.b]);
            
            alpha = this.line_search.a;
            beta = this.line_search.b;
            
            while (true)

                if (iter > this.max_iter)
                    warning('[NewtonDescent::solveEqInfeasStart]: Exiting due to maximum iterations reached...');
                    exit_code = NewtonDescent.MAX_ITERS_REACHED;
                    break;
                end
                iter = iter + 1;

                g = this.gradObjFun_ptr(x) + this.A'*v;
                H = this.hessianObjFun_ptr(x);
                
                h = this.A*x - this.b;
                [dx, dv] = this.solveKKT_ptr(H, this.A, g, h);

                % back-tracking line search
                t = 1;
                r = r_fun(x,v);
                while (true)
                    Dx = t*dx;
                    Dv = t*dv;
                    r_next = r_fun(x+Dx,v+Dv);
                    if (r_next <= (1-alpha*t)*r), break; end
                    t = beta*t;         
                end

                x = x + t*dx;
                v = v + t*dv;
                
                if (nargout > 1), x_data = [x_data x]; end
                
                r = r_fun(x,v);

                if (norm(this.A*x-this.b)<1e-8 & r<this.eps)
                    exit_code = NewtonDescent.TOLERANCE_REACHED;
                    break;
                end
                    
            end

        end
         
        %% Calculates the minimum of convex problem with inequality constraints.
        %  @param[in] x0: Initial point.
        %  @param[out] x: Optimal primal variable solution.
        %  @param[out] v: Optimal dual variable solution.
        %  @param[out] x_data: Matrix with the intermediate primal variable calculations (optional).
        %  @param[out] exit_code: Indicates the reason why the optimization stopped.
        function [x, v, x_data, exit_code] = solveIneqInterPoint(this, x0)

            if (~this.ineq_constr_flag)
                error('[NewtonDescent::solveIneqInterPoint]: No inequalitiy constraints are set...');
            end
            
            v = [];
            
            % calc initial t
            [~, grad_phi, ~] = this.barrierFun(x0);
            A0 = this.gradObjFun_ptr(x0);
            b0 = grad_phi;
            z = -(A0\b0);
            t = z(1);
            
            x = x0;      
            iter = 1;
            x_data = [x];
            mu = 20;
            
            Neq = length(this.Fi(x0));
            
            out_iters = 0;
            inner_iters = [];
            
            t = 0.05;
            
            phi = this.barrierFun(x);
            J0 = t*this.objFun_ptr(x) + phi;

            % outer iteration
            while (Neq/t > this.eps)
                
                t = mu*t;
                
                out_iters = out_iters + 1;
            
                [~, grad_phi, hess_phi] = this.barrierFun(x);
                g = t*this.gradObjFun_ptr(x) + grad_phi;
                H = t*this.hessianObjFun_ptr(x) + hess_phi;
                
                inner_iters(out_iters) = 0;

                % inner iteration
                while (true)
                    
                    inner_iters(out_iters) = inner_iters(out_iters) + 1;

                    if (iter > this.max_iter)
                        warning('[NewtonDescent::solveIneqInterPoint]: Exiting due to maximum iterations reached...');
                        exit_code = NewtonDescent.MAX_ITERS_REACHED;
                        break;
                    end
                    iter = iter + 1;
 
                    dx = - H\g;
                    t1 = this.line_search.run(x, dx, g);
                    x = x + t1*dx;
                    
                    phi = this.barrierFun(x0);
                    J = t*this.objFun_ptr(x0) + phi
                    
                    t1
                    x
                    H
                    g
                    dx
                    disp('--------------------------')
                    pause
                    
                    if (nargout > 1), x_data = [x_data x]; end

                    [~, grad_phi, hess_phi] = this.barrierFun(x);
                    g = t*this.gradObjFun_ptr(x) + grad_phi;
                    H = t*this.hessianObjFun_ptr(x) + hess_phi;

                    lambda2 = 0.5*dx'*H*dx;
                    if (lambda2 < this.eps)
                        exit_code = NewtonDescent.TOLERANCE_REACHED;
                        break; 
                    end

                end
            
            end

            out_iters
            inner_iters
            total_iters = sum(inner_iters)
        end
        
        %% Calculates the minimum of convex problem with equality and inequality constraints.
        %  @param[in] x0: Initial point.
        %  @param[out] x: Optimal primal variable solution.
        %  @param[out] v: Optimal dual variable solution.
        %  @param[out] x_data: Matrix with the intermediate primal variable calculations (optional).
        %  @param[out] exit_code: Indicates the reason why the optimization stopped.
        function [x, v, x_data, exit_code] = solveEqIneqInterPoint(this, x0)

            if (~this.eq_constr_flag)
                error('[NewtonDescent::solveEqIneqInterPoint]: No equalitiy constraints are set...');
            end
            
            if (~this.ineq_constr_flag)
                error('[NewtonDescent::solveEqIneqInterPoint]: No inequalitiy constraints are set...');
            end
            
            if (nargin < 2)
                
                x0 = this.A \ this.b;
                s0 = max(this.Fi(x0));
                z0 = [s0; x0];
                
                phase1_solver = NewtonDescent(@this.ph1, @this.gradPh1, @this.hessPh1);
                phase1_solver.setEqConstr(this.A,this.b);
                phase1_solver.setInEqConstr(this, this.ph1Fi, this.ph1barrierFun);
                
                phase1_solver.setKKTsolveMethod(KKTSolveMethod.BLOCK_ELIM);
                
                
                [z, v, x_data] = solver.solve(z0);
                
            end
            
            % calc initial t
            [~, grad_phi, ~] = this.barrierFun(x0);
            A0 = [this.gradObjFun_ptr(x0) this.A'];
            b0 = grad_phi;
            z = -(A0\b0);
            t = z(1);
            v = z(2:end);

            m = size(this.A,1);
            
            x = x0;      
            iter = 1;
            x_data = [x];
            mu = 20;
            
            % outer iteration
            while (Neq/t > this.eps)
                
                t = mu*t;
            
                [~, grad_phi, hess_phi] = this.barrierFun(x);
                g = t*this.gradObjFun_ptr(x) + grad_phi;
                H = t*this.hessianObjFun_ptr(x) + hess_phi;

                % inner iteration
                while (true)

                    if (iter > this.max_iter)
                        warning('[NewtonDescent::solveIneqInterPoint]: Exiting due to maximum iterations reached...');
                        exit_code = NewtonDescent.MAX_ITERS_REACHED;
                        break;
                    end
                    iter = iter + 1;

                    h = zeros(m,1); % A*x - b;
                    [dx, v] = this.solveKKT_ptr(H, A, g, h);

                    t1 = this.line_search.run(x, dx, g);
                    x = x + t1*dx;
                    
                    if (nargout > 1), x_data = [x_data x]; end

                    [~, grad_phi, hess_phi] = this.barrierFun(x);
                    g = t*this.gradObjFun_ptr(x) + grad_phi;
                    H = t*this.hessianObjFun_ptr(x) + hess_phi;

                    lambda2 = 0.5*dx'*H*dx;
                    if (lambda2 < this.eps)
                        exit_code = NewtonDescent.TOLERANCE_REACHED;
                        break; 
                    end

                end
            
            end

        end

        %% Sets the object that will be used in line search.
        %  @param[in] line_search: Pointer to line search object.
        function setLineSearch(this, line_search)
            
            this.line_search = line_search;
            
        end
        
        %% Sets the method for solving the KKT system.
        %  @param[in] kkt_solve_method: Enum of type @KKTSolveMethod, specifying the method for solving the KKT system.
        function setKKTsolveMethod(this, kkt_solve_method)
            
            % solveKKT_ptr: pointer to KKT solve function
            if (kkt_solve_method == KKTSolveMethod.FULL_INV), this.solveKKT_ptr = @this.solveKKTFullInv; 
            elseif (kkt_solve_method == KKTSolveMethod.BLOCK_ELIM), this.solveKKT_ptr = @this.solveKKTBlockElim; 
            elseif (kkt_solve_method == KKTSolveMethod.EQ_ELIM), this.solveKKT_ptr = @this.solveKKTEqElim; 
            else, error('[NewtonDescent::setKKTsolveMethod]: Unsupported KKT solve method...');
            end
            
            this.kkt_solve_method = kkt_solve_method;
            
        end
            
        %% Sets the equality constraints.
        %  @param[in] A: Equalities constraint matrix.
        %  @param[in] b: Equalities constraint vector.
        function setEqConstr(this, A,b)
            
            this.eq_constr_flag = true;
            this.A = A;
            this.b = b;
            
        end
        
        %% Clears the equality constraints.
        function clearEqConstr(this)
            
            this.eq_constr_flag = false;
            
        end
        
        %% Sets the inequality constraints.
        %  @param[in] Fi: Pointer to function that returns a vector with the value of each inequality constraint.
        %  @param[in] barrierFun: Pointer to function that calculates the value, the gradient and hessian of the log barrier function.
        function setInEqConstr(this, ineq_const)
            
            this.ineq_constr_flag = true;
            this.ineq_const = ineq_const;
            
        end
        
        function setLinIneqConstr(this, Ai, bi)
            
            this.setInEqConstr(LinIneqConstr(Ai,bi));
            
        end
        
        %% Clears the inequality constraints.
        function clearInEqConstr(this)
            
            this.ineq_constr_flag = false;
            
        end
        
        %% Sets the stopping threshold for the solve method.
        %  @param[in] eps: Stopping threshold.
        function setStopThreshold(this, eps)
            
            this.eps = eps;
            
        end
        
        %% Sets the maximum iterations for the solve method.
        %  @param[in] max_iter: Maximum iterations.
        function setMaxIters(this, max_iter)
            
            this.max_iter = max_iter;
            
        end

        %% Sets the tolerance for checking if an equality holds.
        %  @param[in] tol: Tolerance for checking if an equality holds.
        function setEqualCheckTol(this, tol)
            
            this.eq_check_tol = tol;
            
        end
            
    end
    
    methods (Access = private)
        
        %% Solves iteratively the KKT system using full inversion of the KKT matrix.
        function [dx, v] = solveKKTFullInv(this, H, A, g, h)
            
            [m, n] = size(A);
            
            z = [H A'; A zeros(m,m)] \ -[g; h];
            dx = z(1:n);
            v = z(n+1:end);
            
        end
        
        %% Solves iteratively the KKT system using variable elimination.
        function [dx, w] = solveKKTBlockElim(this, H, A, g, h)
            
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
        
        %% Solves iteratively the KKT system by eliminating the equality constraints and solving an unconstrained system.
        function [dx, w] = solveKKTEqElim(this, H, A, g, h)
            
            dz = -(this.F'*H*this.F) \ (this.F'*g);
            dx = this.F * dz;
            w = [];

        end
        
        function fo = ph1(this, x)
            
            fo = x(1);
            
        end
        
        function g = gradPh1(this, x)
            
            g = [1; zeros(this.N_var,1)];
            
        end
        
        function H = hessPh1(this, x)
            
            n = this.N_var + 1;
            H = zeros(n,n);
            H(1,1) = 1;
            
        end
        
        function fi = ph1Fi(this, x)
           
            s = x(1);
            x = x(2:end);
            fi = this.Fi(x) - s;
            
        end

        function [x, exit_code] = phase1solve(phase1BarrierFun)
            
            if (~this.ineq_constr_flag)
                error('[NewtonDescent::solveIneqInterPoint]: No inequalitiy constraints are set...');
            end
            
            v = [];
            
            % calc initial t
            [~, grad_phi, ~] = this.barrierFun(x0);
            A0 = this.gradObjFun_ptr(x0);
            b0 = grad_phi;
            z = -(A0\b0);
            t = z(1);
            
            x = x0;      
            iter = 1;
            x_data = [x];
            mu = 20;
            
            Neq = length(this.Fi(x0));
            
            out_iters = 0;
            inner_iters = [];
            
            t = 0.05;
            
            phi = this.barrierFun(x);
            J0 = t*this.objFun_ptr(x) + phi;

            % outer iteration
            while (Neq/t > this.eps)
                
                t = mu*t;
                
                out_iters = out_iters + 1;
            
                [~, grad_phi, hess_phi] = this.barrierFun(x);
                g = t*this.gradObjFun_ptr(x) + grad_phi;
                H = t*this.hessianObjFun_ptr(x) + hess_phi;
                
                inner_iters(out_iters) = 0;

                % inner iteration
                while (true)
                    
                    inner_iters(out_iters) = inner_iters(out_iters) + 1;

                    if (iter > this.max_iter)
                        warning('[NewtonDescent::solveIneqInterPoint]: Exiting due to maximum iterations reached...');
                        exit_code = NewtonDescent.MAX_ITERS_REACHED;
                        break;
                    end
                    iter = iter + 1;
 
                    dx = - H\g;
                    t1 = this.line_search.run(x, dx, g);
                    x = x + t1*dx;
                    
                    phi = this.barrierFun(x0);
                    J = t*this.objFun_ptr(x0) + phi
                    
                    t1
                    x
                    H
                    g
                    dx
                    disp('--------------------------')
                    pause
                    
                    if (nargout > 1), x_data = [x_data x]; end

                    [~, grad_phi, hess_phi] = this.barrierFun(x);
                    g = t*this.gradObjFun_ptr(x) + grad_phi;
                    H = t*this.hessianObjFun_ptr(x) + hess_phi;

                    lambda2 = 0.5*dx'*H*dx;
                    if (lambda2 < this.eps)
                        exit_code = NewtonDescent.TOLERANCE_REACHED;
                        break; 
                    end

                end
            
            end

            out_iters
            inner_iters
            total_iters = sum(inner_iters)
        end
        
    end
    
    properties (Constant, Access = public)
        
        TOLERANCE_REACHED = 0;
        MAX_ITERS_REACHED = 1;
        INFEASIBLE = 2;
        UNBOUNDED = 3;
        GENERAL_ERROR = 4;

    end
    
    properties (Access = protected)
        
        objFun_ptr         % objective function pointer
        gradObjFun_ptr     % gradient of objective function pointer
        hessianObjFun_ptr  % hessian of objective function pointer
        line_search         % object to line_search object
        
        eps      % stopping threshold
        max_iter % maximum iterations
        eq_check_tol % tollerance for checking if an equality holds
        
        lowTriangSolve   % solver for linear equations with low triangular matrix
        upperTriangSolve % solver for linear equations with upper triangular matrix
        
        % used in solving the equality constrained problem with constraints elimination
        x_hat  % a particular solution of the equality constraints
        F      % the columns of F span the null space of the equality constraint matrix
        
        A % equalities constraint matrix
        b % equalities constraint vector
        eq_constr_flag % flag that is true if equality constraints are set
        
        ineq_constr % @IneqConstr object
        ineq_constr_flag % flag that is true if inequality constraints are set

        barrierFun % pointer to function that calculates the value, the gradient and hessian of the log barrier function
        ph1barrierFun % pointer to function that calculates the value, the gradient and hessian of the log barrier function for the phaseI method
        
        
        kkt_solve_method % method for solving the KKT system
        solveKKT_ptr % pointer to method for solving the KKT system
        
        N_var % number of optimization variables

    end
    

end

