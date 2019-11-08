%% Gradient Descent class
%

classdef NewtonDescent < handle
       
    properties (Constant, Access = public)
        
        TOLERANCE_REACHED = 0;
        MAX_ITERS_REACHED = 1;
        INFEASIBLE = 2;
        UNBOUNDED = 3;
        GENERAL_ERROR = 4;

    end
    
    methods (Access = public)
        %% Constructor.
        %  @param[in] objFun_ptr: Pointer to objective function.
        %  @param[in] gradObjFun_ptr: Pointer to gradient objective function.
        %  @param[in] gradObjFun_ptr: Pointer to hessian objective function.
        function this = NewtonDescent(N_var, objFun_ptr, gradObjFun_ptr, hessianObjFun_ptr)

            this.setLineSearch(BackTrackLineSearch(objFun_ptr, 0.01, 0.5));
            this.setKKTSolveMethod(KKTSolver.FULL_INV);
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
            
            this.setDamping(0);

        end
        
        %% Calculates the minimum of unconstrained convex problem.
        %  @param[in] x0: Initial point.
        %  @param[out] x: Optimal primal variable solution.
        %  @param[out] v: Optimal dual variable solution.
        %  @param[out] x_data: Matrix with the intermediate primal variable calculations (optional).
        %  @param[out] exit_code: Indicates the reason why the optimization stopped.
        function [x, v, x_data, exit_code] = solve(this, x0)
            
            if (this.eq_constr_flag)
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
            
            Id = this.d*eye(this.N_var, this.N_var);
            v = 0;
            x = x0;
            iter = 1;
            x_data = [x];

            while (true)
 
                if (iter > this.max_iter)
                    warning('[NewtonDescent::solveUnconstr]: Exiting due to maximum iterations reached...');
                    exit_code = NewtonDescent.MAX_ITERS_REACHED;
                    break;
                end
                iter = iter + 1;
                
                df = this.gradObjFun_ptr(x);
                ddf = this.hessianObjFun_ptr(x) + Id;
                dx = - ddf\df;
                
                lambda2 = -0.5*df'*dx;
                if (lambda2 < this.eps)
                    exit_code = NewtonDescent.TOLERANCE_REACHED;
                    break; 
                end
                
                t = this.line_search.run(x, dx, df);
                x = x + t*dx;
                
                if (nargout > 1), x_data = [x_data x]; end
 
            end

        end
        
        %% Calculates the minimum of convex problem with equality constraints.
        %  @param[in] x0: Initial point.
        %  @param[out] x: Optimal primal variable solution.
        %  @param[out] v: Optimal dual variable solution.
        %  @param[out] x_data: Matrix with the intermediate primal variable calculations (optional).
        %  @param[out] exit_code: Indicates the reason why the optimization stopped.
        function [x, v, x_data, exit_code] = solveEq(this, x0)
            
            if (~this.eq_constr_flag), error('[NewtonDescent::solveEq]: No equalitiy constraints are set...'); end
            
            Id = this.d*eye(this.N_var, this.N_var);
            
            if (max(abs((this.A*x0-this.b))) > 1e-8), x0 = this.A\this.b; end
            
            m = size(this.A,1);
            x = x0;
            h = zeros(m,1);
            iter = 1;
            x_data = [x];

            while (true)
                
                if (iter > this.max_iter)
                    warning('[NewtonDescent::solveEq]: Exiting due to maximum iterations reached...');
                    exit_code = NewtonDescent.MAX_ITERS_REACHED;
                    break;
                end
                iter = iter + 1;
                
                g = this.gradObjFun_ptr(x);
                H = this.hessianObjFun_ptr(x) + Id;

                [dx, v] = this.kkt_solver(H, this.A, g, h);
                
                lambda2 = 0.5*dx'*H*dx;
                if (lambda2 < this.eps)
                    exit_code = NewtonDescent.TOLERANCE_REACHED;
                    break; 
                end
              
                t = this.line_search.run(x, dx, g);
                x = x + t*dx;
                
                if (nargout > 2), x_data = [x_data x]; end
                    
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
            
            if (this.kkt_solve_method == KKTSolver.EQ_ELIM)
                error('[NewtonDescent::solveEqInfeasStart]: KKTSolver.EQ_ELIM is not supported for this function.');
            end
            
            Id = this.d*eye(this.N_var, this.N_var);
            
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
                H = this.hessianObjFun_ptr(x) + Id;
                
                h = this.A*x - this.b;
                [dx, dv] = this.kkt_solver(H, this.A, g, h);

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
            x = x0;
            x_data = [];
            
            phase1 = Phase1Solver(this.ineq_constr);
            [x0, s] = phase1.solve(x0);
            if (s > 0)
                exit_code = NewtonDescent.INFEASIBLE;
                return;
            end
            
            Id = this.d*eye(this.N_var, this.N_var);

            iter = 1;
            x = x0;
            x_data = [x];
            mu = 20;
            
            Neq = length(this.ineq_constr.fi(x0));
            
            out_iters = 0;
            inner_iters = [];
            
            % t = 1/mu;
            t = 20;

            % outer iteration
            while (Neq/t > this.eps)
                
                t = mu*t;
                
                out_iters = out_iters + 1;
                inner_iters(out_iters) = 0;
                
                objFun = @(x) t*this.objFun_ptr(x) + this.ineq_constr.logBarFun(x);
                line_search2 = BackTrackLineSearch(objFun, this.line_search.a, this.line_search.b);

                stop_thres = 1e-3;
                % if (Neq/t <= this.eps), stop_thres = 0.1; end
                    
                % inner iteration
                while (true)
                    
                    inner_iters(out_iters) = inner_iters(out_iters) + 1;

                    if (iter > this.max_iter)
                        break;
                    end
                    iter = iter + 1;
                    
                    [~, grad_phi, hess_phi] = this.ineq_constr.logBarFun(x);
                    g = t*this.gradObjFun_ptr(x) + grad_phi;
                    H = t*this.hessianObjFun_ptr(x) + hess_phi + Id;
 
                    dx = -H\g;
                    
                    lambda2 = 0.5*dx'*H*dx;
                    if (lambda2 < stop_thres)
                        exit_code = NewtonDescent.TOLERANCE_REACHED;
                        break; 
                    end

                    t1 = line_search2.run(x, dx, g);
                    x = x + t1*dx;
                    
                    if (nargout > 1), x_data = [x_data x]; end

                end
                
                if (iter > this.max_iter)
                    warning('[NewtonDescent::solveIneqInterPoint]: Exiting due to maximum iterations reached...');
                    exit_code = NewtonDescent.MAX_ITERS_REACHED;
                    break;
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

            v = [];
            x = x0;
            x_data = [];
            
            phase1 = Phase1Solver(this.ineq_constr);
            phase1.setEqConstr(this.A, this.b);
            [x0, s] = phase1.solve(x0);
            if (s > 1e-12)
                exit_code = NewtonDescent.INFEASIBLE;
                return;
            end

            Id = this.d*eye(this.N_var, this.N_var);

            iter = 1;
            x = x0;
            x_data = [x];
            mu = 20;
            
            Neq = length(this.ineq_constr.fi(x0));
            
            out_iters = 0;
            inner_iters = [];
            
            t = 0.1/mu;
            h = zeros(size(this.A,1),1);

            % outer iteration
            while (Neq/t > this.eps)
                
                t = mu*t;
                
                out_iters = out_iters + 1;
                inner_iters(out_iters) = 0;
                
                objFun = @(x) t*this.objFun_ptr(x) + this.ineq_constr.logBarFun(x);
                line_search2 = BackTrackLineSearch(objFun, this.line_search.a, this.line_search.b);

                stop_thres = 1e-3;
                % if (Neq/t <= this.eps), stop_thres = 0.1; end
                    
                % inner iteration
                while (true)
                    
                    inner_iters(out_iters) = inner_iters(out_iters) + 1;

                    if (iter > this.max_iter)
                        break;
                    end
                    iter = iter + 1;
                    
                    iter
                    
                    [~, grad_phi, hess_phi] = this.ineq_constr.logBarFun(x);
                    g = t*this.gradObjFun_ptr(x) + grad_phi;
                    H = t*this.hessianObjFun_ptr(x) + hess_phi + Id;
 
                    [dx, v] = this.kkt_solver.solve(H, this.A, g, h);
                    
                    lambda2 = 0.5*dx'*H*dx;
                    if (lambda2 < stop_thres)
                        exit_code = NewtonDescent.TOLERANCE_REACHED;
                        break; 
                    end

                    t1 = line_search2.run(x, dx, g);
                    x = x + t1*dx;
                    
                    if (nargout > 1), x_data = [x_data x]; end

                end
                
                if (iter > this.max_iter)
                    warning('[NewtonDescent::solveEqIneqInterPoint]: Exiting due to maximum iterations reached...');
                    exit_code = NewtonDescent.MAX_ITERS_REACHED;
                    break;
                end
            
            end
            
            out_iters
            inner_iters
            total_iters = sum(inner_iters)

        end

        %% Sets the object that will be used in line search.
        %  @param[in] line_search: Pointer to line search object.
        function setLineSearch(this, line_search)
            
            this.line_search = line_search;
            
        end
        
        %% Sets the method for solving the KKT system.
        %  @param[in] kkt_solve_method: Enum from @KKTSolver, specifying the method for solving the KKT system.
        function setKKTSolveMethod(this, kkt_solve_method)
            
            if (kkt_solve_method == KKTSolver.FULL_INV), this.kkt_solver = KKTSolver(KKTSolver.FULL_INV);
            elseif (kkt_solve_method == KKTSolver.BLOCK_ELIM), this.kkt_solver = KKTSolver(KKTSolver.BLOCK_ELIM);
            elseif (kkt_solve_method == KKTSolver.EQ_ELIM), this.kkt_solver = KKTSolver(KKTSolver.EQ_ELIM);
            else, error('[NewtonDescent::setKKTsolveMethod]: Unsupported KKT solve method...');
            end

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
        function setInEqConstr(this, ineq_constr)
            
            this.ineq_constr_flag = true;
            this.ineq_constr = ineq_constr;
            
        end
        
        function setLinIneqConstr(this, Ai, bi)
            
            this.setInEqConstr(LinIneqConstr(Ai,bi));
            
        end
        
        %% Clears the inequality constraints.
        function clearInEqConstr(this)
            
            this.ineq_constr_flag = false;
            
        end
        
        %% Set damping for damped Newton method.
        %  @param[in] d: damping value.
        function setDamping(this, d)
            
            this.d = d;
            
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
        
        A % equalities constraint matrix
        b % equalities constraint vector
        eq_constr_flag % flag that is true if equality constraints are set
        
        ineq_constr % @IneqConstr object
        ineq_constr_flag % flag that is true if inequality constraints are set

        kkt_solver % pointer to @KKTSolver for solving the KKT system
        
        N_var % number of optimization variables
        
        d % damping value for damped Newton method
        
        phase1_solver % pointer to @Phase1Solver

    end
    

end

