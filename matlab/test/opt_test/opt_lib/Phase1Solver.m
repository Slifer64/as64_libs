%% Phase 1 solver class
%

classdef Phase1Solver < handle
       
    methods (Access = public)
        %% Constructor.
        %  @param[in] ineq_constr: @IneqConstr pointer.
        function this = Phase1Solver(ineq_constr)

            this.eps = 1e-6;
            
            this.ineq_constr = ineq_constr;

            this.barrierFun = @(x)ph1LogBarFun(this.ineq_constr,x);
            
            this.setDamping(0.01);
            
            this.setKKTSolveMethod(KKTSolver.FULL_INV);
            
        end
        
        %% Sets the equality constraints.
        %  @param[in] A: Equality constraints matrix.
        %  @param[in] b: Equality constraints vector.
        function setEqConstr(this, A, b)
            
            this.eq_constr_flag = true;
            this.A = A;
            this.b = b;
            
        end
        
        %% Clears the equality constraints.
        function clearEqConstr(this)
            
            this.eq_constr_flag = false;
            
        end
        
        %% Set damping for damped Newton method.
        %  @param[in] d: damping value.
        function setDamping(this, d)
            
            this.d = d;
            
        end
        
        %% Sets the method for solving the KKT system.
        %  @param[in] kkt_solve_method: Enum of type @KKTSolveMethod, specifying the method for solving the KKT system.
        function setKKTSolveMethod(this, kkt_solve_method)

            this.kkt_solver = KKTSolver(kkt_solve_method);
            
        end
        
        %% Solves the phase1 problem.
        %  @param[in] x0: Initial point.
        %  @param[out] x: Ouput point.
        %  @param[out] s: Auxiliary variable of phase1 problem. If s<0 the phase1 problem is feasible.
        function [x, s] = solve(this, x0)
            
            if (this.eq_constr_flag), [x, s] = this.solveWithEqConstr(x0);
            else, [x, s] = this.solveNoEqConstr(x0);
            end
            
        end

    end
    
    methods (Access = private)
       
        function [x, s] = solveWithEqConstr(this, x0)

            if (~this.eq_constr_flag), error('[Phase1Solver::solveWithEqConstr]: No equality constraints set...'); end
                
            if (max(abs((this.A*x0-this.b))) > 1e-8), x0 = this.A\this.b; end
            
            s = max(this.ineq_constr.fi(x0)) + 0.1;
            x = [s; x0];
            Id = this.d*eye(length(x), length(x)); % damping matrix
            % x_data = [x];
            t = 1;
            objFun = @(x) t*x(1) + this.barrierFun(x);
            line_search = BackTrackLineSearch(objFun, 0.01, 0.5);
            grad_ts = [t; zeros(length(x)-1,1)];
            m = size(this.A,1);
            h = zeros(m,1);
            A2 = [zeros(m,1) this.A];
            
            while (true)
                
                if (x(1) <= 0), break; end
                
                [~, grad_phi, hess_phi] = this.barrierFun(x);
                g = grad_ts + grad_phi;
                H = (hess_phi + Id);

                [dx, v] = this.kkt_solver.solve(H, A2, g, h);
                
                lambda2 = 0.5*dx'*H*dx;
                if (lambda2 < this.eps), break; end
                
                t1 = line_search.run(x, dx, g);
                x = x + t1*dx;
                % x_data = [x_data x];
                
            end
            
            s = x(1);
            x = x(2:end);
            
        end
        
        function [x, s] = solveNoEqConstr(this, x0)

            s = max(this.ineq_constr.fi(x0)) + 0.1;
            x = [s; x0];
            Id = this.d*eye(length(x), length(x)); % damping matrix
            % x_data = [x];
            t = 1;
            objFun = @(x) t*x(1) + this.barrierFun(x);
            line_search = BackTrackLineSearch(objFun, 0.01, 0.5);
            grad_ts = [t; zeros(length(x)-1,1)];

            while (true)
                
                if (x(1) <= 0), break; end
                
                [~, grad_phi, hess_phi] = this.barrierFun(x);
                g = grad_ts + grad_phi;
                H = (hess_phi + Id);

                dx = - H\g;
                
                lambda2 = 0.5*dx'*H*dx;
                if (lambda2 < this.eps), break; end
                
                t1 = line_search.run(x, dx, g);
                x = x + t1*dx;
                % x_data = [x_data x];
                
            end
            
            s = x(1);
            x = x(2:end);
            
        end

    end

    properties (Access = protected)

        eps              % stopping tolerance  
        eq_constr_flag   % flag indicating if equality constraints are set
        A                % Equality constraints matrix.
        b                % Equality constraints vector.
        ineq_constr      % @IneqConstr pointer.
        barrierFun
        d                % damping value for damped Newton method.
        kkt_solver       % pointer of type @KKTSolver
        
    end
    
end

