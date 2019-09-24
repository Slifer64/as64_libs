%% Phase 1 solver class
%

classdef Phase1Solver < handle
       
    methods (Access = public)
        %% Constructor.
        %  @param[in] ineq_constr: @IneqConstr pointer.
        function this = Phase1Solver(ineq_constr)

            this.eps = 1e-6;
            
            this.ineq_constr = ineq_constr;

            this.barrierFun = @this.ineq_constr.ph1LogBarFun;
            
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
        
        function solve(this)
            
            if (this.eq_constr_flag), this.solveWithEqConstr()
            else, this.solveNoEqConstr()
            end
            
        end
        
        function solveWithEqConstr(this)
            
        end
        
        function [x, s] = solveNoEqConstr(this, x0)

            y = this.ineq_constr.fi(x0);
            Neq = length(y);
            s = max(this.ineq_constr.fi(x0)) + 0.001;
            
            x = [s; x0];      
            iter = 1;
            x_data = [x];
            mu = 20;
            
            out_iters = 0;
            inner_iters = [];
            
            t = 0.005;

            % outer iteration
            while (Neq/t > this.eps)
                
                if (x(1) <= 0), break; end
                
                t = mu*t;
                
                out_iters = out_iters + 1;
            
                [~, grad_phi, hess_phi] = this.barrierFun(x);
                g = t*this.gradObjFun_ptr(x) + grad_phi;
                H = t*this.hessianObjFun_ptr(x) + hess_phi;
                
                inner_iters(out_iters) = 0;

                % inner iteration
                while (true)
                    
                    inner_iters(out_iters) = inner_iters(out_iters) + 1;

%                     if (iter > this.max_iter)
%                         warning('[NewtonDescent::solveIneqInterPoint]: Exiting due to maximum iterations reached...');
%                         exit_code = NewtonDescent.MAX_ITERS_REACHED;
%                         break;
%                     end
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
                    if (lambda2 < this.eps) break; end

                end
            
            end
            
            s = x(1);
            x = x(2:end);

            out_iters
            inner_iters
            total_iters = sum(inner_iters)
            
        end


    end

    properties (Access = protected)

        eps              % stopping tolerance
        
        eq_constr_flag
        A                % Equality constraints matrix.
        b                % Equality constraints vector.

        ineq_constr      % @IneqConstr pointer.
        
        barrierFun
        
    end
    
end

