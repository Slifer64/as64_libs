%% Gradient Descent class
%

classdef NewtonDescent
       
    methods (Access = public)
        %% Gradient Descent class constructor.
        %  @param[in] objFun_ptr: Pointer to objective function.
        %  @param[in] gradObjFun_ptr: Pointer to gradient objective function.
        %  @param[in] lineSearch: Pointer to line search object. (optinal, default = BackTrackLineSearch)
        function this = NewtonDescent(objFun_ptr, gradObjFun_ptr, hessianObjFun_ptr, lineSearch)

            if (nargin < 4), lineSearch = BackTrackLineSearch(objFun_ptr, 0.01, 0.5); end
  
            this.objFun_ptr = objFun_ptr;
            this.gradObjFun_ptr = gradObjFun_ptr;
            this.hessianObjFun_ptr = hessianObjFun_ptr;
            this.lineSearch = lineSearch;

        end
        
        %% Calculates the minimum based on gradient descent
        %  @param[in] x0: Initial point.
        %  @param[in] eps: Stopping threshold (optinal, default = 1e-6).
        %  @param[in] max_iter: Maximum iterations (optinal, default = 100).
        function [x, x_data] = solve(this, x0, eps, max_iter)
            
            if (nargin < 3), eps = 1e-6; end
            if (nargin < 4), max_iter = 100; end

            x = x0;
            df = this.gradObjFun_ptr(x);
            ddf = this.hessianObjFun_ptr(x);
            
            iter = 1;
            
            x_data = [];

            while (true)
                
                if (iter > max_iter)
                    warning('Exiting due to maximum iterations reached...');
                    break;
                end
                iter = iter + 1;
                
                if (nargout > 1), x_data = [x_data x]; end
                
                dx = - ddf\df;
                t = this.lineSearch.run(x, dx, df);
                x = x + t*dx;

                df = this.gradObjFun_ptr(x);
                ddf = this.hessianObjFun_ptr(x);

                lambda2 = -0.5*df'*dx;
                if (lambda2 < eps), break; end
                    
            end
            
            if (nargout > 1), x_data = [x_data x]; end

        end
        
    end
    
    properties (Access = protected)
        
        objFun_ptr         % objective function pointer
        gradObjFun_ptr     % gradient of objective function pointer
        hessianObjFun_ptr  % hessian of objective function pointer
        lineSearch         % object to lineSearch object

    end
    

end

