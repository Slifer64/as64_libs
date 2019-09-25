%% BackTracking Line Search class
%

classdef BackTrackLineSearch
       
    methods (Access = public)
        %% BackTracking Line Search class constructor.
        %  @param[in] objFun_ptr: Pointer to objective function.
        %  @param[in] a: Fraction of the decrease of the objective function predicted by linear extrapolation. (optional, default = 0.01)
        %  @param[in] b: Update factor of step length. (optional, default = 0.5)
        function this = BackTrackLineSearch(objFun_ptr, a, b)

            if (nargin < 2), a = 0.01; end
            if (nargin < 3), b = 0.5; end
            
            this.objFun_ptr = objFun_ptr; 
            this.a = a;
            this.b = b;

        end
        
        %% Calculates the step length of the descent direction.
        %  @param[in] x: Current point.
        %  @param[in] dx: Descent direction.
        %  @param[in] df: Gradient of the objective at the current point.
        %  @param[out] t: The step length.
        function t = run(this, x, dx, df)
            
            t = 1;
            f = this.objFun_ptr(x);
            adfdx = this.a*dot(df,dx);

            if (~(isfinite(f) & isreal(f)))
                error('[BackTrackLineSearch::run]: The given point in not in the domain of the objective function...');
            end
            
            while (true)
                f_next = this.objFun_ptr(x + t*dx);
                if (isfinite(f_next) & isreal(f_next))
                    if ( f_next <= f + t*adfdx ), break; end
                end   
                t = this.b*t;
                if (t < 1e-30)
                   warning('[BackTrackLineSearch::run]: The step has grown too small. Line search may not converge...');
                   t = 0;
                   break;
                end
            end

        end
        
    end
    
    properties (Access = public)
        
        objFun_ptr   % objective function pointer
        a            % fraction of the decrease of the objective function predicted by linear extrapolation
        b            % update factor of step length

    end
    

end

