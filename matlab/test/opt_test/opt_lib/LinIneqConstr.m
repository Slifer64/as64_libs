%% Linear Inequality constraitns class
%

classdef LinIneqConstr < IneqConstr
       
    methods (Access = public)
        %% Constructor. Sets the inequality constraints A*x <= b.
        %  @param[in] Ai: Inquality constraints matrix.
        %  @param[in] bi: Inquality constraints vector.
        function this = LinIneqConstr(Ai, bi)

            this.Ai = Ai;
            this.bi = bi;

        end
        
        function yi = fi(this, x)
            
            yi = this.Ai*x - this.bi;
            
        end
        
        function [phi, grad_phi, hess_phi] = logBarFun(this, x)
            
            y = this.Ai*x - this.bi;
            
            phi = -sum(log(-y));
            
            m = length(this.bi);
            d = ones(m,1)./y;
                
            if (nargout > 1), grad_phi = this.Ai' * d; end
            if (nargout > 2), hess_phi = this.Ai' * diag(d.^2) * this.Ai; end
            
        end
        
        function [phi, grad_phi, hess_phi] = ph1LogBarFun(this, x)
            
            m = size(this.bi);
            Ai2 = [-ones(m,1) this.Ai];
            
            y = Ai2*x - this.bi;
            
            phi = -sum(log(-y));
            
            m = size(this.bi);
            d = ones(m,1)./y;
                
            if (nargout > 1), grad_phi = Ai2' * d; end
            if (nargout > 2), hess_phi = Ai2' * diag(d.^2) * Ai2; end
            
            % x_bar = [s; x];
            % fi_bar = @(x_bar) fi(x_bar(2:end)) - x_bar(1);
            % phi(x_bar) = -sum(log(fi_bar(x_bar)));
            % grad_fi_bar = [-1; grad_fi(x)];
            % hess_fi_bar = [zeros(1,length(x)+1); zeros(length(x),1) hess_fi(x)];
            
        end

    end

    properties (Access = protected)
        
        Ai % linear inequality constraints matrix
        bi % linear inequality constraints vector

    end
    
end

