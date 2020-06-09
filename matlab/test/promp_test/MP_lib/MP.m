%% ProMP class
%

classdef MP < handle
    
    methods (Access = public)
        
        function this = MP(N_kernels)
            
            this.N_kernels = N_kernels;
            
            this.c = ((0:(N_kernels-1))/(N_kernels-1))';
            this.h = 1 / ( this.c(2) - this.c(1) )^2;
            this.w = zeros(this.N_kernels,1);      

        end
        
        function train_err = train(this, x, yd)
           
            n_data = length(x);
            Psi = zeros(this.N_kernels, n_data);
            for j=1:n_data, Psi(:,j) = this.regressVec(x(j)); end
            this.w = (yd/Psi)';

            if (nargout > 0)
                y = zeros(size(yd));
                for j=1:n_data, y(j) = this.output(x(j)); end
                train_err = norm(y-yd)/n_data; 
            end
            
        end

        function psi = kernelFun(this, x)
            
            psi = exp(-this.h*(x-this.c).^2);
            
        end
        
        function phi = regressVec(this, x)
            
            psi = this.kernelFun(x);
            phi = psi / (sum(psi) + this.zero_tol);
    
        end
        
        function y = output(this, x)
            
            y = dot(this.w, this.regressVec(x));
            
        end
        
    end
    
    
    properties (Access = public)
    
        N_kernels
        w
        c
        h
        
        zero_tol = 1e-32;
        
    end
    
    
end