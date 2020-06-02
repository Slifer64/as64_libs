%% ProMP class
%

classdef ProMP < handle
    
    methods (Access = public)
        
        function this = ProMP(N_kernels)
            
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
            

            this.mu_w = this.w;
            this.Sigma_w = zeros(this.N_kernels, this.N_kernels);
            for j=1:this.N_kernels
               psi = this.kernelFun(this.c(j)); 
               this.Sigma_w(j,:) = psi';
               this.Sigma_w(:,j) = psi;
            end
            this.Sigma_w = 0.05*this.Sigma_w/this.N_kernels;
            this.L_w = sqrt(this.Sigma_w);
            
            
            if (nargout > 0)
                y = zeros(size(yd));
                for j=1:n_data, y(j) = this.output(x(j)); end
                train_err = norm(y-yd)/n_data; 
            end
            
        end
        
        
        function sampleWeights(this)
           
            this.w = this.mu_w + this.L_w*randn(this.N_kernels, 1);
            
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
        
        mu_w
        Sigma_w
        L_w
        
        zero_tol = 1e-32;
        
    end
    
    
end