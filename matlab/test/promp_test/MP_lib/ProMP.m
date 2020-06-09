%% ProMP class
%

classdef ProMP < MP
    
    methods (Access = public)
        
        function this = ProMP(N_kernels)
            
            this = this@MP(N_kernels);
            
            this.mu_w = zeros(this.N_kernels,1);
            
            this.setPhiBasedSigma();

        end
        
        function setPhiBasedSigma(this, sigma_scaling)
           
            if (nargin < 2), sigma_scaling=0.05; end
            
            this.Sigma_w = zeros(this.N_kernels, this.N_kernels);
            for j=1:this.N_kernels
               psi = this.kernelFun(this.c(j)); 
               this.Sigma_w(j,:) = psi';
               this.Sigma_w(:,j) = psi;
            end
            this.Sigma_w = sigma_scaling*this.Sigma_w/this.N_kernels;
            this.L_w = sqrt(this.Sigma_w);
            
        end

        function train(this, x, yd_data)

            n_data = length(x);
            Psi = zeros(this.N_kernels, n_data);
            for j=1:n_data, Psi(:,j) = this.regressVec(x(j)); end
            
            n_demos = size(yd_data, 1);
            w_data = zeros(this.N_kernels, n_demos);
            for i=1:n_demos
                w_data(:,i) = (yd_data(i,:)/Psi)';
            end
            
            this.mu_w = mean(w_data, 2);
            
            temp = w_data - repmat(this.mu_w, 1, n_demos);
            this.Sigma_w = temp*temp' / n_demos;
            
            if (this.N_kernels > n_demos)
               this.Sigma_w = this.Sigma_w + 1e-2*eye(this.N_kernels, this.N_kernels); 
            end
            
            this.L_w = sqrt(this.Sigma_w);
            
            this.w = this.mu_w;
            
        end
         
        function sampleWeights(this)
           
            this.w = this.mu_w + this.L_w*randn(this.N_kernels, 1);
            
        end
        
    end
    
    
    properties (Access = public)
        
        mu_w
        Sigma_w
        L_w
        
    end
    
    
end