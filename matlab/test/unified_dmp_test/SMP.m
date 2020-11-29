%% Class: Stochastic Movement Primitive

classdef SMP < MP

    methods (Access = public)
        %% Weighted Sum of Gaussians constructor.
        %  @param[in] N_kernels: The number of kernels.
        %  @param[in] kernel_std_scaling: Scaling of the kernel's std. (optional, default=1.0)
        function this = SMP(N_kernels, kernel_std_scaling)

            this = this@MP(N_kernels, kernel_std_scaling);
            
            n = this.N_kernels;
            S = zeros(n,n);
            for i=1:n
                for j=i+1:n
                    S(i,j) = exp(-0.8 * abs(i-j));
                    %S(i,j) = 1 - 1*(abs(i-j)/n)^3;
                end
            end
            S = S + S' + eye(n,n);
            
            this.Sigma_w = S;
            
        end

        %% =============================================================
        
        function train(this, x, y_o, Y_data) % override
 
            n = this.numOfKernels();

            % learn the nominal weights w_o based on the nominal trajectory y_o
            n_data = length(x);
            H = zeros(n_data, n);
            for i=1:n_data, H(i,:) = this.regressVec(x(i))'; end
            inv_H = H \ eye(n_data, n_data);
            w_o = inv_H*y_o';
            this.w = w_o;
            
            if (nargin < 4), return; end

            ind = round([0.1:0.1:1]*n_data); % update points indices (maybe add it to input arguments?)
            N = length(ind); % number of update points

            m = size(Y_data,1); % number of demos (or columns in each ||.||^2 of J)
            W_data = zeros(n, m);
            Z_data = zeros(N, m);
            for i=1:m
                y_k = Y_data(i,:); 
                W_data(:,i) = inv_H*y_k';
                Z_data(:,i) = y_k(ind)'; % or recalculate z based on learned w to be consistent?
            end
            
            % could also use cell array for H if each update point has dim>1
            H = H(ind,:); % get the regressor vectors of the update points
            y_o = H*w_o; % recalculate y_o at the update points using w_o (instead of y_o(ind)) to be consistent with what we have learned
            Zo_data = repmat(y_o, 1, m);
            
            W_err = W_data - repmat(w_o, 1, m);
            Z_err = cell(N,1);
            for i=1:N, Z_err{i} = Z_data(i,:) - Zo_data(i,:); end
            
            R = 1e-5;

            Sw0 = this.Sigma_w;
            L0 = chol(Sw0, 'lower');
            x0 = this.mat2params(L0);
            options = optimoptions('fminunc', 'Algorithm','trust-region', 'SpecifyObjectiveGradient',true);
            tic
            [x, fval] = fminunc(@(x)SMP.objFun(x, W_err, Z_err, H, R), x0, options);
            fval
            toc
            L = this.params2mat(x,n);
            this.Sigma_w = L*L';
            
            figure;
            subplot(2,1,1);
            imshow(Sw0, 'InitialMagnification', 2000);
            subplot(2,1,2);
            imshow(this.Sigma_w, [min(this.Sigma_w(:)) max(this.Sigma_w(:))], 'InitialMagnification', 2000);
            
        end
        
        function plotWeightsCovariance(this, ax)
            
            if (nargin < 2)
                figure;
                ax = axes();
            end
            imshow(this.Sigma_w, [min(this.Sigma_w(:)) max(this.Sigma_w(:))], 'InitialMagnification', 2000, 'Parent',ax);
            
        end
              
        %% =============================================================
        
        function updatePos(this, x, z, Rz)
            
            if (nargin < 4), Rz = this.sigma_R*eye(length(z),length(z)); end
            
            N = length(x);
            n = length(this.w);
            H = zeros(N,n);
            for i=1:N, H(i,:) = this.regressVec(x(i))'; end
            z_hat = H*this.w;
            this.w = this.w + this.Sigma_w*H' / (Rz + H*this.Sigma_w*H') * (z - z_hat);
            
        end
        
        
    end
    
    %% =================  Methods: private  =================
    methods (Access = private, Static)
        
        function [J, Jx] = objFun(x, W_err, Y_err, H, R)
            
            %n = length(this.w);
            n = ( -1 + sqrt(1+8*length(x)) )/2;
            
            N = size(H,1);

            L = SMP.params2mat(x,n);
            Sw = L*L';
            
            %i_diag = 1:n+1:n^2;
            %dL_dai = -diag(L).^2;
            
            J = 0;
            Jx_mat = zeros(n,n);
            
            for k=1:N
                H_k = H(k,:);
                A = H_k'/(R + H_k*Sw*H_k');
                U = A*Y_err{k};
                Y = (W_err - Sw*U);

                J = J + sum(sum(Y.^2));
                %J = trace(Y*Y')

                if (nargout > 1)
                    V = 2*Y'*(Sw*A*H_k-eye(n,n));
                    B = U*V;
                    Jx_mat = Jx_mat + (B'*L + B*L);
                end
                %Jx_mat(i_diag) = diag(Jx_mat).*dL_dai;
                
            end
            
            if (nargout > 1), Jx = SMP.mat2params(Jx_mat); end

        end
        
        function L = params2mat(x, n)
            
            L = zeros(n,n);
            
            i1 = 1;
            for j=1:n
               i2 = i1 + n-j; 
               L(j:n,j) = x(i1:i2);
               i1 = i2+1;
            end
            
            %L(1:n+1:n^2) = 1./diag(L);
            
        end
        
        function x = mat2params(L)
            
            n = size(L,1);
            %L(1:n+1:n^2) = 1./diag(L);
            
            N = (n+1)*n/2;
            x = zeros(N,1);
            i1 = 1;
            for j=1:n
               i2 = i1 + n-j; 
               x(i1:i2) = L(j:n,j);
               i1 = i2+1;
            end
            
        end
        
        
    end
    
    %% =================  Properties: private  =================
    properties (Access = public)
        
        Sigma_w
        
    end
    
    properties (Constant, Access = private)
        
        sigma_R = 1e-5 % default noise variance for update of weights
        
    end
    
end
