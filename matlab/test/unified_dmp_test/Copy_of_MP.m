%% Class

classdef MP < matlab.mixin.Copyable

    methods (Access = public)
        %% Weighted Sum of Gaussians constructor.
        %  @param[in] N_kernels: The number of kernels.
        %  @param[in] kernel_std_scaling: Scaling of the kernel's std. (optional, default=1.0)
        function this = MP(N_kernels, kernel_std_scaling)

            if (nargin < 2), kernel_std_scaling = 1.0; end
            
            this.N_kernels = N_kernels;

            this.w = zeros(this.N_kernels,1);
            this.c = ((1:this.N_kernels)-1)'/(this.N_kernels-1);
            
            this.h = 1./(kernel_std_scaling*(this.c(2:end)-this.c(1:end-1))).^2;
            this.h = [this.h; this.h(end)];
            
            n = this.N_kernels;
            S = zeros(n,n);
            for i=1:n
                for j=i+1:n
                    S(i,j) = exp(-0.01 * abs(i-j));
                    %S(i,j) = 1 - 1*(abs(i-j)/n)^3;
                end
            end
            S = S + S' + eye(n,n);
            
            this.Sigma_w = S;
            % imshow(this.Sigma_w, 'InitialMagnification', 800);
            
        end

        
        %% Returns the number of kernels.
        %  @return The number of kernels.
        function n_ker = numOfKernels(this)
            
            n_ker = length(this.w);
            
        end
        

        %% =============================================================
        
        %% Train model.
        %  @param[in] train_method: the training method to use, as a string ('LWR', 'LS').
        %  @param[in] x: Row vector with the timestamps of the training data points. Must take values in [0 1].
        %  @param[in] Fd: Row vector with the desired values.
        %  @param[out] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
        function train_error = train(this, x, Fd)

            N = length(x);
            n = length(this.w);

            H = zeros(N,n);
            for i=1:N, H(i,:) = this.regressVec(x(i))'; end
            this.w = H\Fd';
            
            if (nargout > 0)
                F = zeros(size(Fd));
                for i=1:size(F,2)
                    F(i) = this.output(x(i));
                end
                train_error = norm(F-Fd)/length(F);
            end
            
        end
        
        function optWeightsCovariance(this)
            
            n = length(this.w);
            
            S_hat = eye(n, n);
            
            w_old = this.w;
            
            x = 0:0.005:1;
            N = length(x);
            H = zeros(N,n);
            for i=1:length(x), H(i,:) = this.regressVec(x(i)); end
            y_old = H*w_old;
            z_old = [y_old(1); y_old(end)];
            
            ks = [0.7];% 0.8 0.9 1 1.1 1.2 1.3 1.4 1.5 1.6];
            m = length(ks);
            
            W = zeros(n, m);
            z = zeros(2,m);
            
            for j=1:m
                y = ks(j)*(y_old-y_old(1)) + y_old(1);
                W(:,j) = H\y;
                z(:,j) = [y(1); y(end)];
            end
            
%             y = 1.5*(y_old-y_old(1)) + y_old(1);
%             W = repmat(H\y,1,N);
%             z = [y'; ones(1,N)*y(1)];
%             z_old = [y_old'; ones(1,N)*y_old(1)];
            
            W_err = W - repmat(w_old,1,m);
            z_err = z - repmat(z_old,1,m);
%             z_err = z - z_old;
%             H = [this.regressVec(x(1))'; this.regressVec(x(end))'];
%             R = 1e-5*eye(2,2);
            H = [this.regressVec(x(end))'];
            R = 1e-5;
            z_err = z_err(2,:);
            
%             Sw0 = eye(n,n);
            Sw0 = this.Sigma_w;
            L0 = chol(Sw0, 'lower');
            x0 = this.lowtriangmat2vec(L0);
            options = optimoptions('fminunc', 'Algorithm','trust-region', 'SpecifyObjectiveGradient',true);
            tic
            [x, fval] = fminunc(@(x)objFun(this, x, W_err, z_err, H, R), x0, options);
            fval
            toc
            L = this.vec2lowtriangmat(x,n);
            this.Sigma_w = L*L';
            
            figure;
            subplot(2,1,1);
            imshow(Sw0, 'InitialMagnification', 2000);
            subplot(2,1,2);
            imshow(this.Sigma_w, 'InitialMagnification', 2000);
            
        end
        
        function [J, Jx] = objFun(this, x, W_err, z_err, H, R)
            
            n = length(this.w);
            
            %Sw = reshape(x,n,n);
            L = this.vec2lowtriangmat(x,n);
            Sw = L*L';
            
            A = H'/(R + H*Sw*H');
            U = A*z_err;
            Y = (W_err - Sw*U);
            
            J = sum(sum(Y.^2));
            %J = trace(Y*Y')

            if (nargout > 1)
                V = 2*Y'*(Sw*A*H-eye(n,n));
                B = U*V;
                Jx = this.lowtriangmat2vec(B'*L + B*L);
                
%                 Jx = zeros(length(x),1);
%                 k = 1;
%                 C = zeros(n,n);
%                 m = size(U,2);
%                 for j=1:n
%                    for i=j:n
%                       C(i,:) = L(:,j)';
%                       Jx(k) = trace(V*(C+C')*U);
% %                       for l=1:m
% %                         Jx(k) = Jx(k) + V(l,:)*(C+C')*U(:,l);
% %                       end
%                       k = k + 1;
%                       C(i,:) = zeros(1,n);
%                    end
%                 end
                
%                 if (norm(Jx - Jx2) > 1e-15), display(norm(Jx - Jx2)); end
                
            end

        end
        
        function L = vec2lowtriangmat(this, x, n)
            
            L = zeros(n,n);
            
            i1 = 1;
            for j=1:n
               i2 = i1 + n-j; 
               L(j:n,j) = x(i1:i2);
               i1 = i2+1;
            end
            
        end
        
        function x = lowtriangmat2vec(this, L)
            
            n = size(L,1);
            N = (n+1)*n/2;
            x = zeros(N,1);
            i1 = 1;
            for j=1:n
               i2 = i1 + n-j; 
               x(i1:i2) = L(j:n,j);
               i1 = i2+1;
            end
            
        end
        
        %% =============================================================
        
        %% Returns the scaled position produced by the model for a given phase variable value.
        %  If a new final (or initial) value has been set, then the
        %  produced position will be spatially scaled accordingly.
        %  @param[in] x: The phase variable (must be in [0 1]).
        %  @return (scaled) position.
        function f = output(this,x)

            Phi = this.regressVec(x);
            f = dot(Phi,this.w);
            
        end
        
        
        %% Returns the scaled velocity produced by the model for a given phase variable value.
        %  @param[in] x: The phase variable (must be in [0 1]).
        %  @param[in] x_dot: The phase variable 1st time derivative.
        %  @return (scaled) velocity.
        function f_dot = outputDot(this, x, x_dot)
            
            Phi_dot = this.regressVecDot(x, x_dot);
            f_dot = dot(Phi_dot,this.w);
            
        end
        
        
        %% Returns the scaled acceleration produced by the model for a given phase variable value.
        %  @param[in] x: The phase variable (must be in [0 1]).
        %  @param[in] x_dot: The phase variable 1st time derivative.
        %  @param[in] x_ddot: The phase variable 2nd time derivative.
        %  @return (scaled) acceleration.
        function f_ddot = outputDDot(this, x, x_dot, x_ddot)
            
            Phi_ddot = this.regressVecDDot(x, x_dot, x_ddot);
            f_ddot = dot(Phi_ddot, this.w);
            
        end

        
        function updatePos(this, x, z, Rz)
            
            if (nargin < 4), Rz = this.sigma_R*eye(length(z),length(z)); end
            
            N = length(x);
            n = length(this.w);
            H = zeros(N,n);
            for i=1:N, H(i,:) = this.regressVec(x(i))'; end
            z_hat = H*this.w;
            this.w = this.w + this.Sigma_w*H' / (Rz + H*this.Sigma_w*H') * (z - z_hat);
            
        end


        %% ============================================================
        
        %% Returns the scaled regressor vector ks*phi.
        %  @param[in] x: The phase variable (must be in [0 1]).
        %  @return (scaled) regressor vector.
        function phi = regressVec(this, x)
            
            psi = this.kernelFun(x);
            phi = psi / (sum(psi) + this.zero_tol);

        end
        
        %% Returns the scaled regressor vector 1st time derivative ks*phi_dot.
        %  @param[in] x: The phase variable (must be in [0 1]).
        %  @param[in] x_dot: The phase variable 1st time derivative.
        %  @return (scaled) regressor vector 1st time derivative.
        function phi_dot = regressVecDot(this, x, x_dot)
            
            psi = this.kernelFun(x);
            psi_dot = this.kernelFunDot(x, x_dot);
            sum_psi = sum(psi);
            sum_psi_dot = sum(psi_dot);

            phi = psi / ( sum(sum_psi) + this.zero_tol );
            phi_dot =  ( psi_dot - phi*sum_psi_dot ) / ( sum_psi + this.zero_tol);

        end
        
        %% Returns the scaled regressor vector 2nd time derivative ks*phi_ddot.
        %  @param[in] x: The phase variable (must be in [0 1]).
        %  @param[in] x_dot: The phase variable 1st time derivative.
        %  @param[in] x_ddot: The phase variable 2nd time derivative.
        %  @return (scaled) regressor vector 2nd time derivative.
        function phi_ddot = regressVecDDot(this, x, x_dot, x_ddot)
            
            psi = this.kernelFun(x);
            psi_dot = this.kernelFunDot(x, x_dot);
            psi_ddot = this.kernelFunDDot(x, x_dot, x_ddot);
            sum_psi = sum(psi);
            sum_psi_dot = sum(psi_dot);
            sum_psi_ddot = sum(psi_ddot);

            phi = psi / ( sum(sum_psi) + this.zero_tol );
            phi_dot = ( psi_dot - phi*sum_psi_dot ) / ( sum_psi + this.zero_tol);
            phi_ddot = (psi_ddot - 2*phi_dot*sum_psi_dot - phi*sum_psi_ddot) / ( sum_psi + this.zero_tol);  

        end

        
        %% =============================================================
        
        function [P_data, dP_data, ddP_data] = simulate(this, x, dx, ddx)
            
            N = length(x);
            
            if (isscalar(dx)), dx = dx*ones(1,N); end
            if (isscalar(ddx)), ddx = ddx*ones(1,N); end
            
            P_data = zeros(1,N);
            dP_data = zeros(1,N);
            ddP_data = zeros(1,N);
            for i=1:N
               P_data(i) = this.output(x(i));
               dP_data(i) = this.outputDot(x(i), dx(i));
               ddP_data(i) = this.outputDDot(x(i), dx(i), ddx(i));
            end
            
        end


        function ax = plotPsi(this, x, ax)
            
            if (nargin < 3), ax = axes(); end
            
            Psi = this.kernelFun(x);
            prev_hold = ax.NextPlot;
            hold(ax,'on');
            for i=1:this.N_kernels, plot(x, Psi(i,:), 'LineWidth',2, 'Parent',ax); end
            ax.NextPlot = prev_hold;

        end
        
        
    end
    
    %% =================  Methods: private  =================
    methods (Access = public) % private?  
        
        %% Returns a column vector with the values of the kernel functions.
        %  @param[in] x: The phase variable.
        %  @return: Column vector with the values of the kernel functions.
        function psi = kernelFun(this, x)

            n = length(x);
            psi = zeros(this.N_kernels, n);
            
            for j=1:n
                psi(:,j) = exp(-this.h.*((x(j)-this.c).^2));
            end 

        end
        
        function psi_dot = kernelFunDot(this, x, x_dot)

            n = length(x);
            psi = this.kernelFun(x);
            psi_dot = zeros(this.N_kernels, n);
            
            for j=1:n
                a = (x(j)-this.c)*x_dot(j);
                psi_dot(:,j) = -2*this.h.*( psi(:,j).*a);
            end 

        end
        
        function psi_ddot = kernelFunDDot(this, x, x_dot, x_ddot)

            n = length(x);
            psi = this.kernelFun(x);
            psi_dot = this.kernelFunDot(x,x_dot);
            psi_ddot = zeros(this.N_kernels, n);

            for j=1:n
                a = (x(j)-this.c)*x_dot(j);
                a_dot = (x(j)-this.c)*x_ddot(j) + x_dot(j)^2;
                psi_ddot(:,j) = -2*this.h.*( psi_dot(:,j).*a + psi(:,j).*a_dot ); 
            end 

        end
        
    end
    
    %% =================  Properties: private  =================
    properties (Access = public) %{?GMP})
        
        N_kernels % number of kernels (basis functions)
        w % N_kernels x 1 vector with the kernels' weights
        c % N_kernels x 1 vector with the kernels' centers
        h % N_kernels x 1 vector with the kernels' inverse width
        
        Sigma_w
        
    end
    
    properties (Constant, Access = private)
        
        zero_tol = 1e-200 % small value used to avoid divisions with very small numbers
        
        sigma_R = 1e-5 % default noise variance for update of weights
        
    end
    
end
