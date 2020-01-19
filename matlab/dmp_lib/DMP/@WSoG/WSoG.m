%% WSoG class
%  Weighted sum of Guassians.
%

classdef WSoG < handle
       
    properties
        N_kernels % number of kernels (basis functions)

        w % N_kernels x 1 vector with the kernels' weights
        c % N_kernels x 1 vector with the kernels' centers
        h % N_kernels x 1 vector with the kernels' inverse width

        shapeAttrGatingFun % Pointer to gating function that ensures the output of WSoG decays to zero at the end (i.e. when the phase variable reaches 1.0)

        zero_tol % small value used to avoid divisions with very small numbers
        
        final_output % final output
        
    end

    methods
        %% Weighted Sum of Gaussians constructor.
        %  @param[in] N_kernels: The number of kernels.
        %  @param[in] shapeAttrGatingFun: Pointer to gating function that ensures the output of WSoG decays
        %                                 to zero at the end (i.e. when the phase variable reaches 1.0).
        %  @param[in] kernel_std_scaling: Scaling of the kernel's std. (optional, default=1.0)
        %
        function this = WSoG(N_kernels, shapeAttrGatingFun, kernel_std_scaling) %, N_kernels, s_gat_ptr

            if (nargin < 3), kernel_std_scaling = 1.0; end
            
            this.N_kernels = N_kernels;
            this.shapeAttrGatingFun = shapeAttrGatingFun;
            
            this.zero_tol = 1e-100; %realmin;

            this.w = zeros(this.N_kernels,1);
            this.c = ((1:this.N_kernels)-1)'/(this.N_kernels-1);
            % this.c = linspace(-0.04,1.04, N_kernels)';
            
            this.h = 1./(kernel_std_scaling*(this.c(2:end)-this.c(1:end-1))).^2;
            this.h = [this.h; this.h(end)];
            
        end

        %% Returns the number of kernels.
        %  @return The number of kernels.
        %
        function n_ker = getNumOfKernels(this)
            
            n_ker = length(this.w);
            
        end
        
        
        %% Trains the WSoG.
        %  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
        %  @param[in] x: Row vector with the timestamps of the training data points. Must take values in [0 1].
        %  @param[in] Fd: Row vector with the desired values.
        %  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
        %
        function [train_error, F] = train(this, train_method, x, Fd)
            
            this.final_output = Fd(end);
            % Fd = Fd - this.final_output;
            
            n_data = length(x);

            s = zeros(1, n_data);
            Psi = zeros(this.N_kernels, n_data);
            for i=1:n_data
                s(i) = this.shapeAttrGatingFun(x(i));
                Psi(:,i) = this.kernelFunction(x(i));
            end

            if (train_method == DMP_TRAIN.LWR), this.w = LWR(Psi, s, Fd, this.zero_tol);
            elseif (train_method == DMP_TRAIN.LS), this.w = leastSquares(Psi, s, Fd, this.zero_tol);
            else, error('[WSoG::train]: Unsopported training method...');
            end

            if (nargout > 0)
                F = zeros(size(Fd));
                for i=1:size(F,2)
                    F(i) = this.output(x(i));
                end
                train_error = norm(F-Fd)/length(F);
            end

        end
        
        
        function [train_error, F] = train_constr(this, x, Fd, T, v, a)
            
            this.final_output = Fd(end);
            % Fd = Fd - this.final_output;
            
            N = length(x);
            Phi = zeros(N, this.N_kernels);
            Phi_dot = zeros(N, this.N_kernels);
            Phi_ddot = zeros(N, this.N_kernels);
            for i=1:N
               Phi(i,:) = this.regressVec(x(i))';
               dx = 1/T;
               Phi_dot(i,:) = this.regressVecDot(x(i), dx)';
               ddx = 0;
               Phi_ddot(i,:) = this.regressVecDDot(x(i), dx, ddx)';
            end
            
            H = Phi'*Phi;
            f = -Fd*Phi;
            
            % =====================================
%             A = zeros(4*N, this.N_kernels);
%             A(1:N,:) = Phi_dot;
%             A(N+1:2*N,:) = -Phi_dot;
%             A(2*N+1:3*N,:) = Phi_ddot;
%             A(3*N+1:end,:) = -Phi_ddot;
%             
%             b = zeros(4*N, 1);
%             b(1:N) = v;
%             b(N+1:2*N) = v;
%             b(2*N+1:3*N) = a;
%             b(3*N+1:end) = a;
            
            % =====================================
            A = zeros(2*(2*N-1), this.N_kernels);
            A(1:N,:) = Phi_dot;
            A(N+1:2*N,:) = -Phi_dot;
            A(2*N+1:3*N-1,:) = Phi_dot(2:N,:)-Phi_dot(1:N-1,:);
            A(3*N:end,:) = -A(2*N+1:3*N-1,:);
            
            b = zeros(2*(2*N-1), 1);
            b(1:N) = v;
            b(N+1:2*N) = v;
            b(2*N+1:3*N-1) = a;
            b(3*N:end) = a;
            
            % =====================================
%             % Only velocity
%             A = zeros(2*N, this.N_kernels);
%             A(1:N,:) = Phi_dot;
%             A(N+1:2*N,:) = -Phi_dot;
%             
%             b = zeros(2*N, 1);
%             b(1:N) = v;
%             b(N+1:2*N) = v;
            
            tic 
            this.w = quadprog(H,f,A,b);
            toc
            
            if (nargout > 0)
                F = zeros(size(Fd));
                for i=1:size(F,2)
                    F(i) = this.output(x(i));
                end
                train_error = norm(F-Fd)/length(F);
            end

        end

        
        function [train_error, F] = train_smooth(this, x, Fd, T, a1, a2)
            
            this.final_output = Fd(end);
            % Fd = Fd - this.final_output;
            
            N = length(x);
            Phi = zeros(N, this.N_kernels);
            Phi_dot = zeros(N, this.N_kernels);
            for i=1:N
               Phi(i,:) = this.regressVec(x(i))';
               dx = 1/T;
               Phi_dot(i,:) = this.regressVecDot(x(i), dx)';
            end
            
            D1 = zeros(this.N_kernels, this.N_kernels);
            D2 = zeros(this.N_kernels, this.N_kernels);
            for i=1:N-1
               d1i = Phi(i+1,:) - Phi(i,:);
               D1 = D1 + d1i*d1i';
               
               d2i = Phi_dot(i+1,:) - Phi_dot(i,:);
               D2 = D2 + d2i*d2i';
            end
            
            H = Phi'*Phi + a1*D1 + a2*D2;
            f = -Fd*Phi;
                        
            this.w = quadprog(H,f);

            if (nargout > 0)
                F = zeros(size(Fd));
                for i=1:size(F,2)
                    F(i) = this.output(x(i));
                end
                train_error = norm(F-Fd)/length(F);
            end

        end
        
        
        function [train_error, F] = train_cvx(this, x, Fd, T, v, a)
            
            this.final_output = Fd(end);
            % Fd = Fd - this.final_output;
            
            N = length(x);
            Phi = zeros(N, this.N_kernels);
            Phi_dot = zeros(N, this.N_kernels);
            for i=1:N
               Phi(i,:) = this.regressVec(x(i))';
               dx = 1/T;
               Phi_dot(i,:) = this.regressVecDot(x(i), dx)';
            end

            H = Phi'*Phi;
            f = -Fd*Phi;
            
            % =====================================
            A1 = Phi_dot;
            A2 = Phi_dot(2:N,:)-Phi_dot(1:N-1,:);
            b1 = v;
            b2 = a;
            
            tic
            cvx_begin
                
                variable w(this.N_kernels);
                expression J
                
                % J = norm(Phi*w-Fd', 2);
                J = 0.5*w'*H*w + f*w;
                
                minimize(J);
                subject to
                    norm(A1*w, Inf) <= b1
                    norm(A2*w, Inf) <= b2
                           
            cvx_end
            toc
            
            this.w = w;

            if (nargout > 0)
                F = zeros(size(Fd));
                for i=1:size(F,2)
                    F(i) = this.output(x(i));
                end
                train_error = norm(F-Fd)/length(F);
            end

        end
        
        function [train_error, F] = train_cvx2(this, x, Fd, T, v, a)
            
            this.final_output = Fd(end);
            % Fd = Fd - this.final_output;
            
            N = length(x);
            Phi = zeros(N, this.N_kernels);
            Phi_dot = zeros(N, this.N_kernels);
            for i=1:N
               Phi(i,:) = this.regressVec(x(i))';
               dx = 1/T;
               Phi_dot(i,:) = this.regressVecDot(x(i), dx)';
            end
            
            H = Phi'*Phi;
            f = -Fd*Phi;

            A = zeros(2*(2*N-1), this.N_kernels);
            A(1:N,:) = Phi_dot;
            A(N+1:2*N,:) = -Phi_dot;
            A(2*N+1:3*N-1,:) = Phi_dot(2:N,:)-Phi_dot(1:N-1,:);
            A(3*N:end,:) = -A(2*N+1:3*N-1,:);
            
            b = zeros(2*(2*N-1), 1);
            b(1:N) = v;
            b(N+1:2*N) = v;
            b(2*N+1:3*N-1) = a;
            b(3*N:end) = a;

            % =====================================
            tic
            cvx_begin
                
                variable w(this.N_kernels);
                expression J
                
                J = 0.5*w'*H*w + f*w;
                
                minimize(J);
                subject to
                    A*w <= b
                           
            cvx_end
            toc
            
            this.w = w;

            if (nargout > 0)
                F = zeros(size(Fd));
                for i=1:size(F,2)
                    F(i) = this.output(x(i));
                end
                train_error = norm(F-Fd)/length(F);
            end

        end
        
        
        %% Returns a column vector with the values of the kernel functions.
        %  @param[in] x: The phase variable.
        %  @return: Column vector with the values of the kernel functions.
        %
        function psi = kernelFunction(this, x)

            n = length(x);
            psi = zeros(this.N_kernels, n);
            
            for j=1:n
                psi(:,j) = exp(-this.h.*((x(j)-this.c).^2));
            end 

        end
        
        function phi = regressVec(this, x)
            
            psi = this.kernelFunction(x);
            phi = psi / (sum(psi) + this.zero_tol);

        end
        
        function phi_dot = regressVecDot(this, x, dx)
            
            psi = this.kernelFunction(x);
            psi_dot = this.kernelFunctionDot(x, dx);
            sum_psi = sum(psi);
            sum_psi_dot = sum(psi_dot);
            
            phi_dot = ( psi_dot*sum_psi - psi*sum_psi_dot ) / ( sum_psi^2 + this.zero_tol);

        end
        
        function phi_ddot = regressVecDDot(this, x, dx, ddx)
            
            psi = this.kernelFunction(x);
            psi_dot = this.kernelFunctionDot(x, dx);
            psi_ddot = this.kernelFunctionDDot(x, dx, ddx);
            sum_psi = sum(psi);
            sum_psi_dot = sum(psi_dot);
            sum_psi_ddot = sum(psi_ddot);
            sum_psi2 = sum_psi^2;
            
            phi_ddot = ( ( psi_ddot*sum_psi - 2*psi_dot*sum_psi_dot - psi*sum_psi_ddot )*sum_psi2 - 2*psi*sum_psi*sum_psi_dot^2 ) / ( sum_psi2^2 + this.zero_tol);
            

        end
        
        function psi_dot = kernelFunctionDot(this, x, dx)

            n = length(x);
            psi = this.kernelFunction(x);
            psi_dot = zeros(this.N_kernels, n);
            
            for j=1:n
                psi_dot(:,j) = -2*this.h.*((x(j)-this.c)).*psi(:,j) * dx(j);
            end 

        end
        
        function psi_ddot = kernelFunctionDDot(this, x, dx, ddx)

            n = length(x);
            psi = this.kernelFunction(x);
            psi_dot = this.kernelFunctionDot(x,dx);
            psi_ddot = zeros(this.N_kernels, n);
            
            for j=1:n
                psi_ddot(:,j) = -2*this.h.*( (x(j)-this.c).*psi(:,j)*ddx(j) + psi(:,j)*dx(j)^2 + (x(j)-this.c).*psi_dot(:,j)*dx(j) ); 
            end 

        end
         
        
        %% Returns the normalized weighted sum of the Gaussians for the given phase variable (time instant).
        %  @param[in] x: The phase variable.
        %  @param[out] f: The normalized weighted sum of the Gaussians.
        %
        function f = output(this,x)
            
%             Psi = this.kernelFunction(x);         
%             f = this.shapeAttrGatingFun(x) * dot(Psi,this.w) / (sum(Psi) + this.zero_tol); % add 'zero_tol' to avoid numerical issues
            
            Phi = this.regressVec(x);
            f = this.shapeAttrGatingFun(x) * dot(Phi,this.w);
            % f = f + this.final_output;
            
        end
        
        function f_dot = outputDot(this, x, dx)
            
%             Psi = this.kernelFunction(x);
%             Psi_dot = this.kernelFunctionDot(x, dx);
%             sum_Psi = sum(Psi);
%             sum_Psi_dot = sum(Psi_dot);
%             
%             Phi = ( Psi_dot*sum_Psi - Psi*sum_Psi_dot ) / ( sum_Psi^2 + this.zero_tol);
%             
%             f_dot = this.shapeAttrGatingFun(x) * dot(Phi,this.w);
            
            
            Phi_dot = this.regressVecDot(x,dx);
            f_dot = this.shapeAttrGatingFun(x) * dot(Phi_dot,this.w);
            
        end
        
        function f_ddot = outputDDot(this, x, dx, ddx)
            
%             Psi = this.kernelFunction(x);
%             Psi_dot = this.kernelFunctionDot(x, dx);
%             Psi_ddot = this.kernelFunctionDDot(x, dx, ddx);
%             sum_Psi = sum(Psi);
%             sum_Psi_dot = sum(Psi_dot);
%             sum_Psi_ddot = sum(Psi_ddot);
%             sum_Psi2 = sum_Psi^2;
%             
%             Phi = ( ( Psi_ddot*sum_Psi - 2*Psi_dot*sum_Psi_dot - Psi*sum_Psi_ddot )*sum_Psi2 - 2*Psi*sum_Psi*sum_Psi_dot^2 ) / ( sum_Psi2^2 + this.zero_tol);
%             
%             f_ddot = this.shapeAttrGatingFun(x) * dot(Phi,this.w);
            
            Phi_ddot = this.regressVecDDot(x,dx,ddx);
            f_ddot = this.shapeAttrGatingFun(x) * dot(Phi_ddot,this.w);
            
        end

    end
end
