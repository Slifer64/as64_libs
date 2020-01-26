%% WSoG class
%  Weighted sum of Guassians.
%

classdef WSoG < matlab.mixin.Copyable

    methods (Access = public)
        %% Weighted Sum of Gaussians constructor.
        %  @param[in] N_kernels: The number of kernels.
        %  @param[in] kernel_std_scaling: Scaling of the kernel's std. (optional, default=1.0)
        function this = WSoG(N_kernels, kernel_std_scaling) %, N_kernels, s_gat_ptr

            if (nargin < 2), kernel_std_scaling = 1.0; end
            
            this.N_kernels = N_kernels;
            
            this.zero_tol = 1e-30; %realmin;

            this.w = zeros(this.N_kernels,1);
            this.c = ((1:this.N_kernels)-1)'/(this.N_kernels-1);
            % this.c = linspace(-0.04,1.04, N_kernels)';
            
            this.h = 1./(kernel_std_scaling*(this.c(2:end)-this.c(1:end-1))).^2;
            this.h = [this.h; this.h(end)];
            
            d = this.c(2) - this.c(1);
            c_end = this.c(end);
            extra = [d; 2*d; 3*d];
            this.c = [-extra+this.c(1); this.c; extra+c_end];
            this.h = [repmat(this.h(end),length(extra),1); this.h; repmat(this.h(end),length(extra),1)];
            
            this.N_kernels = this.N_kernels + 2*length(extra);
            this.w = zeros(this.N_kernels,1);
            
            this.sigma_eps = 1e-3;
            
            this.f0_d = 0;
            this.fg_d = 1;
            this.f0 = this.f0_d;
            this.setFinalValue(this.fg_d);
            
        end

        %% Returns the number of kernels.
        %  @return The number of kernels.
        function n_ker = numOfKernels(this)
            
            n_ker = length(this.w);
            
        end

        
        %% Sets the initial value.
        %  @param[in] f0: Initial value.
        function setStartValue(this, f0)
            
            this.f0 = f0;
            this.calcSpatialScaling();
            
        end
        
        %% Sets the final value.
        %  @param[in] fg: Final value.
        function setFinalValue(this, fg)
            
            this.fg = fg;
            this.calcSpatialScaling();
            
        end
        
        %% =============================================================
        
        
        %% Returns the normalized weighted sum of the Gaussians for the given phase variable (time instant).
        %  @param[in] x: The phase variable.
        %  @param[out] f: The normalized weighted sum of the Gaussians.
        %
        function f = output(this,x)

            Phi = this.regressVec(x);
            f = dot(Phi,this.w) - this.spat_s*this.f0_d + this.f0;
            
        end
        
        function f_dot = outputDot(this, x, dx)
            
            Phi_dot = this.regressVecDot(x,dx);
            f_dot = dot(Phi_dot,this.w);
            
        end
        
        function f_ddot = outputDDot(this, x, dx, ddx)
            
            Phi_ddot = this.regressVecDDot(x, dx, ddx);
            f_ddot = dot(Phi_ddot, this.w);
            
        end
        
        function f_3dot = output3Dot(this, x, dx, ddx, d3x)
            
            Phi_3dot = this.regressVec3Dot(x, dx, ddx, d3x);
            f_3dot = dot(Phi_3dot, this.w);
            
        end

        
        %% =============================================================
        
        %% Updates the weights to produce output 'p' at timestamp 'x'.
        %  @param[in] x: Timestamp.
        %  @param[in] p: Desired position at 'x'.
        %  @param[in] sigma_p: Variance of 'p' (optional, default=WSoG::sigma_eps).
        function updatePos(this, x, p, sigma_p)
            
            if (nargin < 4), sigma_p = this.sigma_eps; end
            
            H = this.regressVec(x)';
            p_hat = this.output(x);
            e = p - p_hat;
            
            this.updateWeights(H, e, sigma_p);
            
        end
        
        function updateVel(this, x, dx, v, sigma_v)
            
            if (nargin < 5), sigma_v = this.sigma_eps; end
            
            H = this.regressVecDot(x, dx)';
            v_hat = H*this.w;
            e = v - v_hat;
            
            this.updateWeights(H, e, sigma_v);
            
        end
        
        function updateAccel(this, x, dx, a, sigma_a)
            
            if (nargin < 5), sigma_a = this.sigma_eps; end
            
            H = this.regressVecDDot(x, dx, 0)';
            a_hat = H*this.w;
            e = a - a_hat;
            this.updateWeights(H, e, sigma_a);
            
        end
        
        function updatePosVel(this, x, dx, p, v, sigma_pv)
            
            if (nargin < 6), sigma_pv = this.sigma_eps*ones(2,1); end
            
            H = [this.regressVec(x)'; this.regressVecDot(x, dx)'];
            p_hat = this.output(x);
            v_hat = this.outputDot(x, dx);
            pv_hat = [p_hat; v_hat];
            e = [p; v] - pv_hat;
            this.updateWeights(H, e, diag(sigma_pv));
            
        end
        
        function updatePosAccel(this, x, dx, p, a, sigma_pa)
            
            if (nargin < 6), sigma_pa = this.sigma_eps*ones(2,1); end
            
            H = [this.regressVec(x)'; this.regressVecDDot(x, dx, 0)'];
            p_hat = this.output(x);
            a_hat = this.outputDDot(x, dx, 0);
            pa_hat = [p_hat; a_hat];
            e = [p; a] - pa_hat;
            this.updateWeights(H, e, diag(sigma_pa));
            
        end
        
        function updateVelAccel(this, x, dx, v, a, sigma_va)
            
            if (nargin < 6), sigma_va = this.sigma_eps*ones(2,1); end
            
            H = [this.regressVecDot(x, dx)'; this.regressVecDDot(x, dx, 0)'];
            va_hat = H*this.w;
            e = [v; a] - va_hat;
            this.updateWeights(H, e, diag(sigma_va));
            
        end
        
        function updatePosVelAccel(this, x, dx, p, v, a, sigma_pva)
            
            if (nargin < 7), sigma_pva = this.sigma_eps*ones(3,1); end
            
            Hp = this.regressVec(x)';
            Hv = this.regressVecDot(x, dx)';
            Ha = this.regressVecDDot(x, dx, 0)';
            H = [Hp; Hv; Ha];
            
            p_hat = this.output(x);
            v_hat = dot(Hv,this.w);
            a_hat = dot(Ha, this.w);
            pva_hat = [p_hat; v_hat; a_hat];
            
            e = [p; v; a] - pva_hat;
            this.updateWeights(H, e, diag(sigma_pva));
            
        end
        
        function updateWeights(this, H, e, Sigma_z)
            
            Sigma_w = eye(this.N_kernels, this.N_kernels);    
            K = Sigma_w*H' / (Sigma_z + H*Sigma_w*H');
            this.w = this.w + K*e;
            
        end
        
        %% =============================================================
        
        %% Trains the WSoG.
        %  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
        %  @param[in] x: Row vector with the timestamps of the training data points. Must take values in [0 1].
        %  @param[in] Fd: Row vector with the desired values.
        %  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
        function [train_error, F] = train(this, train_method, x, Fd)

            n_data = length(x);

            s = ones(1, n_data);
            Psi = zeros(this.N_kernels, n_data);
            for i=1:n_data
                Psi(:,i) = this.kernelFun(x(i));
            end

            if (train_method == GMP_TRAIN.LWR), this.w = LWR(Psi, s, Fd, this.zero_tol);
            elseif (train_method == GMP_TRAIN.LS), this.w = leastSquares(Psi, s, Fd, this.zero_tol);
            else, error('[WSoG::train]: Unsopported training method...');
            end

            if (nargout > 0)
                F = zeros(size(Fd));
                for i=1:size(F,2)
                    F(i) = this.output(x(i));
                end
                train_error = norm(F-Fd)/length(F);
            end
            
            this.f0_d = dot(this.regressVec(0),this.w);
            this.fg_d = dot(this.regressVec(1),this.w);
            this.setStartValue(this.f0_d);
            this.setFinalValue(this.fg_d);

        end
        
        
        function [train_error, F] = train_qp_constr(this, x, Fd, T, v, a)
            
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

        
        function [train_error, F] = train_qp_smooth(this, x, Fd, T, Wv, Wa)
            
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
            
            Hp = Phi'*Phi;
%             H1 = zeros(this.N_kernels, this.N_kernels);
%             H2 = zeros(this.N_kernels, this.N_kernels);
%             for i=1:N-1
%                d1i = Phi(i+1,:) - Phi(i,:);
%                H1 = H1 + d1i*d1i';
%                
%                d2i = Phi_dot(i+1,:) - Phi_dot(i,:);
%                H2 = H2 + d2i*d2i';
%             end
            
            Hv = Phi_dot'*Wv*Phi_dot; 
            % H2 = Phi_ddot'*Phi_ddot; 
            
            i1 = N;
            i2 = 1;
            for i=1:N
                if (x(i) > 0.03)
                    i1 = i;
                    break;
                end
            end
            for i=N:-1:i1
                if (x(i) < 0.97)
                    i2 = i;
                    break;
                end
            end
            if (~isscalar(Wa))
                Wa = Wa(i1:i2,i1:i2);
            end
            Ha = Phi_ddot(i1:i2,:)'*Wa*Phi_ddot(i1:i2,:); 
            
            H = Hp + Hv + Ha;
            f = -Fd*Phi;
         
            this.w = quadprog(H,f);
            
            % figure;
            % plot(x, Phi_ddot*this.w);
            
%             dF = Phi_dot*this.w;
%             abs_dF = abs(dF);
%             max_v = max(abs_dF);
%             min_v = min(abs_dF);
%             temp = 1 + (abs_dF-min_v)*(5 - 1)/(max_v-min_v);
%             Q1 = diag(1./temp);
%             
%             H1 = Phi_dot'*Q1*Phi_dot; 
%             H = H0 + a1*H1 + a2*H2;
%             this.w = quadprog(H,f);

            if (nargout > 0)
                F = zeros(size(Fd));
                for i=1:size(F,2)
                    F(i) = this.output(x(i));
                end
                train_error = norm(F-Fd)/length(F);
            end

        end
        
        
        function [train_error, F] = train_cvx(this, x, Pd, dPd, ddPd, T, w_v, w_a)
            
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
            
%             i1 = N;
%             i2 = 1;
%             for i=1:N
%                 if (x(i) > 0.03)
%                     i1 = i;
%                     break;
%                 end
%             end
%             for i=N:-1:i1
%                 if (x(i) < 0.97)
%                     i2 = i;
%                     break;
%                 end
%             end
%             if (~isscalar(Wa))
%                 Wa = Wa(i1:i2,i1:i2);
%             end
%             Ha = Phi_ddot(i1:i2,:)'*Wa*Phi_ddot(i1:i2,:); 
         
            cvx_begin
            
                variable w(this.N_kernels);
                expressions J1 J2 J3;
                
                J1 = norm(Phi*w-Pd', 2);
                J2 = norm(Phi_dot*w-dPd', 2);
                J3 = norm(Phi_ddot*w-ddPd', 2);
                
                minimize (J1 + w_v*J2 + w_a*J3)
            
            cvx_end
            
            this.w = w;

            if (nargout > 0)
                F = zeros(size(Pd));
                for i=1:size(F,2)
                    F(i) = this.output(x(i));
                end
                train_error = norm(F-Pd)/length(F);
            end

        end
        
        
        
        function [train_error, F] = train_cvx_constr(this, x, Fd, T, v, a)
            
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
                expressions J e
                
                e = Phi*w-Fd';
                J = e'*e;
                % J = norm(Phi*w-Fd', 2);
                % J = 0.5*w'*H*w + f*w;
                
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
                expressions J e
                
                e = Phi*w-Fd';
                J = e'*e;
                % J = norm(Phi*w-Fd', 2);
                % J = 0.5*w'*H*w + f*w;
                
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
        
        %% =============================================================
        
        function [P_data, dP_data, ddP_data, d3P_data] = simulate(this, x, dx, ddx, d3x)
            
            N = length(x);
            
            if (isscalar(dx)), dx = dx*ones(1,N); end
            if (isscalar(ddx)), ddx = ddx*ones(1,N); end
            if (isscalar(d3x)), d3x = d3x*ones(1,N); end
            
            P_data = zeros(1,N);
            dP_data = zeros(1,N);
            ddP_data = zeros(1,N);
            d3P_data = zeros(1,N);
            for i=1:N
               P_data(i) = this.output(x(i));
               dP_data(i) = this.outputDot(x(i), dx(i));
               ddP_data(i) = this.outputDDot(x(i), dx(i), ddx(i));
               d3P_data(i) = this.output3Dot(x(i), dx(i), ddx(i), d3x(i));
            end
            
        end

        
        %% =============================================================
        
        function phi = regressVec(this, x)
            
            psi = this.kernelFun(x);
            phi = this.spat_s * psi / (sum(psi) + this.zero_tol);

        end
        
        function phi_dot = regressVecDot(this, x, dx)
            
            psi = this.kernelFun(x);
            psi_dot = this.kernelFunDot(x, dx);
            sum_psi = sum(psi);
            sum_psi_dot = sum(psi_dot);
            
            % phi_dot = ( psi_dot*sum_psi - psi*sum_psi_dot ) / ( sum_psi^2 + this.zero_tol);
            
            phi = psi / ( sum(sum_psi) + this.zero_tol );
            phi_dot =  this.spat_s * ( psi_dot - phi*sum_psi_dot ) / ( sum_psi + this.zero_tol);

        end
        
        function phi_ddot = regressVecDDot(this, x, dx, ddx)
            
            psi = this.kernelFun(x);
            psi_dot = this.kernelFunDot(x, dx);
            psi_ddot = this.kernelFunDDot(x, dx, ddx);
            sum_psi = sum(psi);
            sum_psi_dot = sum(psi_dot);
            sum_psi_ddot = sum(psi_ddot);
            % sum_psi2 = sum_psi^2;
            
            % phi_ddot2 = ( ( psi_ddot*sum_psi - 2*psi_dot*sum_psi_dot - psi*sum_psi_ddot )*sum_psi2 - 2*psi*sum_psi*sum_psi_dot^2 ) / ( sum_psi2^2 + this.zero_tol);
            
            % phi_ddot2 = ( ( psi_ddot*sum_psi - psi*sum_psi_ddot )*sum_psi - 2*sum_psi_dot*(psi_dot*sum_psi-psi*sum_psi_dot) ) / ( sum_psi^3 + this.zero_tol);

            phi = psi / ( sum(sum_psi) + this.zero_tol );
            phi_dot = ( psi_dot - phi*sum_psi_dot ) / ( sum_psi + this.zero_tol);
            phi_ddot = this.spat_s * (psi_ddot - 2*phi_dot*sum_psi_dot - phi*sum_psi_ddot) / ( sum_psi + this.zero_tol);  

        end
        
        function phi_3dot = regressVec3Dot(this, x, dx, ddx, d3x)
            
            psi = this.kernelFun(x);
            psi_dot = this.kernelFunDot(x, dx);
            psi_ddot = this.kernelFunDDot(x, dx, ddx);
            psi_3dot = this.kernelFun3Dot(x, dx, ddx, d3x);
            sum_psi = sum(psi);
            sum_psi_dot = sum(psi_dot);
            sum_psi_ddot = sum(psi_ddot);
            sum_psi_3dot = sum(psi_3dot);

            phi = psi / ( sum(sum_psi) + this.zero_tol );
            phi_dot = ( psi_dot - phi*sum_psi_dot ) / ( sum_psi + this.zero_tol);
            phi_ddot = (psi_ddot - 2*phi_dot*sum_psi_dot - phi*sum_psi_ddot) / ( sum_psi + this.zero_tol);
            phi_3dot = this.spat_s * (psi_3dot - 3*phi_ddot*sum_psi_dot - 3*phi_dot*sum_psi_ddot - phi*sum_psi_3dot) / ( sum_psi + this.zero_tol);

        end

        %% ============================================================
        
        function f0_d = getStartDemoValue(this)
            
            f0_d = this.f0_d;
            
        end
        
        function fg_d = getFinalDemoValue(this)
            
            fg_d = this.fg_d;
            
        end
        
        function f0 = getStartValue(this)
            
            f0 = this.f0;
            
        end
        
        function fg = getFinalValue(this)
            
            fg = this.fg;
            
        end
        
        function s = getSpatialScaling(this)
           
            s = this.spat_s;
            
        end
        
        %% =============================================================
        
        function plotPsi(this, x)
            
            Psi = this.kernelFun(x);
            figure;
            hold on;
            for i=1:this.N_kernels
               plot(x, Psi(i,:), 'LineWidth',2); 
            end
            hold off

        end
        
    end
    
    methods (Access = private)
        
        function calcSpatialScaling(this)
            
            this.spat_s = (this.fg - this.f0) / (this.fg_d - this.f0_d);
            
        end
        
        %% =============================================================
        
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
        
        function psi_dot = kernelFunDot(this, x, dx)

            n = length(x);
            psi = this.kernelFun(x);
            psi_dot = zeros(this.N_kernels, n);
            
            for j=1:n
                a = (x(j)-this.c)*dx(j);
                psi_dot(:,j) = -2*this.h.*( psi(:,j).*a);
            end 

        end
        
        function psi_ddot = kernelFunDDot(this, x, dx, ddx)

            n = length(x);
            psi = this.kernelFun(x);
            psi_dot = this.kernelFunDot(x,dx);
            psi_ddot = zeros(this.N_kernels, n);

            for j=1:n
                a = (x(j)-this.c)*dx(j);
                a_dot = (x(j)-this.c)*ddx(j) + dx(j)^2;
                psi_ddot(:,j) = -2*this.h.*( psi_dot(:,j).*a + psi(:,j).*a_dot ); 
            end 

        end
        
        function psi_3dot = kernelFun3Dot(this, x, dx, ddx, d3x)

            n = length(x);
            psi = this.kernelFun(x);
            psi_dot = this.kernelFunDot(x,dx);
            psi_ddot = this.kernelFunDDot(x,dx, ddx);
            psi_3dot = zeros(this.N_kernels, n);
            
            for j=1:n
                a = (x(j)-this.c)*dx(j);
                a_dot = (x(j)-this.c)*ddx(j) + dx(j)^2;
                a_ddot = (x(j)-this.c)*d3x(j) + 3*dx(j)*ddx(j);
                psi_3dot(:,j) = -2*this.h.*( psi_ddot(:,j).*a + 2*psi_dot(:,j).*a_dot + psi(:,j).*a_ddot ); 
            end 

        end
        
        
    end
    
    properties (Access = private)
        
        N_kernels % number of kernels (basis functions)
        w % N_kernels x 1 vector with the kernels' weights
        c % N_kernels x 1 vector with the kernels' centers
        h % N_kernels x 1 vector with the kernels' inverse width
        
        zero_tol % small value used to avoid divisions with very small numbers
        
        sigma_eps % minumum noise variance for update of weights
        
        spat_s % spatial scaling
        
        f0_d % initial demo value
        fg_d % goal of demo
        f0 % initial value
        fg % goal value
        
    end
    
end
