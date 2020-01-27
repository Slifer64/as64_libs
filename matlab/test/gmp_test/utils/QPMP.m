%% Quadratic Optimization Motion Primitive class
%  y_ddot(t) = Phi(x) * w
%  y_dot(t) = y_dot(t0) + int_{t0}_{t}(Phi(x) * w)
%  y(t) = y(t0) + y_dot(t0)(t-t0) + int_{t0}_{t}(int_{t0}_{t}(Phi(x) * w))

classdef QPMP < matlab.mixin.Copyable

    methods (Access = public)
        %% Weighted Sum of Gaussians constructor.
        %  @param[in] N_kernels: The number of kernels.
        %  @param[in] yd_ddot: Training data acceleration.
        %  @param[in] taud: Training data time duration.
        %  @param[in] yd0: Training data initial position.
        %  @param[in] gd: Training data goal position.
        %  @param[in] kernel_std_scaling: Scaling of the kernel's std. (optional, default=1.0)
        function this = QPMP(N_kernels, yd_ddot, taud, yd0, gd, kernel_std_scaling)

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
            
            this.yd_ddot = yd_ddot;
            this.taud = taud;
            this.yd0 = yd0;
            this.gd = gd;
            
        end

        %% Returns the number of kernels.
        %  @return The number of kernels.
        function n_ker = numOfKernels(this)
            
            n_ker = length(this.w);
            
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
        
        %% Trains the WSoG.
        %  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
        %  @param[in] x: Row vector with the timestamps of the training data points. Must take values in [0 1].
        %  @param[in] Fd: Row vector with the desired values.
        %  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
        function train(this, tau, y0, g)
 
            this.tau = tau;
            
            spat_s = (g - y0) / (this.gd - this.yd0);
            temp_s = this.taud / tau;
            
            y_ddot = spat_s * temp_s^2 * this.yd_ddot;
            
            

        end
        
        function init(this, y0, y_dot0)
           
            this.yd = y0;
            this.yd_dot = y_dot0;
            
        end
        
        function y_ddot = getAccel(this, y, y_dot, dt)
           
            this.t = this.t + dt;
            x = this.t / this.tau;
            
            this.yd = this.yd + this.yd_dot*dt;
            this.yd_dot = this.yd_dot + this.yd_ddot*dt;
            this.yd_ddot = dot(regressVec(this, x), this.w);
            
            y_ddot = this.az*this.bz*(this.yd - y) + this.az*(this.yd_dot - y_dot) + this.yd_ddot;
            
        end
        
        function [Time, y, y_dot, y_ddot] = trajGen(this, y0, y_dot0, dt)
            
            N = length(this.yd_ddot);
            
            this.init(y0, y_dot0);
            
            y = zeros(1,N);
            y_dot = zeros(1,N);
            y_ddot = zeros(1,N);
            
            for i=1:N-1
                y_ddot(i) = this.getAccel(y(i), y_dot(i), dt);
                y(i+1) = y(i) + y_dot(i)*dt;
                y_dot(i+1) = y_dot(i) + y_ddot(i)*dt;
            end
            
            Time = dt*(0:(N-1))/(N-1);
            
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
            phi = psi / (sum(psi) + this.zero_tol);

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

        %% =============================================================

    end
    
    methods (Access = private)
        
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
        
        yd_ddot; % training data acceleration
        taud % training data time duration
        yd0
        gd
        
    end
    
end
