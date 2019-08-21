clc;
close all;
clear;

set_matlab_utils_path();

rng(0);

% n = 100;
% m = 50;
% k = 10;
% A = rand(m,n);
% H = eye(n,n) + A'*A;
% B = rand(n,k);
% 
% lowTriangSolve = dsp.LowerTriangularSolver;
% upperTriangSolve = dsp.UpperTriangularSolver;
% 
% L = chol(H,'lower');
% X = upperTriangSolve(L', lowTriangSolve(L,B));
% X2 = H \ B;
% 
% norm(X(:)-X2(:))
% 
% return

% %% ===========  Linear minus sum of log of affine inequalities  =================
% n = 50;
% m = 100;
% obj = LinearMinusSumLogAffIneqObj(n,m);
% x0 = obj.genPointInDom();

%% ===========  Quadratic Objective  =================
n = 2;
x0 = 1*rand(n,1);
P = diag([1 10]);
obj = QuadraticObj(n, P);
% obj = QuadraticObj(n);


% %% ===========  Sum of exponentials of affine functions  =================
% n = 2;
% x0 = 2*rand(n,1);
% obj = SumExpAffObj();


%% ============================================
%% ============================================

%% ===========  Solve Method  =================
% opt_obj = GradientDescent(@obj.fun, @obj.gradFun);
% eps = 1e-3;
% iters = 200;

opt_obj = NewtonDescent(@obj.fun, @obj.gradFun, @obj.hessianFun);
eps = 1e-5;
iters = 100;

[x, x_data] = opt_obj.solve(x0, eps, iters);

p_start_hat = obj.fun(x)

%% ============================================
%% =============  Plot results ================

iterNum = 0:(size(x_data,2)-1);

J = zeros(1,size(x_data,2));
dJ = zeros(size(J));
for i=1:length(J)
    J(i) = obj.fun(x_data(:,i));
    dJ(i) = norm(obj.gradFun(x_data(:,i)));
end
    
figure;
subplot(2,1,1);
plot(iterNum, J, 'LineWidth',2);
ylabel('$J$', 'interpreter','latex', 'fontsize',18);
axis tight;
subplot(2,1,2);
plot(iterNum, dJ, 'LineWidth',2);
ylabel('$||\nabla J||_2$', 'interpreter','latex', 'fontsize',18);
xlabel('iter \#', 'interpreter','latex', 'fontsize',18);
axis tight;

% Contour plot
if (length(x) == 2)
    x1 = linspace(-2.0, 2.0, 200);
    x2 = linspace(-2.0, 2.0, 200);
    [X1,X2] = meshgrid(x1,x2);
    Z = zeros(size(X1));
    for i=1:size(Z,1)
        for j=1:size(Z,2), Z(i,j) = obj.fun([X1(i,j); X2(i,j)]); end
    end
    figure;
    hold on;
    contour(X1,X2,Z, (0:50).^2);
    scatter(x_data(1,:),x_data(2,:), 'Marker','*', 'MarkerFaceColor','magenta');
    plot(x_data(1,:),x_data(2,:), 'Color','red');
    hold off;
end


function 


function P = genRandSPDmatrix(n)
            
    L = zeros(n,n);
    for i=1:n, L(i,1:i) = rand(1,i); end
    for i=1:n
       L(i,i) = abs(L(i,i));
       if (L(i,i) < 0.01), L(i,i) = 0.01; end
    end
    P = L*L';

end

