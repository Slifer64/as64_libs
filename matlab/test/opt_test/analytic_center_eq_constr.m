clc;
close all;
clear;

set_matlab_utils_path();

rng(0);

n = 100;
m = 25;

P = genRandSPDmatrix(n);
A = P(1:m,:);
x = 2*(rand(n,1) + 1.1);
b = A*x;

%% ============================================
%% ============================================

%% ===========  Solve Method  =================
% opt_obj = GradientDescent(@fun, @gradFun);
% eps = 1e-3;
% iters = 200;

x0 = A \ b;

solver = NewtonDescent(@fun, @gradFun, @hessianFun);
solver.setStopThreshold(1e-7);
solver.setMaxIters(100);
% [x, x_data] = solver.solve(x0);

tic
[x, w, x_data] = solver.solveEq(x0, A, b, KKTSolveMethod.FULL_INV);
fprintf('=====================================\n');
toc
fprintf('Solve eq - FULL_INV: p_star = %f\n',fun(x));
fprintf('=====================================\n');

tic
[x, w, x_data] = solver.solveEq(x0, A, b, KKTSolveMethod.BLOCK_ELIM);
fprintf('=====================================\n');
toc
fprintf('Solve eq - BLOCK_ELIM: p_star = %f\n',fun(x));
fprintf('=====================================\n');

tic
[x, w, x_data] = solver.solveEq(x0, A, b, KKTSolveMethod.EQ_ELIM);
fprintf('=====================================\n');
toc
fprintf('Solve eq - EQ_ELIM: p_star = %f\n',fun(x));
fprintf('=====================================\n');

x0 = x0 + 100*rand(n,1);
tic
[x, w, x_data] = solver.solveEqInfeasStart(x0, A, b, KKTSolveMethod.FULL_INV);
fprintf('=====================================\n');
toc
fprintf('Solve eq - Infeas start - FULL_INV: p_star = %f\n',fun(x));
fprintf('=====================================\n');

x0 = x0 + 100*rand(n,1);
tic
[x, w, x_data] = solver.solveEqInfeasStart(x0, A, b, KKTSolveMethod.BLOCK_ELIM);
fprintf('=====================================\n');
toc
fprintf('Solve eq - Infeas start - BLOCK_ELIM: p_star = %f\n',fun(x));
fprintf('=====================================\n');

tic
options = optimoptions(@fmincon, 'Algorithm','interior-point', 'MaxIterations',100, 'SpecifyObjectiveGradient',true);
x2 = fmincon(@fun,x0,[],[],A,b, [],[], [], options);
fprintf('=====================================\n');
toc
fprintf('fmincon: p_star = %f\n',fun(x2));
fprintf('=====================================\n');

%% ============================================
%% =============  Plot results ================

iterNum = 0:(size(x_data,2)-1);

J = zeros(1,size(x_data,2));
dJ = zeros(size(J));
for i=1:length(J)
    J(i) = fun(x_data(:,i));
    dJ(i) = norm(gradFun(x_data(:,i)));
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
        for j=1:size(Z,2), Z(i,j) = fun([X1(i,j); X2(i,j)]); end
    end
    figure;
    hold on;
    contour(X1,X2,Z, (0:50).^2);
    scatter(x_data(1,:),x_data(2,:), 'Marker','*', 'MarkerFaceColor','magenta');
    plot(x_data(1,:),x_data(2,:), 'Color','red');
    hold off;
end


function [f, df] = fun(x)

    f = -sum(log(x+100));

    if (nargout > 1), df = gradFun(x); end
end

function g = gradFun(x)

    g = -1./(x+100);

end

function H = hessianFun(x)

    H = diag(1./(x+100).^2);

end

function P = genRandSPDmatrix(n)
    
    A = rand(n,n);
    P = eye(n,n) + A'*A;
     
%     L = zeros(n,n);
%     for i=1:n, L(i,1:i) = rand(1,i); end
%     for i=1:n
%        L(i,i) = abs(L(i,i));
%        if (L(i,i) < 0.1), L(i,i) = 0.1; end
%     end
%     P = L*L';

end

