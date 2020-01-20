clc;
close all;
clear;

set_matlab_utils_path();

rng(0);

global P x_c;

n = 2;
m = 25;

% P = genRandSPDmatrix(n);
% x_c = rand(n,1);
P = [1.1 0.9; 0.3 1.5];
x_c = [0; 0];

a = -0.55;
x1 = -1;
y1 = 0;
b = y1 - a*x1;
Ai = [-a 1];
bi = b;

a = -1.1;
x1 = -1;
y1 = 0.5;
b = y1 - a*x1;
A = [-a 1];
b = b;
% A = [-0.9 -0.7];
% b = 1;

fi_y = @(x) (bi - Ai(:,1:end-1)*x)./Ai(:,end);
feq_y = @(x) (b - A(:,1:end-1)*x)./A(:,end);

% x0 = Ai\(0.5*bi);
x0 = [2; 2];

%% ============= Solve phase1 ==============
phase1 = Phase1Solver(LinIneqConstr(Ai,bi));
phase1.setEqConstr(A,b);
[x, s] = phase1.solve(x0);

%% ============  Solve  ================
options = optimoptions(@fmincon, 'Algorithm','interior-point', 'MaxIterations',100, 'SpecifyObjectiveGradient',true);
tic
x_fmincon = fmincon(@fun,x0,Ai,bi,A,b, [],[], [], options);
fprintf('=====================================\n');
toc
fprintf('fmincon: p_star = %f\n',fun(x_fmincon));
x_fmincon
fprintf('=====================================\n');

tic
H = P;
f = -2*P*x_c;
x_qp = quadprog(H,f,Ai,bi,A,b);
fprintf('=====================================\n');
toc
fprintf('quadprog: p_star = %f\n',fun(x_qp));
x_qp
fprintf('=====================================\n');

N_var = length(x0);
solver = NewtonDescent(N_var, @fun, @gradFun, @hessianFun);
solver.setStopThreshold(1e-4);
solver.setKKTSolveMethod(KKTSolver.BLOCK_ELIM);
solver.setMaxIters(150);
solver.setLinIneqConstr(Ai,bi);
solver.setEqConstr(A,b);
tic
% solver.setKKTsolveMethod(KKTSolveMethod.FULL_INV);
[x, v, x_data] = solver.solve(x0);
fprintf('=====================================\n');
toc
fprintf('Solve ineq: p_star = %f\n',fun(x));
x
fprintf('=====================================\n');


%% ============  Plot  =================

x1_max = 3;
x2_max = 3;
x1 = linspace(-x1_max, x1_max, 200);
x2 = linspace(-x2_max, x2_max, 200);
[X1,X2] = meshgrid(x1,x2);
Z = zeros(size(X1));
for i=1:size(Z,1)
    for j=1:size(Z,2), Z(i,j) = fun([X1(i,j); X2(i,j)]); end
end
figure;
ax = axes();
hold(ax,'on');
contour(X1,X2,Z, (0:0.3:5).^2);
xlim = ax.XLim;
ylim = ax.YLim;
plot([-x1_max x1_max], [fi_y(-x1_max) fi_y(x1_max)], 'LineWidth',2, 'LineStyle','--', 'Color','magenta');
plot([-x1_max x1_max], [feq_y(-x1_max) feq_y(x1_max)], 'LineWidth',2, 'LineStyle','--', 'Color',[0 0.7 0]);
% scatter(x_data(1,:),x_data(2,:), 'Marker','*', 'MarkerFaceColor','magenta');
% plot(x_data(1,:),x_data(2,:), 'Color','red');
ax.XLim = xlim;
ax.YLim = ylim;
% Xfill = [xlim(1) xlim(1) xlim(2) xlim(2)];
% Yfill = [max(fi_y(xlim(1)),ylim(1)) min(fi_y(xlim(1)),ylim(2)) max(fi_y(xlim(2)),ylim(2)) ylim(1)];
Xfill = [-10 10 10];
Yfill = [fi_y(-10), 100, fi_y(10)];
h = fill(Xfill,Yfill, 'cyan');
set(h, 'FaceAlpha',0.3, 'EdgeAlpha',0);
scatter(x0(1,:),x0(2,:), 'Marker','o', 'MarkerFaceColor','green', 'LineWidth',3, 'SizeData',200, 'MarkerEdgeAlpha',0);
scatter(x_fmincon(1,:),x_fmincon(2,:), 'Marker','*', 'CData',[1 0 0], 'LineWidth',3, 'SizeData',200);
scatter(x_qp(1,:),x_qp(2,:), 'Marker','*', 'CData',[0.5 0 0], 'LineWidth',3, 'SizeData',200);
scatter(x(1,:),x(2,:), 'Marker','*', 'CData',[0.85 0.33 0.1], 'LineWidth',3, 'SizeData',200);
legend({'level sets','$\le$ constr','$=$ constr','infeasible region','$x_0$','$x^*_{fmincon}$', '$x^*_{quadprog}$','$x^*$'}, 'interpreter','latex', 'fontsize',15);
hold(ax,'off');



function [f, df] = fun(x)

    global P x_c;
    
    f = (x-x_c)'*P*(x-x_c);

    if (nargout > 1), df = gradFun(x); end
end

function g = gradFun(x)

    global P x_c;
    
    g = P*(x-x_c);

end

function H = hessianFun(x)

    global P;
    
    H = P;

end

function P = genRandSPDmatrix(n)
    
    A = rand(n,n);
    P = eye(n,n) + A'*A;

end

