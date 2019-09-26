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
P = [1 0.7; 0.7 1];
x_c = [0; 0];

Ai = [0.7 -0.4];
bi = -1.5; 

fi_y = @(x) (bi - Ai(:,1:end-1)*x)./Ai(:,end);

% x0 = Ai\(0.5*bi);
x0 = [2; -2];

%% ============= Solve phase1 ==============
phase1 = Phase1Solver(LinIneqConstr(Ai,bi));
[x, s] = phase1.solve(x0);

s


%% ============  Solve  ================
options = optimoptions(@fmincon, 'Algorithm','interior-point', 'MaxIterations',100, 'SpecifyObjectiveGradient',true);
tic
x_fmincon = fmincon(@fun,x0,Ai,bi,[],[], [],[], [], options);
fprintf('=====================================\n');
toc
fprintf('fmincon: p_star = %f\n',fun(x_fmincon));
x_fmincon
fprintf('=====================================\n');

tic
H = P;
f = -2*P*x_c;
x_qp = quadprog(H,f,Ai,bi);
fprintf('=====================================\n');
toc
fprintf('quadprog: p_star = %f\n',fun(x_qp));
x_qp
fprintf('=====================================\n');

N_var = length(x0);
solver = NewtonDescent(N_var, @fun, @gradFun, @hessianFun);
solver.setStopThreshold(1e-7);
solver.setMaxIters(30);
solver.setLinIneqConstr(Ai, bi);
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
% scatter(x_data(1,:),x_data(2,:), 'Marker','*', 'MarkerFaceColor','magenta');
% plot(x_data(1,:),x_data(2,:), 'Color','red');
ax.XLim = xlim;
ax.YLim = ylim;
Xfill = [xlim(1) xlim(1) xlim(2) xlim(2)];
Yfill = [ylim(1) min(fi_y(xlim(1)),ylim(2)) max(fi_y(xlim(2)),ylim(2)) ylim(1)];
h = fill(Xfill,Yfill, 'cyan');
set(h, 'FaceAlpha',0.3, 'EdgeAlpha',0);
scatter(x0(1,:),x0(2,:), 'Marker','o', 'MarkerFaceColor','green', 'LineWidth',3, 'SizeData',200, 'MarkerEdgeAlpha',0);
scatter(x_fmincon(1,:),x_fmincon(2,:), 'Marker','*', 'CData',[1 0 0], 'LineWidth',3, 'SizeData',200);
scatter(x_qp(1,:),x_qp(2,:), 'Marker','*', 'CData',[0.5 0 0], 'LineWidth',3, 'SizeData',200);
scatter(x(1,:),x(2,:), 'Marker','*', 'CData',[0.85 0.33 0.1], 'LineWidth',3, 'SizeData',200);
legend({'level sets','$\le$ constr','infeasible region','$x_0$','$x^*_{fmincon}$', '$x^*_{quadprog}$','$x^*$'}, 'interpreter','latex', 'fontsize',15);
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

