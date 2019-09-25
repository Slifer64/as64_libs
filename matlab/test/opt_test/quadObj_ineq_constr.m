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
[x, s] = phase1.solveNoEqConstr(x0);

s

return

%% ============  Solve  ================
options = optimoptions(@fmincon, 'Algorithm','interior-point', 'MaxIterations',100, 'SpecifyObjectiveGradient',true);
tic
x_fmincon = fmincon(@fun,x0,Ai,bi,[],[], [],[], [], options);
fprintf('=====================================\n');
toc
fprintf('fmincon: p_star = %f\n',fun(x_fmincon));
fprintf('=====================================\n');


N_var = length(x0);
solver = NewtonDescent(N_var, @fun, @gradFun, @hessianFun);
solver.setStopThreshold(1e-5);
solver.setMaxIters(100);
solver.setLinInEqConstr(Ai, bi);
tic
% solver.setKKTsolveMethod(KKTSolveMethod.FULL_INV);
[x, v, x_data] = solver.solve(x0);
fprintf('=====================================\n');
toc
fprintf('Solve ineq: p_star = %f\n',fun(x));
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
scatter(x(1,:),x(2,:), 'Marker','*', 'CData',[0.85 0.33 0.1], 'LineWidth',3, 'SizeData',200);
legend({'level sets','$\le$ constr','infeasible region','$x_0$','$x^*_{fmincon}$','$x^*$'}, 'interpreter','latex', 'fontsize',15);
hold(ax,'off');

return

%% ============================================
%% ============================================

%% ===========  Solve Method  =================
% opt_obj = GradientDescent(@fun, @gradFun);
% eps = 1e-3;
% iters = 200;

x0 = rand(n,1);

N_var = length(x0);

solver = NewtonDescent(N_var, @fun, @gradFun, @hessianFun);
solver.setStopThreshold(1e-5);
solver.setMaxIters(100);
solver.setLinInEqConstr(Ai, bi);

tic
% solver.setKKTsolveMethod(KKTSolveMethod.FULL_INV);
[x, v, x_data] = solver.solve(x0);
fprintf('=====================================\n');
toc
fprintf('Solve ineq: p_star = %f\n',fun(x));
fprintf('=====================================\n');
% checkKKTcond(x1,v)

return 

tic
solver.setKKTsolveMethod(KKTSolveMethod.BLOCK_ELIM);
[x1, v, x_data] = solver.solve(x0);
fprintf('=====================================\n');
toc
fprintf('Solve eq - BLOCK_ELIM: p_star = %f\n',fun(x1));
fprintf('=====================================\n');

tic
solver.setKKTsolveMethod(KKTSolveMethod.EQ_ELIM);
[x1, v, x_data] = solver.solve(x0);
fprintf('=====================================\n');
toc
fprintf('Solve eq - EQ_ELIM: p_star = %f\n',fun(x1));
fprintf('=====================================\n');

x0 = rand(n,1);
tic
solver.setKKTsolveMethod(KKTSolveMethod.FULL_INV);
[x, v, x_data] = solver.solveEqInfeasStart(x0);
fprintf('=====================================\n');
toc
fprintf('Solve eq - Infeas start - FULL_INV: p_star = %f\n',fun(x));
fprintf('=====================================\n');
% checkKKTcond(x,v)

x0 = rand(n,1);
tic
solver.setKKTsolveMethod(KKTSolveMethod.BLOCK_ELIM);
[x, v, x_data] = solver.solveEqInfeasStart(x0);
fprintf('=====================================\n');
toc
fprintf('Solve eq - Infeas start - BLOCK_ELIM: p_star = %f\n',fun(x));
fprintf('=====================================\n');
% checkKKTcond(x,v)

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

