clc;
close all;
clear;

set_matlab_utils_path();

rng(0);

global Ai bi t;

n = 100;
m_i = 160;
m = 40;

t = 1;
Ai = rand(m_i,n);
bi = rand(m_i,1);

A = rand(m,n);
b = rand(m,1);

x0 = rand(n,1);

s = max(Ai*x0-bi) + 0.5;

z0 = [s; x0];

%% ============  Solve  ================
% options = optimoptions(@fmincon, 'Algorithm','interior-point', 'MaxIterations',100, 'SpecifyObjectiveGradient',true);
% tic
% z1 = fmincon(@fun,z0,[],[],[],[], [],[], [], options);
% fprintf('=====================================\n');
% toc
% fprintf('fmincon: p_star = %f\n',fun(z1));
% fprintf('=====================================\n');


phase1 = Phase1Solver(LinIneqConstr(Ai,bi));
tic
[x, s] = phase1.solve(x0);
fprintf('=====================================\n');
toc
fprintf('Phase1 ineq solve: p_star = %f\n',fun([s; x]));
if (s>0), fprintf('[WARNING]: s = %.2f\n',s); end
ind = find(Ai*x-bi >= 0);
if (~isempty(ind)), fprintf('[WARNING]: Inequalities were not satisfied...'); end
fprintf('=====================================\n');

%% ===================================

phase1 = Phase1Solver(LinIneqConstr(Ai,bi));
phase1.setEqConstr(A,b);
phase1.setKKTSolveMethod(KKTSolver.FULL_INV);
tic
[x, s] = phase1.solve(x0);
fprintf('=====================================\n');
toc
fprintf('Phase1 eq-ineq solve: p_star = %f\n',fun([s; x]));
if (s>0), fprintf('[WARNING]: s = %.2f\n',s); end
ind = find(Ai*x-bi >= 0);
if (~isempty(ind)), fprintf('[WARNING]: Inequalities were not satisfied...'); end
max_eq_err = max(abs(A*x-b));
if (max_eq_err>1e-9), fprintf('[WARNING]: Equalities were not satisfied... Maximum error is: %g', max_eq_err); end
fprintf('=====================================\n');


% %% ============  Plot  =================
% 
% fi_y = @(x) (bi - Ai(:,1:end-1)*x)./Ai(:,end);
% 
% x1_max = 3;
% x2_max = 3;
% x1 = linspace(-x1_max, x1_max, 200);
% x2 = linspace(-x2_max, x2_max, 200);
% [X1,X2] = meshgrid(x1,x2);
% Z = zeros(size(X1));
% for i=1:size(Z,1)
%     for j=1:size(Z,2), Z(i,j) = fun([X1(i,j); X2(i,j)]); end
% end
% figure;
% ax = axes();
% hold(ax,'on');
% contour(X1,X2,Z, (0:0.3:5).^2);
% xlim = ax.XLim;
% ylim = ax.YLim;
% plot([-x1_max x1_max], [fi_y(-x1_max) fi_y(x1_max)], 'LineWidth',2, 'LineStyle','--', 'Color','magenta');
% % scatter(x_data(1,:),x_data(2,:), 'Marker','*', 'MarkerFaceColor','magenta');
% % plot(x_data(1,:),x_data(2,:), 'Color','red');
% ax.XLim = xlim;
% ax.YLim = ylim;
% Xfill = [xlim(1) xlim(1) xlim(2) xlim(2)];
% Yfill = [ylim(1) min(fi_y(xlim(1)),ylim(2)) max(fi_y(xlim(2)),ylim(2)) ylim(1)];
% h = fill(Xfill,Yfill, 'cyan');
% set(h, 'FaceAlpha',0.3, 'EdgeAlpha',0);
% scatter(x0(1,:),x0(2,:), 'Marker','o', 'MarkerFaceColor','green', 'LineWidth',3, 'SizeData',200, 'MarkerEdgeAlpha',0);
% scatter(x_fmincon(1,:),x_fmincon(2,:), 'Marker','*', 'CData',[1 0 0], 'LineWidth',3, 'SizeData',200);
% scatter(x(1,:),x(2,:), 'Marker','*', 'CData',[0.85 0.33 0.1], 'LineWidth',3, 'SizeData',200);
% legend({'level sets','$\le$ constr','infeasible region','$x_0$','$x^*_{fmincon}$','$x^*$'}, 'interpreter','latex', 'fontsize',15);
% hold(ax,'off');


function [f, df] = fun(x)

    global Ai bi t;
    
    s = x(1);
    
    [phi, ~, ~] = ph1LogBarFun(x);

    f = t*s + phi;

    if (nargout > 1), df = gradFun(x); end
    
end

function g = gradFun(x)

    global Ai bi t;
    
    [~, grad_phi, ~] = ph1LogBarFun(x);
    
    N_var = length(x);
    grad_ts = [t; zeros(N_var-1,1)];
    
    g = grad_ts + grad_phi;

end

function H = hessianFun(x)

    global Ai bi t;
    
    [~, ~, hess_phi] = ph1LogBarFun(x);
    
    H = hess_phi;

end

function [phi, grad_phi, hess_phi] = ph1LogBarFun(x)
            
    global Ai bi t;

    m = length(bi);
    Ai2 = [-ones(m,1) Ai];

    y = bi - Ai2*x;

    phi = -sum(log(y));

    d = ones(m,1)./y;

    if (nargout > 1), grad_phi = Ai2' * d; end
    if (nargout > 2), hess_phi = Ai2' * diag(d.^2) * Ai2; end
    
end

