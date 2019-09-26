clc;
close all;
clear;

set_matlab_utils_path();

rng(0);

global P x_c;

n = 100;
m_i = 50;

P = genRandSPDmatrix(n);
x_c = rand(n,1);

Ai = rand(m_i, n);
bi = rand(m_i, 1);

x0 = rand(n,1);

%% ============= Solve phase1 ==============
phase1 = Phase1Solver(LinIneqConstr(Ai,bi));
[x, s] = phase1.solve(x0);

s

%% ============  Solve  ================
% options = optimoptions(@fmincon, 'Algorithm','interior-point', 'MaxIterations',100, 'SpecifyObjectiveGradient',true);
% tic
% x_fmincon = fmincon(@fun,x0,Ai,bi,[],[], [],[], [], options);
% fprintf('=====================================\n');
% toc
% fprintf('fmincon: p_star = %f\n',fun(x_fmincon));
% fprintf('=====================================\n');

tic
H = P;
f = -2*P*x_c;
x_qp = quadprog(H,f,Ai,bi);
fprintf('=====================================\n');
toc
fprintf('quadprog: p_star = %f\n',fun(x_qp));
fprintf('=====================================\n');

N_var = length(x0);
solver = NewtonDescent(N_var, @fun, @gradFun, @hessianFun);
solver.setStopThreshold(1e-7);
solver.setMaxIters(100);
solver.setLinIneqConstr(Ai, bi);
tic
% solver.setKKTsolveMethod(KKTSolveMethod.FULL_INV);
[x, v, x_data] = solver.solve(x0);
fprintf('=====================================\n');
toc
fprintf('Solve ineq: p_star = %f\n',fun(x));
fprintf('=====================================\n');


% %% ============================================
% %% =============  Plot results ================
% 
% iterNum = 0:(size(x_data,2)-1);
% 
% J = zeros(1,size(x_data,2));
% dJ = zeros(size(J));
% for i=1:length(J)
%     J(i) = fun(x_data(:,i));
%     dJ(i) = norm(gradFun(x_data(:,i)));
% end
%     
% figure;
% subplot(2,1,1);
% plot(iterNum, J, 'LineWidth',2);
% ylabel('$J$', 'interpreter','latex', 'fontsize',18);
% axis tight;
% subplot(2,1,2);
% plot(iterNum, dJ, 'LineWidth',2);
% ylabel('$||\nabla J||_2$', 'interpreter','latex', 'fontsize',18);
% xlabel('iter \#', 'interpreter','latex', 'fontsize',18);
% axis tight;



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

