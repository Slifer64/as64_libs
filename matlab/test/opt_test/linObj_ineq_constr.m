clc;
close all;
clear;

set_matlab_utils_path();

rng(0);

global c;

n = 50;
m_i = 100;


Ai = randn(m_i, n);
x0 = randn(n,1);
s0 = randn(m_i,1);
lambda0 = -s0/100;
lambda0(lambda0<0) = 0;
s0(s0<0) = 0;
bi = Ai*x0 - s0;
c = -Ai'*lambda0;

x0 = zeros(n,1);

%% ============= Solve phase1 ==============
phase1 = Phase1Solver(LinIneqConstr(Ai,bi));
[x0_1, s] = phase1.solve(x0);

s

%% ============  Solve  ================
tic
x_lp = linprog(c,Ai,bi);
fprintf('=====================================\n');
toc
fprintf('linprog: p_star = %f\n',fun(x_lp));
fprintf('=====================================\n');

N_var = length(x0);
solver = NewtonDescent(N_var, @fun, @gradFun, @hessianFun);
solver.setStopThreshold(1e-6);
solver.setMaxIters(4000);
solver.setLinIneqConstr(Ai, bi);
solver.setDamping(0.01);
tic
% solver.setKKTsolveMethod(KKTSolveMethod.FULL_INV);
[x, v, x_data] = solver.solve(x0);
fprintf('=====================================\n');
toc
fprintf('Solve ineq: p_star = %f\n',fun(x));
fprintf('=====================================\n');

dgap = fun(x0_1) - fun(x_lp)
t0 = m_i / dgap

fun(x)
fun(x_lp)

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

    global c;
    
    f = c'*x;

    if (nargout > 1), df = gradFun(x); end
end

function g = gradFun(x)

    global c;
    
    g = c;

end

function H = hessianFun(x)
    
    n = length(x);
    H = zeros(n,n);

end

