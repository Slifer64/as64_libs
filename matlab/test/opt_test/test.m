clc;
close all;
clear;

set_matlab_utils_path();

rng(0);


%% ===========  Linear minus sum of log of affine inequalities  =================
n = 100;
m = 50;
obj = LinearMinusSumLogAffIneqObj(n,m);
x0 = obj.genPointInDom();

% %% ===========  Quadratic Objective  =================
% n = 2;
% x0 = 1*rand(n,1);
% P = diag([1 10]);
% obj = QuadraticObj(n, P);
% % obj = QuadraticObj(n);


% %% ===========  Sum of exponentials of affine functions  =================
% n = 2;
% x0 = 1*rand(n,1);
% obj = SumExpAffObj();


%% ============================================
%% ============================================

%% ===========  Solve Method  =================
opt_obj = GradientDescent(@obj.fun, @obj.gradFun);

[x, x_data] = opt_obj.run(x0, 1e-3, 200);

obj.fun(x)

%% ============================================
%% =============  Plot results ================


J = zeros(1,size(x_data,2));
dJ = zeros(size(J));
for i=1:length(J)
    J(i) = obj.fun(x_data(:,i));
    dJ(i) = norm(obj.gradFun(x_data(:,i)));
end
    
figure;
subplot(2,1,1);
plot(J, 'LineWidth',2);
ylabel('$J$', 'interpreter','latex', 'fontsize',18);
axis tight;
subplot(2,1,2);
plot(dJ, 'LineWidth',2);
ylabel('$||\nabla J||_2$', 'interpreter','latex', 'fontsize',18);
xlabel('iter \#', 'interpreter','latex', 'fontsize',18);
axis tight;

% x1 = linspace(-0.5, 0.5,200);
% x2 = linspace(-0.5, 0.5,200);
% [X1,X2] = meshgrid(x1,x2);
% Z = zeros(size(X1));
% for i=1:size(Z,1)
%     for j=1:size(Z,2), Z(i,j) = obj.fun([X1(i,j); X2(i,j)]); end
% end
% figure;
% hold on;
% contour(X1,X2,Z);
% scatter(x_data(1,:),x_data(2,:), 'Marker','*', 'MarkerFaceColor','magenta');
% plot(x_data(1,:),x_data(2,:), 'Color','red');
% hold off;


