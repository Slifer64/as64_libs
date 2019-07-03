clc
close all
clear


% x = 0:0.01:1;
% t = x.^2;
% net = feedforwardnet(10);
% net = train(net,x,t);
% % view(net)
% y = net(x);
% y_i = net(x(1));
% perf = perform(net,y,t)


% x1 =  [4 5 6];
% x2 =  [0 1 0];
% x = {x1;x2};
% t = [0 0 1];
% net = feedforwardnet;
% net.numinputs = 2;
% net = configure(net,x);
% net = train(net,x,t);
% view(net)
% 
% return

t = 0:0.01:12;
x1 = sin(t);
x2 = t;
X = {x1; x2};
y = x1.*x2;

net = fitnet([10]);
net.numinputs = 2;
net.inputConnect = [1 1; 0 0];
% view(net)
% net = configure(net, X,y);
net = train(net,X,y);

y_hat = zeros(size(y));

for j=1:size(t,2)
    x_in = {[X{1}(j); X{2}(j)]};
    x_out = net(x_in);
    y_hat(j) = x_out{1};
end

% y_hat = net(X);

figure;
plot(t,y, t,y_hat);
legend({'$y$','$\hat{y}$'}, 'interpreter','latex', 'fontsize',15);


