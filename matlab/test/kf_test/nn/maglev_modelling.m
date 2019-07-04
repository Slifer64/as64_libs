clc;
close all;
clear;

[x,t] = maglev_dataset;


setdemorandstream(491218381)

net = narxnet(1:2,1:2,10);
view(net)


[Xs,Xi,Ai,Ts] = preparets(net,x,{},t);

[net,tr] = train(net,Xs,Ts,Xi,Ai);

plotperform(tr)

Y = net(Xs,Xi,Ai);

perf = mse(net,Ts,Y)


plotresponse(Ts,Y)


net2 = closeloop(net);
view(net2)

[Xs,Xi,Ai,Ts] = preparets(net2,x,{},t);
Y = net2(Xs,Xi,Ai);
plotresponse(Ts,Y)
