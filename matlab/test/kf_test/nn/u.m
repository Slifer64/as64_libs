

X2_hat = zeros(1, n_data);
xin = Xi;
X2_hat(1:n_lag) = cell2mat(xin);
for j=n_lag+1:n_data
    xout = net(cell(0,1),xin,Ai);
    X2_hat(j) = xout{1};
    xin(2:end) = xin(1:end-1);
    xin{1} = xout{1};
    
    xout{1}
    X_hat(j)
    
    pause
end
