clc;
close all;
clear;


n = 20;

S = zeros(n, n);

for i=1:n
    for j=1:n
        S(i,j) = calc(i,j);
    end
end


S(1,:)

e = eig(S);
is_spd = isempty(find(e<=0))




function a = calc(i,j)

    
    % a = 1 / ( 0.1*abs(i-j)^2 + 1 );
    
    a = exp(-0.5 * abs(i-j));


end



