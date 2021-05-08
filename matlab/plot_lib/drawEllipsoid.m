% Returns the coordinates of an ellipsoid.
% The ellipsoids equation can be written in the quadratic form:
%   (x-c)'*P*(x-c) = 1
% where 'P' is the ellipsoids quadratic form matrix, 'c' its center and
% 'x' a random point that belongs on the ellipsoid.
% The ellipsoid is generated as follows:
% Points on a unit sphere are generated. These points obey the equation:
%   y'*y = 1
% Using the transformation:
%   x = inv(U)*y + c
% where U is the upper cholesky factorization of P, i.e. P = U'*U, we 
% transform the sphere to the desired ellipsoid. Indeed:
%   y = U*(x-c) => y'*y = 1 => (x-c)'*U'*U*(x-c) = (x-c)'*P*(x-c) = 1
% @param[in] P: the elipsoids quadratic form matrix
% @param[in] c: the elipsoids centers
% @param[in] n: number of faces (optional, default=20)
% @return X,Y,Z: (n+1) x (n+1) matrices with x, y and z coordinates of the ellipsoid.
function [X, Y, Z] = drawEllipsoid(P, c, n)
    
    if (nargin < 3), n = 20; end
    
    T = inv(chol(P,'upper')); % P = U'*U
    %T = inv(sqrtm(P)); % P = sqrt(P)*sqrt(P)

    % generate grid of points
    phi = [linspace(0,2*pi,n) 2*pi];
    theta = [linspace(0,pi,n) pi];
    [Phi, Theta] = meshgrid(phi, theta);

    m = size(Phi,1);
    n = size(Phi, 2);

    X = zeros(m,n);
    Y = zeros(m,n);
    Z = zeros(m,n);
    
    % generate points on the ellipsoid
    for i=1:m
        for j=1:n
            phi = Phi(i,j);
            theta = Theta(i,j);
            
            % polar coordinates of unit sphere
            x = cos(phi)*sin(theta);
            y = sin(phi)*sin(theta);
            z = cos(theta);
            p = [x; y; z];
            
            % transform each point on the sphere to a point on the ellipsoid
            p = T*p + c;

            X(i,j) = p(1);
            Y(i,j) = p(2);
            Z(i,j) = p(3);
        end
    end


end