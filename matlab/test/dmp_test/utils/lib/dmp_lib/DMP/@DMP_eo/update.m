function update(this, x, Y, Z, Yc, Zc)
            
    if (nargin < 6), Yc = zeros(3,1); end
    if (nargin < 7), Zc = zeros(3,1); end

    if (isscalar(Yc)), Yc = ones(3,1)*Yc; end
    if (isscalar(Zc)), Zc = ones(3,1)*Zc; end

%     disp('========== [DMP_eo::update] ============');
%     x
%     Yi = Y'
%     Zi = Z'
%     Y0i = Y0'
    
    for i=1:3, this.dmp{i}.update(x, Y(i), Z(i), 0, Yc(i), Zc(i)); end

    this.dY = zeros(3,1);
    this.dZ = zeros(3,1);
    for i=1:3
        this.dY(i) = this.dmp{i}.getYdot();
        this.dZ(i) = this.dmp{i}.getZdot();    
    end
    this.dx = this.phaseDot(x);

end
        