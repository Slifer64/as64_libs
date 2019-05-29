function ddy = calcYddot(this, x, y, dy, y0, g, tau_dot, yc, zc, yc_dot)

    if (nargin < 7), tau_dot = 0; end
    if (nargin < 8), yc = 0; end
    if (nargin < 9), zc = 0; end
    if (nargin < 10), yc_dot = 0; end

    % tau = this.getTau();
    % z = dy*tau - yc;
    % this.calcStatesDot(x, y, z, y0, g, yc, zc);

    tau = this.getTau();
    z = dy*tau - yc;

    tau = this.getTau();

    shape_attr = this.shapeAttractor(x, y0, g);
    goal_attr = this.goalAttractor(x, y, z, g);
    dz = ( goal_attr + shape_attr + zc) / tau;

    ddy = (dz + yc_dot - tau_dot*dy)/tau;

end