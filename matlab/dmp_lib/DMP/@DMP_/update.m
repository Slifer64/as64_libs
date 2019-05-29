function update(this, x, y, z, y0, g, y_c, z_c)

    if (nargin < 7), y_c=0; end
    if (nargin < 8), z_c=0; end

    tau = this.getTau();
    shape_attr = this.shapeAttractor(x, y0, g);
    goal_attr = this.goalAttractor(x, y, z, g);

    this.dz = ( goal_attr + shape_attr + z_c) / tau;
    this.dy = ( z + y_c) / tau;
    this.dx = this.phaseDot(x);

end