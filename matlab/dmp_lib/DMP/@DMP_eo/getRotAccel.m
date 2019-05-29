function rotAccel = getRotAccel(this, Q, Qg, tau_dot, yc_dot)
            
    if (nargin < 3), tau_dot=0; end
    if (nargin < 3), yc_dot=0; end

    Qe = DMP_eo.quatError(Q, Qg);
    ddeo = this.getYddot(tau_dot, yc_dot);
    rotVel = this.getRotVel(Q, Qg);
    rotAccel = DMP_eo.ddeo2rotAccel(ddeo, rotVel, Qe);

end