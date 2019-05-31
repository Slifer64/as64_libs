function ddy = getYddot(this, tau_dot, yc_dot)
            
    if (nargin < 3), tau_dot=0; end
    if (nargin < 3), yc_dot=0; end
    ddy = (this.getZdot() - tau_dot*this.getYdot() + yc_dot)/this.getTau(); 
    
end
