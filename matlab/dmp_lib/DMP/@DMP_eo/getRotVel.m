function rotVel = getRotVel(this, Q, Qg)
            
    Qe = DMP_eo.quatError(Q, Qg);
    deo = this.getYdot(); 
    rotVel = DMP_eo.deo2rotVel(deo, Qe);

end