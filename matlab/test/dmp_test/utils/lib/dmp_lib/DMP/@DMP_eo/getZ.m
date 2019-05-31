function z = getZ(this, rotVel, Q, Qg)
            
    Qe = DMP_eo.quatError(Q, Qg);
    deo = DMP_eo.rotVel2deo(rotVel, Qe);
    dy = deo;
    
    z = this.getTau()*dy; 
    
end
