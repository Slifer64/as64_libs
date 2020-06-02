function [p, Q] = lwr4pPose(q, Q_prev)

    fk = lwr4_fkine(q);
    
    p = fk(1:3,4);
    Q = rotm2quat(fk(1:3,1:3))';
    
    if (nargin > 1)
        if (dot(Q, Q_prev)<0), Q = -Q; end
    end

end