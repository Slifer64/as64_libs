function eo = quat2eo(Q, Qg)
            
    eo = quatLog(DMP_eo.quatError(Q,Qg));
    
end
