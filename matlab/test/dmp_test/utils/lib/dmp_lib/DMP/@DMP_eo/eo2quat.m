function Q = eo2quat(eo, Qg)
            
    Q = quatProd( quatInv(quatExp(eo)), Qg);
    
end
