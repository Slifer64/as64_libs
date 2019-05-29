function Qe = quatError(Q, Qg)

    Qe = quatProd(Qg, quatInv(Q));

end