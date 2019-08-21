%% KKT solve method enum
%

classdef KKTSolveMethod < handle
       
    properties (Constant)
        
        FULL_INV = 0 % Invert the entire KKT matrix, O((m+n)^3)
        BLOCK_ELIM = 1 % Solve KKT system with block elimination, O(n*m^2)
        EQ_ELIM = 2 % Eliminate equality constraints and solve unconstrained problem, O(n*(n-m)^2)
        
    end

end

