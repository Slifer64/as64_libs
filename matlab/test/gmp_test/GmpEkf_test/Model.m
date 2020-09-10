%% Model class

classdef Model < matlab.mixin.Copyable
    
    %% ==========================================================
    
    methods (Access = public)
       
        function this = Model()
            
            
        end
        
    end
   
    %% ==========================================================
    
    methods (Access = public, Abstract)
        
        init(this, params)

        offline_train_mse = train(this, train_method, train_data)

        simulate(this, Time)

        setStart(this, y0)

        setGoal(this, g)

        theta_next = stateTransFun(this, theta, cookie)
        
        z = msrFun(this, theta, cookie)

        J = stateTransFunJacob(this, theta, cookie)
        
        J = msrFunJacob(this, theta, cookie)
        
        [y, y_dot] = nextState(this, x, y, y_dot, tau, dt)
        
    end
    
end
