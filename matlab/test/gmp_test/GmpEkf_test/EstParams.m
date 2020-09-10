%% EstParams

classdef EstParams < handle
    
    %% =================================================
   
    methods (Access = public)
        
        
        function this = EstParams(est_params)
           
            for i=1:length(est_params)
                if (strcmpi(est_params{i},'g')), this.est_g = true;
                elseif (strcmpi(est_params{i},'tau')), this.est_tau = true;
                elseif (strcmpi(est_params{i},'x')), this.est_x = true;
                else, warning(['Unrecognized param ''' est_params{i} '''']);
                end
            end

        end
        
        function setActualParams(this, params)
           
            if (~this.est_g), this.g = params.g; end
            if (~this.est_tau), this.tau = params.tau; end
            if (~this.est_x), this.x = params.x; end
            
        end
        
        
        function theta = getEstParams(this)
            
            theta = [];
            if (this.est_g), theta = [theta; this.g]; end
            if (this.est_tau), theta = [theta; this.tau]; end
            if (this.est_x), theta = [theta; this.x]; end
            
        end
        
        
        function ret = getGoal(this, theta)
  
            ret = this.g;
            
        end
        
        function ret = getTau(this)
           
            ret = this.tau;
            
        end
        
        function ret = getPhaseVar(this)
           
            ret = this.x;
            
        end
        

    end
    
    %% =================================================
    
    properties (Access = public)
        
        est_g
        est_tau
        est_x
        
    end
    
    %% =================================================
    
    properties (Access = private)

        g
        tau
        x
        
    end
    
    

end



