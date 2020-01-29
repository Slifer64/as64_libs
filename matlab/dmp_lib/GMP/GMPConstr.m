%% GMP constraint class
%

classdef GMPConstr < matlab.mixin.Copyable
    
    methods (Access = public)
        
        %% Constructor.
        %  @param[in] t: time instant of constraint
        %  @param[in] value: value of constraint
        %  @param[in] type: type of constraint {'=', '>', '<'}.
        function this = GMPConstr(t, value, type)
                
            if (nargin < 3), return; end
            
            this.t = t;
            this.value = value;
            this.type = type;
            
            if (type~='=' && type~='<' && type~='>')
                error('[GMPConstr::GMPConstr()]: Invalid constraint type: ''%s''',type);
            end
            
        end

    end
    
    properties (Access = public)
     
        t % parameter 'a_z' relating to the spring-damper system
        value % parameter 'b_z' relating to the spring-damper system
        type % 
        
    end
    
end
