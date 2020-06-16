%% Class for creating a trajectory by defining the via points.
%  The via points are then connected using splines

classdef CreateTraj_Splines < CreateTraj
   
    methods (Access = public)
        
        function this = CreateTraj_Splines(ax_)

            if (nargin < 1)
                ax_ = axes();
                ax_.XLim = [0 1];
                ax_.YLim = [0 1];
            end
            
            this = this@CreateTraj(ax_);
            
            this.addToGUI();
            
        end
        
    end
    
    methods (Access = protected)
        
        function y = getTrajectory(this, x)
            
            y = spline(this.points(1,:), this.points(2,:), x);
            
        end
        
        function addToGUI(this)

            this.ui_fig.Name = 'Create Trajectory with Splines';
            
        end

    end
    
    
    properties (Access = protected)

        
    end
    
end