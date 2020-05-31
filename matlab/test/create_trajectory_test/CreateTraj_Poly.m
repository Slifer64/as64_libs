%% Class for creating a trajectory by defining the via points.
%  The via points are then connected using splines

classdef CreateTraj_Poly < CreateTraj
   
    methods (Access = public)
        
        function this = CreateTraj_Poly(ax_)

            if (nargin < 1)
                ax_ = axes();
                ax_.XLim = [0 1];
                ax_.YLim = [0 1];
            end
            
            this = this@CreateTraj(ax_);
            
            this.Np = 6;
            
            this.addToGUI();
 
        end
        
    end
    
    methods (Access = protected)
        
        function y = getTrajectory(this, x)

            xd = this.points(1,:);
            yd = this.points(2,:);
            
            A = zeros(this.Np+1, length(xd));
            for i=1:size(A,1)
                A(i,:) = xd.^(i-1);
            end
            
            w = yd/A;
                
            y = zeros(size(x));
            for j=1:length(x)
                a = zeros(this.Np+1, 1);
                for i=1:length(a), a(i) = x(j)^(i-1); end
                y(j) = w*a;
            end
            
        end
        
        function addToGUI(this)

            this.ui_fig.Name = 'Create Trajectory with Polynomial';
            
            this.ui_fig.Position = this.ui_fig.Position + [0 0 100 0];
            
            PolOrd_p = uipanel(this.ui_fig, 'Title','Model Params','FontSize',18,...
                         'BackgroundColor',[0.9 0.9 0.9],...
                         'Position',[210, 100 180 80]);

            xlim_lb = uilabel(PolOrd_p, 'Text','Poly order', 'FontSize',20, ...
                'Position',[10 10 100 30]);
            xlim_le = uieditfield(PolOrd_p, 'numeric', 'Value',this.Np, 'FontSize',18, ...
                'Position',[110 10 50 30]);
            xlim_le.ValueChangedFcn = @this.PolyOrderChanged;
            
        end

        
        function PolyOrderChanged(this, le, ev_info)
           
            this.Np = le.Value;
            this.redraw();
            
        end
 
    end
    
    properties (Access = protected)

        Np % polynomial order
        
    end
    
end