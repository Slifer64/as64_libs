%% Class for creating a trajectory by defining the via points.
%  The via points are then connected using splines

classdef CreateTraj_WSoG < CreateTraj
   
    methods (Access = public)
        
        function this = CreateTraj_WSoG(ax_)

            if (nargin < 1)
                ax_ = axes();
                ax_.XLim = [0 1];
                ax_.YLim = [0 1];
            end
            
            this = this@CreateTraj(ax_);
            
            this.N_kernels = 10;
            
            this.addToGUI();
 
        end
        
    end
    
    methods (Access = protected)
        
        function y = getTrajectory(this, x)
            
            this.trainModel(this.points(1,:), this.points(2,:));
                
            y = zeros(size(x));
            for j=1:length(x), y(j) = this.getOuput(x(j)); end
             
        end
        
        function addToGUI(this)

            this.ui_fig.Name = 'Create Trajectory with WSoG';
            
            this.ui_fig.Position = this.ui_fig.Position + [0 0 100 0];
            
            Nker_p = uipanel(this.ui_fig, 'Title','Model Params','FontSize',18,...
                         'BackgroundColor',[0.9 0.9 0.9],...
                         'Position',[210, 100 180 80]);

            xlim_lb = uilabel(Nker_p, 'Text','N_kernels', 'FontSize',20, ...
                'Position',[10 10 100 30]);
            xlim_le = uieditfield(Nker_p, 'numeric', 'Value',this.N_kernels, 'FontSize',18, ...
                'Position',[110 10 50 30]);
            xlim_le.ValueChangedFcn = @this.numOfKernelsChanged;
            
        end
        
        function numOfKernelsChanged(this, le, ev_info)
           
            this.N_kernels = le.Value;
            this.redraw();
            
        end

        function trainModel(this, xd, yd)
           
            this.c = linspace(xd(1), xd(end), this.N_kernels)';
            this.h = 1/(this.c(2) - this.c(1))^2;
            
            Psi = zeros(this.N_kernels, length(xd));
            for j=1:length(xd), Psi(:,j) = this.regressVec(xd(j)); end
            this.w = yd/Psi;
            
        end
        
        function y = getOuput(this, x)
           
            y = this.w*this.regressVec(x);
            
        end
        
        function phi = regressVec(this, x)
            
            psi = exp(-this.h*(x-this.c).^2);
            phi = psi / (sum(psi) + 1e-32);
            
        end


    end
    
    
    properties (Access = protected)

        N_kernels
        h
        c
        w
        
    end
    
end