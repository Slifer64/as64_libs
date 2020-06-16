%% Class for creating a trajectory by defining the via points.
%  The via points are then connected using splines

classdef CreateTraj < handle
   
    methods (Access = public)
        
        function this = CreateTraj(ax_)

            if (nargin < 1)
                this.ax = axes();
                this.ax.XLim = [0 1];
                this.ax.YLim = [0 1];
            else
                this.ax = ax_;
            end

            hold(this.ax,'on');
            this.pl = plot(nan,nan, 'LineStyle','-', 'LineWidth',2, 'Color','blue', 'Parent',this.ax);
            this.sc = scatter(nan,nan, 'Marker','x', 'MarkerFaceColor','red', 'LineWidth',2, 'SizeData',150, 'Parent',this.ax);
            fig = this.ax.Parent;
            set(this.ax, 'ButtonDownFcn', @(ax_,edata)mouse_click_callback(this, ax_, edata));
            set(fig, 'KeyPressFcn', @(fig_,key_info)key_press_callback(this, fig_, key_info));
            
            this.createGUI();
            
        end
        
    end
    
    methods (Abstract, Access = protected)
        
        y = getTrajectory(this, x)
        
    end
    
    methods (Access = protected)
        
        function createGUI(this)
            
            fig_w = 300;
            fig_h = 200;
            ui_fig = uifigure('Name','Create Trajectory');
            ui_fig.Position = [ui_fig.Position(1) ui_fig.Position(2) fig_w fig_h];
            % ui_fig.Resize = 'off';

            del_last_point_btn = uibutton(ui_fig, 'push', ...
                'Text','Delete last point', ...
                'fontsize',18, ...
                'Position',[10 10 150 50]);
            del_last_point_btn.ButtonPushedFcn = @this.deleteLastPointTriggered;


            ax_lim_lb_h = 30;
            ax_lim_lb_w = 50;
            ax_lib_le_w = 100;
            ax_lib_le_h = ax_lim_lb_h;

            ax_p_w = 180;
            ax_p_h = 100;

            axis_p = uipanel(ui_fig, 'Title','Set Axis','FontSize',18,...
                         'BackgroundColor',[0.9 0.9 0.9],...
                         'Position',[10, fig_h-110 ax_p_w ax_p_h]);

            y = 40;

            xlim_lb = uilabel(axis_p, 'Text','xlim', 'FontSize',20, ...
                'Position',[10 y ax_lim_lb_w ax_lim_lb_h]);
            xlim_le = uieditfield(axis_p, 'text', 'Value','[0; 1]', 'FontSize',18, ...
                'Position',[10+ax_lim_lb_w+5 y ax_lib_le_w ax_lib_le_h]);
            xlim_le.ValueChangedFcn = @this.changeXLimTriggered;

            y = y-ax_lim_lb_h-5;

            ylim_lb = uilabel(axis_p, 'Text','ylim', 'FontSize',20, ...
                'Position',[10 y ax_lim_lb_w ax_lim_lb_h]);
            ylim_le = uieditfield(axis_p, 'text', 'Value','[0; 1]', 'FontSize',18, ...
                'Position',[10+ax_lim_lb_w+5 y ax_lib_le_w ax_lib_le_h]);
            ylim_le.ValueChangedFcn = @this.changeYLimTriggered;
            
            this.ui_fig = ui_fig;

        end
        
        function deleteLastPointTriggered(this, btn, ev_info)

            if (size(this.points,2)<2), this.points=[];
            else, this.points = this.points(:,1:end-1);
            end
            this.redraw();

        end

        function changeXLimTriggered(this, le, ev_info)

            this.ax.XLim = str2num(le.Value);

        end
        
        function changeYLimTriggered(this, le, ev_info)

            this.ax.YLim = str2num(le.Value);

        end
    
        function mouse_click_callback(this, ax_, edata)

            pt = edata.IntersectionPoint(1:2)';

            % disp(['Point: (' num2str(pt(1)) ',' num2str(pt(2)) ')']);

            this.points = [this.points pt];

            this.redraw();

        end

        function key_press_callback(this, fig_, key_info)

            % cpt = get(fig,'CurrentPoint')
            % pt = cpt(1,1:2)
            ch = key_info.Character;

            % disp(['Pressed ''' ch '''']);

            if (strcmpi(ch,'b'))
                this.deleteLastPointTriggered([],[]);
            end

        end

        function redraw(this)

            if (~isempty(this.points))
                x = this.points(1,1):0.01:this.points(1,end);
                x(end) = this.points(1,end); % make sure the final waypoint is included
            else
               x = []; 
            end
            
            if (size(this.points,2) > 1),  y = this.getTrajectory(x);
            else, y = zeros(size(x));
            end
            this.x_data = x;
            this.y_data = y;

            xlim = this.ax.XLim;
            ylim = this.ax.YLim;

            this.pl.XData = this.x_data;
            this.pl.YData = this.y_data;

            if (~isempty(this.points))
                this.sc.XData = this.points(1,:);
                this.sc.YData = this.points(2,:);
            else 
                this.sc.XData = nan;
                this.sc.YData = nan;
            end

            this.ax.XLim = xlim;
            this.ax.YLim = ylim;

            drawnow

        end

    end
    
    
    properties (Access = protected)

        points % via points
        ax % axes objects
        x_data % x-axis values
        y_data % y-axis values
        sc % scatter with the via points
        pl % line with the trajectory
        
        ui_fig % gui figure
        
    end
    
end