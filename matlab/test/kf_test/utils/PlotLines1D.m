%% Plot 1D lines againts time.
%

classdef PlotLines1D < handle


    %% ====  Methods  =====
    methods
            
        %% Constructor.
        % @param[in] ax: axes handle.
        % @param[in] N: number of lines.
        %
        function this = PlotLines1D(ax, N)
        
            this.ax = ax;
            hold(ax,'on');
            this.pl = cell(N,1);
            for i=1:N, this.pl{i} = animatedline(nan(1),nan(1), 'Parent',ax); end
            
        end
        
        
        %% Set the graphics properties for the i-th line.
        % @param[in] i: The number of the line (starting from 1).
        % @param[in] varargin: variable argument list exactly like matlab's @set for a @line graphics object.
        % @param[out] line_handle: handle to @line object.
        %
        function line_handle = setLineProperties(this, i, varargin)
            
            this.pl{i}.set(varargin{:});
            line_handle = this.pl{i};
            
        end
    
        
        %% Set axes legend.
        % @param[in] varargin: variable argument list exactly like matlab's @legend.
        % @param[out] leg_handle: handle to @legend object.
        %
        function leg_handle = setLegend(this, varargin)

            leg_handle = legend(this.ax, varargin{:});
            
        end
        
        
        %% Set axes titile.
        % @param[in] varargin: variable argument list exactly like matlab's @title.
        %
        function setTitle(this, varargin)

            args = {varargin{:}, 'Parent', this.ax};
            title(args{:});
            
        end
        
        
        %% Set x-axis label.
        % @param[in] varargin: variable argument list exactly like matlab's @xlabel.
        %
        function setXLabel(this, varargin)

            args = {varargin{:}, 'Parent', this.ax};
            xlabel(args{:});
            
        end
        
        
        %% Set y-axis label.
        % @param[in] varargin: variable argument list exactly like matlab's @ylabel.
        %
        function setYLabel(this, varargin)

            args = {varargin{:}, 'Parent', this.ax};
            ylabel(args{:});
            
        end
        
        
        %% Update the axes according to measurement update.
        % @param[in] time: scalar or rowvector with timestamps.
        % @param[in] x: vector or matrix where the i-th row contains the values of the i-th line corresponding to the timestamps in 'time'.
        %
        function update(this, time, x)
           
            if (length(time) ~= size(x,2)), error('[PlotLines1D::update]: The columns of x must be equal to the length of time'); end
            
            for i=1:length(this.pl)
                for j=1:length(time), this.pl{i}.addpoints(time(j), x(i,j)); end
                % this.pl{i}.XData = [this.pl{i}.XData time];
                % this.pl{i}.YData = [this.pl{i}.YData x(i,:)];
            end
            
            drawnow;
            
        end

 
    end
    
    properties (Access = private)
       
        ax % axes handle
        pl % cell array with @line handles for each line
        
    end
    
end
