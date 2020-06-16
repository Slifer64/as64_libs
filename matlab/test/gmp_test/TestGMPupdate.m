classdef TestGMPupdate < handle
   
    
    methods (Access = public)
        
        function this = TestGMPupdate(N_kernels, kernels_std_scaling)

            % init GMP
            this.N_kernels = N_kernels;
            this.kernels_std_scaling = kernels_std_scaling;

            % init GUI
            this.createGUI();
            
            % load train data
            load('data/train_data.mat', 'Data');
            Timed = Data.Time;
            this.Pd_data = Data.Pos(1,:);
            this.dPd_data = Data.Vel(1,:);
            this.ddPd_data = Data.Accel(1,:);
            
            this.xd_data = Timed/Timed(end);
            this.tau_d  =Timed(end);
            
            this.trainModel();
            
            % init plots
            this.ax = cell(4,1);
            y_labels = {'pos', 'weights', 'vel', 'accel'};
            fig = figure('Position',[465 14 775 936]);
            ind = [1 4 2 3];
            for i=1:4
                this.ax{ind(i)} = subplot(4,1,i);
                ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',15, 'Parent',this.ax{ind(i)});
                hold(this.ax{ind(i)}, 'on');
            end
            
            for i=1:3
                this.sc{i} = scatter(nan,nan, 'Marker','x', 'MarkerFaceColor','red', 'LineWidth',2, 'SizeData',150, 'Parent',this.ax{i}); 
                this.pl{i} = plot(this.xd_data, nan(size(this.xd_data)), 'LineWidth',2, 'Color',[0 0 1 0.8], 'LineStyle','-', 'Parent',this.ax{i});
            end
            
            this.phi_pl = cell(this.N_kernels,1);
            for i=1:this.N_kernels
                this.phi_pl{i} = plot(nan, nan, 'LineWidth',2, 'LineStyle','-', 'Parent',this.ax{4});
            end
            
            plot(this.xd_data, this.Pd_data, 'LineWidth',2, 'Color','magenta', 'LineStyle',':', 'Parent',this.ax{1});
            plot(this.xd_data, this.dPd_data, 'LineWidth',2, 'Color','magenta', 'LineStyle',':', 'Parent',this.ax{2});
            plot(this.xd_data, this.ddPd_data, 'LineWidth',2, 'Color','magenta', 'LineStyle',':', 'Parent',this.ax{3});
 
            this.plotPosVelAccel();
            this.plotPhi();
            
%             barh0 = bar(this.gmp.wsog.c, w, 'FaceColor',[1 0 1], 'FaceAlpha',0.7, 'Parent',this.ax{4});
            this.barh = bar(nan, nan, 'FaceColor',[0 0 1], 'FaceAlpha',0.5, 'Parent',this.ax{4});
            this.plotWeights();

            % set up callbacks
            set(this.ax{1}, 'ButtonDownFcn', @(ax_,edata)addPoint(this, ax_, edata, 1));
            set(this.ax{2}, 'ButtonDownFcn', @(ax_,edata)addPoint(this, ax_, edata, 2));
            set(this.ax{3}, 'ButtonDownFcn', @(ax_,edata)addPoint(this, ax_, edata, 3));
            
            set(fig, 'KeyPressFcn', @(fig_,key_info)key_press_callback(this, fig_, key_info));
            
        end
        
        function createGUI(this)
            
            fig_w = 300;
            fig_h = 200;
            ui_fig = uifigure('Name','GMP update');
            ui_fig.Position = [ui_fig.Position(1) ui_fig.Position(2) fig_w fig_h];
            % ui_fig.Resize = 'off';

            del_last_point_btn = uibutton(ui_fig, 'push', ...
                'Text','Delete last point', ...
                'fontsize',18, ...
                'Position',[10 10 150 50]);
            del_last_point_btn.ButtonPushedFcn = @this.deleteLastPoint;


            ax_lim_lb_h = 30;
            std_scale_lb_w = 100;
            std_scale_le_w = 50;
            std_scale_le_h = ax_lim_lb_h;

            gmpParams_p = uipanel(ui_fig, 'Title','GMP params','FontSize',18,...
                         'BackgroundColor',[0.9 0.9 0.9],...
                         'Position',[10 fig_h-110 180 100]);

            y = 40;

            std_scale_lb = uilabel(gmpParams_p, 'Text','std scale', 'FontSize',20, ...
                'Position',[10 y std_scale_lb_w ax_lim_lb_h]);
            std_scale_le = uieditfield(gmpParams_p, 'text', 'Value',num2str(this.kernels_std_scaling), 'FontSize',18, ...
                'HorizontalAlignment','center', ...
                'Position',[10+std_scale_lb_w+5 y std_scale_le_w std_scale_le_h]);
            std_scale_le.ValueChangedFcn = @this.changeStdScaling;

            y = y-ax_lim_lb_h-5;

            Nkernels_lb = uilabel(gmpParams_p, 'Text','N_kernels', 'FontSize',20, ...
                'Position',[10 y std_scale_lb_w ax_lim_lb_h]);
            Nkernels_le = uieditfield(gmpParams_p, 'text', 'Value',num2str(this.N_kernels), 'FontSize',18, ...
                'HorizontalAlignment','center', ...
                'Position',[10+std_scale_lb_w+5 y std_scale_le_w std_scale_le_h]);
            Nkernels_le.ValueChangedFcn = @this.changeNkernels;
            
            this.ui_fig = ui_fig;

        end
        
        function changeNkernels(this, le, ev_info)
            
            this.N_kernels = str2num(le.Value);
            
            for i=1:length(this.phi_pl), delete(this.phi_pl{i}); end
            
            this.phi_pl = cell(this.N_kernels,1);
            for i=1:this.N_kernels
                this.phi_pl{i} = plot(nan, nan, 'LineWidth',2, 'LineStyle','-', 'Parent',this.ax{4});
            end
            
            this.trainModel();
            
            this.plotPosVelAccel();
            this.plotPhi();
            this.plotWeights();
            
        end
        
        function changeStdScaling(this, le, ev_info)
            
            this.kernels_std_scaling = str2double(le.Value);
            this.trainModel();
            
            this.plotPosVelAccel();
            this.plotPhi();
            this.plotWeights();
            
        end
        
        function addPoint(this, ax_, edata, i)

            this.points_add_stack = [this.points_add_stack this.sc{i}];
            
            pt = edata.IntersectionPoint(1:2)';
            this.sc{i}.XData = [this.sc{i}.XData pt(1)];
            this.sc{i}.YData = [this.sc{i}.YData pt(2)];
            drawnow;

        end
        
        function deleteLastPoint(this, btn, ev_info)
           
            sc_ = this.points_add_stack(end);
            
            if (length(sc_.XData) == 1), return; end
                
            sc_.XData = sc_.XData(1:end-1);
            sc_.YData = sc_.YData(1:end-1);
            
            if (length(sc_.XData) < 1)
                sc_.XData = nan;
                sc_.YData = nan;
            end
            
            drawnow;
            
            this.points_add_stack = this.points_add_stack(1:end-1);
            
        end
        
        function trainModel(this)
            
            this.gmp0 = GMP(this.N_kernels, 30, 100, this.kernels_std_scaling);
            offline_train_mse = this.gmp0.train('LS', this.xd_data, this.Pd_data);
            this.gmp = this.gmp0.deepCopy();
            
        end
        
        function [P_data, dP_data, ddP_data] = simulateGMP(this)
            
            N = length(this.xd_data);
            
            x_dot = 1 / this.tau_d;
            x_ddot = 0;
            
            P_data = zeros(1,N);
            dP_data = zeros(1,N);
            ddP_data = zeros(1,N);
            
            for i=1:N
               x = this.xd_data(i);
               P_data(i) = this.gmp.getYd(x);
               dP_data(i) = this.gmp.getYdDot(x, x_dot);
               ddP_data(i) = this.gmp.getYdDDot(x, x_dot, x_ddot);
            end
            
        end
        
        function plotPosVelAccel(this)
            
            [P_data, dP_data, ddP_data] = this.simulateGMP();
            
            this.pl{1}.YData = P_data;
            this.pl{2}.YData = dP_data;
            this.pl{3}.YData = ddP_data;
            
%             this.pl{2} = plot(this.xd_data, P_data, 'LineWidth',2, 'Color',[0 0 1 0.5], 'LineStyle','-', 'Parent',this.ax{1});
%             this.pl{2} = plot(this.xd_data, dP_data, 'LineWidth',2, 'Color',[0 0 1 0.5], 'LineStyle','-', 'Parent',this.ax{2});
%             this.pl{3} = plot(this.xd_data, ddP_data, 'LineWidth',2, 'Color',[0 0 1 0.5], 'LineStyle','-', 'Parent',this.ax{3});
            
            drawnow;
            
        end
        
        function plotPhi(this)
           
            N = length(this.xd_data);
            Phi_data = zeros(this.gmp.wsog.N_kernels, N);
            for i=1:N, Phi_data(:, i) = this.gmp.wsog.kernelFun(this.xd_data(i)); end
            
            for i=1:this.N_kernels
                this.phi_pl{i}.XData = this.xd_data;
                this.phi_pl{i}.YData = Phi_data(i,:);
            end
            
            drawnow;
            
        end
        
        function plotWeights(this)
            
            w = this.gmp.wsog.w;
            w = w / max(abs(w));
            this.barh.XData = this.gmp.wsog.c;
            this.barh.YData = w;
            this.ax{4}.XLim = [this.xd_data(1) this.xd_data(end)];
            this.ax{4}.YLim = [0 1.1];
            
        end
        
        function key_press_callback(this, fig_, key_info)

            ch = key_info.Character;

            % disp(['Pressed ''' ch '''']);

            if (strcmpi(ch,'d'))
                this.deleteLastPoint([],[]);
            elseif (strcmpi(ch,'w'))
                this.updateGMP();
            elseif (strcmpi(ch,'b'))
                this.resetToOriginal();
            end

        end
        
        function updateGMP(this)
           
            TYPE = [GMP_UPDATE_TYPE.POS; GMP_UPDATE_TYPE.VEL; GMP_UPDATE_TYPE.ACCEL];
            
            s = [];
            z = [];
            type = [];
    
            for i=1:3
               
                x_data = this.sc{i}.XData;
                if (length(x_data) == 1), continue; end
                y_data = this.sc{i}.YData;
                
                for j=2:length(x_data)
                    sj = [x_data(j); 1/this.tau_d; 0];
                    s = [s sj];
                    z = [z y_data(j)];
                    type = [type TYPE(i)];     
                end

            end
            
            this.gmp.updateWeights(s, z, type);
            
            this.plotPosVelAccel();
            this.plotWeights();
        end

        function resetToOriginal(this)
            
            this.gmp = this.gmp0.deepCopy();
            
            this.plotPosVelAccel();
            this.plotWeights();
        end
        
    end
    
    methods (Static, Access = public)
       
        function obj = run()
            
            clc;
            close all;
            clear;
            
            set_matlab_utils_path();
            
            N_kernels = 30;
            kernels_std_scaling = 1;
            obj = TestGMPupdate(N_kernels, kernels_std_scaling);
            
        end
        
    end
    
    
    properties (Access = public)
        
        % demo data
        xd_data
        tau_d
        Pd_data
        dPd_data
        ddPd_data
        
        N_kernels
        kernels_std_scaling
        gmp0 % GMP object
        gmp % updated GMP model
        
        ax % 1:position, 2:weights, 3:velocity, 4:acceleration
        sc % scatter plots, 3x1 for pos, vel, accel
        pl % line plots, 3x1 for pos, vel, accel
        phi_pl % line plots for phi
        barh % bar handle for plotting the weights
        
        points_add_stack
        
        % GUI
        ui_fig
      
    end
    
    
end