clc;
close all;
clear;

% ============= Pos-Orient path ================
figure;
ax = plot_3Dpath_with_orientFrames(Pd_data, Qd_data, 'LineWidth',3.0, 'LineStyle','-', 'numberOfFrames',12, 'frameScale',0.2, 'frameLineWidth',2.0, ...
    'FrameXAxisLegend','$x_d$-axis', 'FrameYAxisLegend','$y_d$-axis', 'FrameZAxisLegend','$z_d$-axis', 'LineLegend','demo-path', 'LegendFontSize',14);
    
plot_3Dpath_with_orientFrames(P_data(:,1:4:end), Q_data(:,1:4:end), 'axes',ax, 'LineWidth',2.0, 'LineStyle','--', 'LineColor',[0.93 0.69 0.13], 'numberOfFrames',12, 'frameScale',0.2, 'frameLineWidth',2.0, ...
    'frameXAxisColor', [1 0 1], 'frameYAxisColor', [0 0.5 0], 'frameZAxisColor', [0 1 1], 'frameLineStyle','--', 'animated',false, ...
    'FrameXAxisLegend','$x$-axis', 'FrameYAxisLegend','$y$-axis', 'FrameZAxisLegend','$z$-axis', 'LineLegend','sim-path', 'LegendFontSize',14);
