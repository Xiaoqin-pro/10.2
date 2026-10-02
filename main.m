clc
clear
close all

%% Static 3-D UAV control-point baseline
root = fileparts(mfilename('fullpath'));
if ~exist(fullfile(root,'results'),'dir'), mkdir(fullfile(root,'results')); end

%% Problem parameters
cfg.dataFile = fullfile(root,'data','rc101.txt');
cfg.mapSize = [100 100];
cfg.nOrders = 20;
cfg.nControlPoints = 2;
cfg.maxControlHeight = 35;
cfg.timeWindowLevel = 2;
cfg.serviceTime = 3;
cfg.windowBefore = [50 30 15];
cfg.windowAfter = [70 45 25];
cfg.seed = 20260929;
cfg.safetySamples = 30;
cfg.maxClimbAngle = 25;
cfg.maxTurnAngle = 120;
cfg.smoothWeight = 2;
model = CreateModel(cfg);
state.time = 0;
state.position = model.depot;
state.activeIDs = model.activeIDs;
state.direction = [];

Particle_Number = 20;
maxgen = 100;
[Best,T] = PSO(model,state,maxgen,Particle_Number,1);
save(fullfile(root,'results','main_result.mat'),'model','state','Best','T');
PlotSolution(Best,model,state,fullfile(root,'results','baseline_route_3D.png'),'route');
PlotSolution(Best,model,state,fullfile(root,'results','all_orders_route_3D.png'),'all');
figure('Color','w'); plot(T,'LineWidth',1.8); grid on
xlabel('The Number of Iterations','fontsize',12);
ylabel('The Function Value','fontsize',12);
title('Static 3-D UAV control-point PSO baseline');
exportgraphics(gcf,fullfile(root,'results','baseline_convergence.png'),'Resolution',150);
fprintf('Cost: %.3f\nDistance: %.3f\nLate: %.3f\nFeasible: %d\n', ...
    Best.Cost,Best.Detail.distance,Best.Detail.totalLate,Best.Detail.feasible);
