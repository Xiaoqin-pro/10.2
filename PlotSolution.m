function overview = PlotSolution(BestSol,model,filePath,mode)
%PLOTSOLUTION Plot the optimized route or the reference overview.

overview = [];
N = model.nOrders;
K = model.nControlPoints;

if strcmp(mode,'all')
    % Build a particle position for the reference route so Fitness uses it.
    position = zeros(1,N+(N+1)*K*3);
    position(model.referenceRoute) = (1:N)/(N+1);
    position(N+1:end) = ...
        reshape(permute(model.referenceControl,[3 2 1]),1,[]);
    plotTitle = sprintf('All %d orders: reference overview',model.nOrders);
else
    position = BestSol.Position;
    plotTitle = 'Static 3-D UAV control-point route';
end

[~,detail] = Fitness(position,model);
figure('Color','w');
surf(model.X,model.Y,model.terrainZ, ...
    'EdgeColor','none','FaceAlpha',0.65,'DisplayName','Terrain');
hold on
colormap parula
axis equal
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');

for i = 1:length(model.obstacles)
    obs = model.obstacles(i);
    [X,Y,Z] = cylinder(obs.r,30);
    X = X+obs.x;
    Y = Y+obs.y;
    Z = obs.zMin+Z*(obs.zMax-obs.zMin);
    if i == 1
        surf(X,Y,Z,'FaceColor',[0.75 0.25 0.25], ...
            'FaceAlpha',0.55,'EdgeColor','none', ...
            'DisplayName','Cylindrical obstacle');
    else
        surf(X,Y,Z,'FaceColor',[0.75 0.25 0.25], ...
            'FaceAlpha',0.55,'EdgeColor','none', ...
            'HandleVisibility','off');
    end
end

plot3(detail.points(:,1),detail.points(:,2),detail.points(:,3), ...
    'b-o','LineWidth',2,'MarkerSize',3,'MarkerFaceColor','w', ...
    'DisplayName','UAV route');
plot3(model.depot(1),model.depot(2),model.depot(3), ...
    'ks','MarkerSize',10,'MarkerFaceColor','k','DisplayName','Depot');

for i = 1:length(model.activeIDs)
    id = model.activeIDs(i);
    p = model.orders(id).xyz;
    plot3(p(1),p(2),p(3),'ko','MarkerFaceColor','y', ...
        'MarkerSize',6,'HandleVisibility','off');
    text(p(1)+1,p(2)+1,p(3)+1,sprintf('C%d',id));
end

if strcmp(mode,'all')
    points = reshape([model.orders.xyz],3,[])';
    scatter3(points(:,1),points(:,2),points(:,3),50, ...
        [0.95 0.75 0.1],'filled','DisplayName','Orders');
    overview.route = detail.route;
    overview.points = detail.points;
    overview.distance = detail.distance;
end

title(plotTitle);
legend('Location','best');
exportgraphics(gcf,filePath,'Resolution',150);
end
