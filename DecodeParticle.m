function paths = DecodeParticle(route,control,model,startPoint)
%DECODEPARTICLE Build XYZ polylines from route and XYZ control points.

nLegs = length(route)+1;
K = model.nControlPoints;
paths = cell(nLegs,1);
current = startPoint;
for i = 1:nLegs
    if i <= length(route)
        target = model.orders(route(i)).xyz;
    else
        target = model.depot;
    end
    paths{i} = [current;squeeze(control(i,:,:));target];
    current = target;
end
end
