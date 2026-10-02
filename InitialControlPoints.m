function control = InitialControlPoints(route,model,startPoint)
%INITIALCONTROLPOINTS Straight interpolation with random PSO perturbation later.

nLegs = length(route)+1;
K = model.nControlPoints;
control = zeros(nLegs,K,3);
current = startPoint;
for i = 1:nLegs
    if i <= length(route)
        target = model.orders(route(i)).xyz;
    else
        target = model.depot;
    end
    for k = 1:K
        control(i,k,:) = current+k/(K+1)*(target-current);
    end
    current = target;
end
end
