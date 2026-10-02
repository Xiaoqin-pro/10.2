function [cost,detail] = Fitness(position,model,state)
%FITNESS Evaluate the hybrid particle: order keys and XYZ control points.

N = model.nOrders;
routeKeys = position(1:N);
[~,order] = sort(routeKeys);
route = state.activeIDs(order);
controlVector = position(N+1:end);
control = permute(reshape(controlVector,3,model.nControlPoints,N+1),[3 2 1]);
paths = DecodeParticle(route,control,model,state.position);

currentTime = state.time;
totalDistance = 0;
totalLate = 0;
totalWaiting = 0;
obstacleViolation = 0;
angleViolation = 0;
smoothness = 0;
allPoints = state.position;
records = zeros(N,5);

for k = 1:N
    path = EvaluatePolyline(paths{k},model);
    id = route(k);
    travelTime = path.distance/model.speed;
    arrival = currentTime+travelTime;
    wait = max(0,model.orders(id).ready-arrival);
    serviceStart = arrival+wait;
    late = max(0,serviceStart-model.orders(id).due);

    totalDistance = totalDistance+path.distance;
    totalWaiting = totalWaiting+wait;
    totalLate = totalLate+late;
    obstacleViolation = obstacleViolation+path.obstacleViolation;
    angleViolation = angleViolation+path.angleViolation;
    smoothness = smoothness+path.smoothness;
    records(k,:) = [id arrival serviceStart late wait];
    allPoints = [allPoints;path.points(2:end,:)];
    currentTime = serviceStart+model.orders(id).service;
end

path = EvaluatePolyline(paths{end},model);
totalDistance = totalDistance+path.distance;
obstacleViolation = obstacleViolation+path.obstacleViolation;
angleViolation = angleViolation+path.angleViolation;
smoothness = smoothness+path.smoothness;
allPoints = [allPoints;path.points(2:end,:)];

cost = totalDistance+0.05*totalWaiting+model.smoothWeight*smoothness;
if totalLate>1e-8
    cost = cost+100000+1000*totalLate;
end
if obstacleViolation>1e-8
    cost = cost+100000+10000*obstacleViolation;
end
if angleViolation>1e-8
    cost = cost+100000+10000*angleViolation;
end

detail.route = route;
detail.control = control;
detail.points = allPoints;
detail.paths = paths;
detail.records = records;
detail.distance = totalDistance;
detail.totalLate = totalLate;
detail.totalWaiting = totalWaiting;
detail.obstacleViolation = obstacleViolation;
detail.angleViolation = angleViolation;
detail.smoothness = smoothness;
detail.feasible = totalLate<1e-8 ...
    && obstacleViolation<1e-8 && angleViolation<1e-8;
end

function result = EvaluatePolyline(points,model)
segments = diff(points,1,1);
horizontal = vecnorm(segments(:,1:2),2,2);
pitch = atan2d(segments(:,3),horizontal);
turn = zeros(max(0,size(segments,1)-1),1);
for i = 1:length(turn)
    a = segments(i,1:2); b = segments(i+1,1:2);
    turn(i) = atan2d(abs(a(1)*b(2)-a(2)*b(1)),dot(a,b));
end
angleViolation = sum(max(0,abs(pitch)-model.maxClimbAngle)) ...
    + sum(max(0,turn-model.maxTurnAngle));
smoothness = sum((turn/model.maxTurnAngle).^2) ...
    + sum((diff(pitch)/model.maxClimbAngle).^2);

obstacleViolation = 0;
for i = 1:size(segments,1)
    t = linspace(0,1,model.safetySamples)';
    samples = points(i,:)+t.*segments(i,:);
    ground = interp2(model.X,model.Y,model.terrainZ, ...
        samples(:,1),samples(:,2),'linear');
    for j = 1:length(model.obstacles)
        obs = model.obstacles(j);
        inside = hypot(samples(:,1)-obs.x,samples(:,2)-obs.y) ...
            <= obs.r+model.obstacleSafety ...
            & samples(:,3)>=obs.zMin-model.obstacleSafety ...
            & samples(:,3)<=obs.zMax+model.obstacleSafety;
        amount = max(0,obs.zMax+model.obstacleSafety-samples(:,3));
        obstacleViolation = obstacleViolation+sum(amount(inside));
    end
    obstacleViolation = obstacleViolation+ ...
        sum(max(0,model.minClearance-(samples(:,3)-ground)));
end
result.points = points;
result.distance = sum(vecnorm(segments,2,2));
result.obstacleViolation = obstacleViolation;
result.angleViolation = angleViolation;
result.smoothness = smoothness;
result.isFeasible = obstacleViolation<1e-8 && angleViolation<1e-8;
end
