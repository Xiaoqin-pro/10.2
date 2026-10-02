function [BestSol,BestCost] = PSO(model,state,maxgen,Particle_Number,seed)
%PSO Standard continuous PSO for order keys and XYZ control points.

rng(seed);
N = model.nOrders;
K = model.nControlPoints;
D = N+(N+1)*K*3;
VarMin = zeros(1,D);
VarMax = ones(1,D);
VarMin(1:N) = 0;
VarMax(1:N) = 1;
for i = 1:N+1
    start = N+(i-1)*K*3+1;
    VarMin(start:start+K*3-1) = repmat([0 0 0],1,K);
    VarMax(start:start+K*3-1) = repmat([model.mapSize model.maxControlHeight],1,K);
end
Vmax = 0.2*(VarMax-VarMin);
Vmin = -Vmax;
w = 0.9;
wdamp = 0.99;
c1 = 2;
c2 = 2;

empty.Position = [];
empty.Velocity = [];
empty.Cost = [];
empty.Detail = [];
empty.Best.Position = [];
empty.Best.Cost = [];
empty.Best.Detail = [];
particle = repmat(empty,Particle_Number,1);
GlobalBest.Cost = inf;

for i = 1:Particle_Number
    particle(i).Position = VarMin+rand(1,D).*(VarMax-VarMin);
    route = state.activeIDs(randperm(N));
    control = InitialControlPoints(route,model,state.position);
    particle(i).Position(N+1:end) = control(:)';
    particle(i).Velocity = Vmin+rand(1,D).*(Vmax-Vmin);
    [particle(i).Cost,particle(i).Detail] = ...
        Fitness(particle(i).Position,model,state);
    particle(i).Best = particle(i);
    if particle(i).Best.Cost<GlobalBest.Cost
        GlobalBest = particle(i).Best;
    end
end

BestCost = zeros(maxgen,1);
for it = 1:maxgen
    for i = 1:Particle_Number
        particle(i).Velocity = w*particle(i).Velocity ...
            + c1*rand(1,D).*(particle(i).Best.Position-particle(i).Position) ...
            + c2*rand(1,D).*(GlobalBest.Position-particle(i).Position);
        particle(i).Velocity = max(Vmin,min(Vmax,particle(i).Velocity));
        particle(i).Position = particle(i).Position+particle(i).Velocity;
        particle(i).Position = max(VarMin,min(VarMax,particle(i).Position));
        [particle(i).Cost,particle(i).Detail] = ...
            Fitness(particle(i).Position,model,state);
        if particle(i).Cost<particle(i).Best.Cost
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;
            particle(i).Best.Detail = particle(i).Detail;
        end
        if particle(i).Best.Cost<GlobalBest.Cost
            GlobalBest = particle(i).Best;
        end
    end
    BestCost(it) = GlobalBest.Cost;
    w = w*wdamp;
end

BestSol = GlobalBest;
N = model.nOrders;
[~,order] = sort(BestSol.Position(1:N));
BestSol.Route = state.activeIDs(order);
BestSol.Control = reshape(BestSol.Position(N+1:end),[],K,3);
end
