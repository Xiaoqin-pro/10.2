function [Best,T] = PSO(model,maxgen,Particle_Number,seed)
%PSO Standard continuous PSO for order keys and XYZ control points.

rng(seed);

N = model.nOrders;
K = model.nControlPoints;
D = N+(N+1)*K*3;

%% Variable range
VarMin = zeros(1,D);
VarMax = ones(1,D);
VarMin(1:N) = 0;
VarMax(1:N) = 1;
for i = 1:N+1
    index = N+(i-1)*K*3+1;
    VarMin(index:index+K*3-1) = repmat([0 0 0],1,K);
    VarMax(index:index+K*3-1) = ...
        repmat([model.mapSize model.maxControlHeight],1,K);
end

%% PSO parameters
c = 2.0;
w = 0.9;
wdamp = 0.99;
Vmax = 0.2*(VarMax-VarMin);
Vmin = -Vmax;
sizepop = Particle_Number;

%% Initialization
pop = zeros(sizepop,D);
V = zeros(sizepop,D);
fitness = zeros(sizepop,1);
detail = cell(sizepop,1);

for i = 1:sizepop
    pop(i,:) = VarMin+rand(1,D).*(VarMax-VarMin);
    V(i,:) = Vmin+rand(1,D).*(Vmax-Vmin);

    % Decode order keys and initialize control points on the same route.
    [~,order] = sort(pop(i,1:N));
    route = model.activeIDs(order);
    control = zeros(N+1,K,3);
    current = model.depot;
    for j = 1:N+1
        if j <= N
            target = model.orders(route(j)).xyz;
        else
            target = model.depot;
        end
        for k = 1:K
            control(j,k,:) = current+k/(K+1)*(target-current);
        end
        current = target;
    end
    pop(i,N+1:end) = reshape(permute(control,[3 2 1]),1,[]);

    [fitness(i),detail{i}] = Fitness(pop(i,:),model);
end

%% pbest and gbest
[fitnessgbest,bestindex] = min(fitness);
pbest = pop;
fitnesspbest = fitness;
gbest = pop(bestindex,:);

%% PSO iteration
T = zeros(maxgen,1);
for G = 1:maxgen
    for i = 1:sizepop
        V(i,:) = w*V(i,:) ...
            + c*rand(1,D).*(pbest(i,:)-pop(i,:)) ...
            + c*rand(1,D).*(gbest-pop(i,:));
        V(i,:) = max(Vmin,min(Vmax,V(i,:)));

        pop(i,:) = pop(i,:)+V(i,:);
        pop(i,:) = max(VarMin,min(VarMax,pop(i,:)));

        [fitness(i),detail{i}] = Fitness(pop(i,:),model);

        if fitness(i) < fitnesspbest(i)
            fitnesspbest(i) = fitness(i);
            pbest(i,:) = pop(i,:);
        end

        if fitness(i) < fitnessgbest
            fitnessgbest = fitness(i);
            gbest = pop(i,:);
        end
    end

    T(G) = fitnessgbest;
    w = w*wdamp;
end

%% Best solution
Best.Position = gbest;
[Best.Cost,Best.Detail] = Fitness(gbest,model);
Best.Route = Best.Detail.route;
Best.Control = Best.Detail.control;
end
