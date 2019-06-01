function [maxM, minM, meanMi, meanMa] = buildMFValues_greedy(Robots,Targets)
nbRobots = length(Robots);
%nbTargets = length(Targets);
costNearestNeighbor = cell(1,nbRobots);
costFarthestNeighbor = cell(1,nbRobots);
% filename = 'graph.txt';
% matrixDistances=importdata(filename);%import the matrix of distances between robots and targets
 
for ri=1:nbRobots
%     disp(['robot number ' num2str(ri)]);
    costNearestNeighbor{ri} = greedyNearestNeighbor_modified(ri,Robots,Targets);
end
% for ri=1:nbRobots
%     disp(['costNearestNeighbor:' num2str(costNearestNeighbor{ri})]);
% end
maxM= max(cell2mat(costNearestNeighbor));
% maxM= min(cell2mat(costNearestNeighbor));
% disp(['maxMin=' num2str(maxM)]);

meanMi=mean(cell2mat(costNearestNeighbor));
% disp(['meanMin=' num2str(meanMi)]);

for ri=1:nbRobots
%     disp(['robot number ' num2str(ri)]);
    costFarthestNeighbor{ri} = greedyFarthestNeighbor_modified(ri,Robots,Targets);
end
% for ri=1:nbRobots
%     disp(['costFarthestNeighbor:' num2str(costFarthestNeighbor{ri})]);
% end
minM= min(cell2mat(costFarthestNeighbor));
% minM= max(cell2mat(costFarthestNeighbor));
% disp(['minMax=' num2str(minM)]);

meanMa=mean(cell2mat(costFarthestNeighbor));
% disp(['meanMax=' num2str(meanMa)]);

end