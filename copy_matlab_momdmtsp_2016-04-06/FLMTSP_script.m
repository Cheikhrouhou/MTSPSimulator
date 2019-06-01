% This is the main function
%Select the thresholds using a greedy approach
function [rte, tourCost]=FLMTSP_script(Robots, Targets)

nbTargets =length(Targets);
nbRobots = length(Robots);

[maxMin, minMax, meanMin, meanMax] = buildMFValues_greedy(Robots,Targets);
sumMaxMinMinMax = maxMin+minMax;
sumMeanMinMeanMax = meanMin+meanMax;

tour = cell(1, nbRobots);%the list of tours of each robot (targets)
newTour = cell(1, nbRobots);
tourCost = zeros(1,nbRobots);%total distance traveled by a robot, the tour cost 
oldTourCost = zeros(1,nbRobots);%save the cost before addind the new target
cost = zeros(1,nbRobots);
totalTourCostPerRobot = zeros(1,nbRobots);

maxTourCost = 0;
totalTourCost=0;%initially totalTour cost is 0
 tStart = tic; 

 %% Step 1: task assignment using fyzzy logic system

%add the task t to the tour of each robot
for ti=1:nbTargets
    for ri=1:nbRobots
      
        [tourCost(ri)] = computeTourCost_greedy(ri, ti, tour, Robots, Targets);
        totalTourCostPerRobot(ri)= computeTotalTourCost(ri, tourCost, oldTourCost, nbRobots);
       
         cost = oldTourCost;
         cost(ri) = tourCost(ri);
       
        [M, I]=max(cost);
    
        %fuzzy logic part
        %build the output of the combination between objectives
        output(ri) =fuzzyLogicSystem_mamdani(maxMin, minMax, meanMin, meanMax, sumMaxMinMinMax, sumMeanMinMeanMax, totalTourCostPerRobot(ri),M);
    end
   
    %select the best output
    [bestOutput, index]= min (output);
    
    %%improvement step. 
    [nb, tab] = computeOccurence(bestOutput, output);
    if (nb>1)
        %select the robot with the min distance
        robot_min_distance = tab(1);
        index =robot_min_distance;
        for i = 2 : nb
            dist1=cost_dist(Robots(robot_min_distance,:),Targets(ti,:));%return one column with nbTargets line
            dist2=cost_dist(Robots(tab(i),:),Targets(ti,:));
            if(dist1>dist2)
            %if(Dist_robots_targets(robot_min_distance, ti)>Dist_robots_targets(tab(i), ti))
                if((totalTourCostPerRobot(tab(i))<tourCost(index))||(totalTourCostPerRobot(tab(i))<totalTourCostPerRobot(index)))
                index =tab(i);
                end
            end
        end
    end

   
    len=length(tour{index})+1;
    tour{index}(len,1)= ti; 
    %modify tourCost of the other robot
    
    for ri=1:nbRobots
        if(ri ~= index)
            totalTourCostPerRobot(ri) = totalTourCostPerRobot(index);
            tourCost(ri) = oldTourCost(ri);
        else
            oldTourCost(ri)=tourCost(ri);
        end
        
    end

    totalTourCost = totalTourCostPerRobot(index); 
    
end


%% step2: check if the targets are well distributed between the robots
avg_targets_per_robot = nbTargets/nbRobots;% average number of targets per robot

for ri = 1 : nbRobots
    list_targets = tour{ri};
    
    l = length(list_targets);
  
    while( l > avg_targets_per_robot)
        %give the farthest target to the other robots.
        [farTarget, index] = select_farthest_target(tour,ri, list_targets, Robots, Targets);
       
        nearRobot = select_closest_robot(farTarget, Robots, Targets);
        
        length(list_targets)
        if(nearRobot ~= ri)
            %add the target to the list of nearRobot
            [tour] = addTarget(tour, nearRobot, farTarget);
            %delete the target from the list of robot ri
            [tour] = removeTarget(tour, ri, farTarget);
            list_targets(index) = [];
            l=l-1;
        else
            list_targets(index) = [];
            l=l-1;
        end
        
    end 
end
 
%% phase 2: tour construction
 rte=tour;
[tour, tourCost] = tourConstruction(tour, Robots, Targets);
for ri=1:nbRobots
    rte{ri}=reshape(rte{ri}, 1, [])
    %zero for depot
    rte{ri}(end+1)=0;
    rte{ri}=rte{ri}(tour{ri})
end
%% stop time
 tElapsed = toc(tStart); 
 %% compute total tour cost and select max tour cost
totalTourCost = sum(tourCost);
[maxTourCost, I]=max(tourCost);

%% writeStatisticsWord(tour, tourCost, totalTourCost, maxTourCost, tElapsed);

% writeStatisticsExcel(1, tour, tourCost, totalTourCost, maxTourCost, tElapsed, nbRobots, nbTargets);
% pause(2);%pauses for 1 second


end
function [tour, tourCost]=tourConstruction(tour, Robots, Targets)
%global nbRobots
nbRobots = length(tour);
tourCost = zeros(1,nbRobots);
for ri=1:nbRobots
    
    if(~isempty(tour{ri}))
        [tourCost(ri), varargout] = tsp_ga_modified(ri, tour{ri}, Robots, Targets);
        tour{ri} = varargout;
    end
    
end
end
function [tour] = removeTarget(tour, I, farT)
%delete the target
i = [tour{I}] == farT;
tour{I}(i) = [];
end

function [tour] = addTarget(tour, nearR, farT)
l=length(tour{nearR});%tour length of robot nearR

tour{nearR}(l+1) = farT;
end
   
function [farT, index] = select_farthest_target(tour,I, list_kept_targets, Robots, Targets)

vectD = zeros(1,length(list_kept_targets));
    for ti=1:length(list_kept_targets)
     vectD(ti)=cost_dist((Robots(I, :)),Targets(list_kept_targets(ti),:));
    end
[dist, index]=max(vectD);
farT= tour{I}(index);
end

function nearR = select_closest_robot(target, Robots, Targets)

%global Robots
nbRobots = length(Robots);
vect = zeros(1,nbRobots);
for ri=1:nbRobots
    vect(1,ri) = cost_dist(Robots(ri,:),Targets(target,:));
end

[M, nearR] = min(vect);

end
function writeStatisticsExcel(iteration, tour, tourCost, totalTourCost, maxTourCost, tElapsed, nbRobots,nbTargets)  
 
 sheetName =  strcat('R', num2str(nbRobots),'_T', num2str(nbTargets));
 fileName = 'FLMTSP';
 if(iteration ==1)
     xlswrite(fileName,{'robot'},sheetName, 'A1');
     xlswrite(fileName,{'tourCost'},sheetName, 'B1');
     xlswrite(fileName,{'totalTourCost'},sheetName, 'D1');
     xlswrite(fileName,{'maxTourCost'},sheetName, 'F1');
     xlswrite(fileName,{'tElapsed'},sheetName, 'H1');
      xlswrite(fileName,{'nbTargetsPerRobot'},sheetName, 'J1');
 end
 %iter=strcat('A', num2str(iteration));
 for ri=1:nbRobots
     n1 = ri+(nbRobots*(iteration-1))+1;
     range1=strcat('A', num2str(n1));
    xlswrite(fileName,ri,sheetName,range1);
    range2=strcat('B', num2str(n1));
xlswrite(fileName,tourCost(ri),sheetName, range2);
    range6=strcat('J', num2str(n1));
    if(~isempty(tour{ri}))
        xlswrite(fileName,length(tour{ri})-1,sheetName, range6);
    else
        xlswrite(fileName,length(tour{ri}),sheetName, range6);
    end
 end
  n2 = (nbRobots*(iteration-1))+3;
 range3=strcat('D', num2str(n2));
 xlswrite(fileName,totalTourCost,sheetName,range3);
 range4=strcat('F', num2str(n2));
 xlswrite(fileName,maxTourCost,sheetName, range4);
 range5=strcat('H', num2str(n2));
 xlswrite(fileName,tElapsed,sheetName,range5);
    
end

function [cost] = computeTourCost_GA(ri, ti, tourRobot)
len = length(tourRobot{ri})+1;
tourRobot{ri}(len,1) = ti;
[cost, varargout] = tsp_ga_m(ri, tourRobot{ri});
end

function [cost] = computeTourCost_greedy(ri, ti, tourRobot, Robots, Targets)
%global Robots Targets
len = length(tourRobot{ri})+1;
tourRobot{ri}(len,1) = ti;
%construct matrix
D = zeros(len+1, len+1); 

for j = 1 : len
    D(1,j+1)=cost_dist((Robots(ri,:)),Targets(tourRobot{ri}(j,1),:));
 D(j+1,1) = D(1,j+1);
end

for l = 2 : len+1 %
       % if(i~=j)
    for c = 2 : len+1 
         D(l,c)=cost_dist(Targets(tourRobot{ri}(l-1,1),:),Targets(tourRobot{ri}(c-1,1),:));
   D(c,l)=D(l,c);
    end
end
%D
[p, cost] = greedy(1,D);
%p
%D(1,p(len+1))
cost = cost + D(1,p(len+1));
end

function [cost] = computeTotalTourCost(ri, costTourRobot, oldTourCost, nbRobots)
 cost = 0;
 for i=1:nbRobots
     if (i==ri)
         cost = cost + costTourRobot(i); 
     else
         cost = cost + oldTourCost(i);
     end
 end
end

function [nb, tab] = computeOccurence(element, vector)

%tab = zeros(0);
nb=0;
for i=1:length(vector)
    if(vector(i)==element)
        nb= nb+1;
        tab(1,nb) = i;
    end
end

end

function [p,cost] = greedy(s,D)
%GREEDY Travel to nearest neighbour, starting with node s.

n = size(D,1);
p = zeros(1,n,'uint16');
p(1) = s;
cost = 0;
for k = 2:n
    D(s,:) = inf;
    [junk,s] = min(D(:,s)); %#ok
    p(k) = s;
   cost = cost + junk;
end
end
