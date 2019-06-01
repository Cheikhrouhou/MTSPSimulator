% This is the main function
%Select the thresholds using a greedy approach
function FLMTSP%(xy, depots)
disp('--Inside Main Function--');
global nbTargets 
global nbRobots
global Dist_target_target
global Dist_robots_targets
nbTargets =30;%[30, 70, 100, 200, 300]
nbRobots = 3;%[3, 10, 20, 30, 100]

filename1 = 'Dist_target_target.txt';
Dist_target_target=importdata(filename1);%import the matrix of distances between targets
filename2 = 'Dist_robots_targets.txt';
Dist_robots_targets=importdata(filename2);%import the matrix of distances between robots and targets
%%%%%%%%%%%%%%%%%%%%%

[maxMin, minMax, meanMin, meanMax] = buildMFValues_greedy();
sumMaxMinMinMax = maxMin+minMax;
sumMeanMinMeanMax = meanMin+meanMax;

for iter=1:30
%iter
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
      
        [tourCost(ri)] = computeTourCost_greedy(ri, ti, tour);
        totalTourCostPerRobot(ri)= computeTotalTourCost(ri, tourCost, oldTourCost);
       
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
            if(Dist_robots_targets(robot_min_distance, ti)>Dist_robots_targets(tab(i), ti))
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


%% check if the targets are well distributed between the robots
avg_targets_per_robot = nbTargets/nbRobots;% average number of targets per robot

for ri = 1 : nbRobots
    list_targets = tour{ri};
    
    l = length(list_targets);
  
    while( l > avg_targets_per_robot)
        %give the farthest target to the other robots.
        [farTarget, index] = select_farthest_target(tour,ri, list_targets);
       
        nearRobot = select_closest_robot(farTarget);
        
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
 
%% Step 2: tour construction
[tour, tourCost] = tourConstruction(tour);
%% stop time
 tElapsed = toc(tStart); 
 %% compute total tour cost and select max tour cost
totalTourCost = sum(tourCost);
[maxTourCost, I]=max(tourCost);

%% writeStatisticsWord(tour, tourCost, totalTourCost, maxTourCost, tElapsed);
writeStatisticsExcel(iter, tour, tourCost, totalTourCost, maxTourCost, tElapsed);
pause(2);%pauses for 1 second
end
disp('--End Main Function--');
end
function [tour, tourCost]=tourConstruction(tour)
global nbRobots
tourCost = zeros(1,nbRobots);
for ri=1:nbRobots
    
    if(~isempty(tour{ri}))
        [tourCost(ri), varargout] = tsp_ga_m(ri, tour{ri});
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
   
function [farT, index] = select_farthest_target(tour,I, list_kept_targets)
global Dist_robots_targets
vectD = zeros(1,length(list_kept_targets));
for ti=1:length(list_kept_targets)
   vectD(ti) = Dist_robots_targets(I, list_kept_targets(ti));
end
[dist, index]=max(vectD);
farT= tour{I}(index);
end

function nearR = select_closest_robot(target)
global nbRobots
global Dist_robots_targets
vect = zeros(1,nbRobots);
for ri=1:nbRobots
    vect(1,ri) = Dist_robots_targets(ri,target);
end

[M, nearR] = min(vect);

end
function writeStatisticsExcel(iteration, tour, tourCost, totalTourCost, maxTourCost, tElapsed)
global nbRobots
 global nbTargets
  
 
 sheetName =  strcat('R', num2str(nbRobots),'_T', num2str(nbTargets));
 fileName = 'new_results_FLMTSP';
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
%     for ti = 1: length(tour{ri})
%         xlswrite(fileName,tour{ri}(ti,1),sheetName, range2);
%     end
%    range3=strcat('C', num2str(ri));
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

function [cost] = computeTourCost_greedy(ri, ti, tourRobot)
global Dist_target_target
global Dist_robots_targets
len = length(tourRobot{ri})+1;
tourRobot{ri}(len,1) = ti;
%construct matrix
D = zeros(len+1, len+1); 
% for i = 1 : len
%     D(i,i)=0;
% end

for j = 1 : len
    
 D(1,j+1)=Dist_robots_targets(ri, (tourRobot{ri}(j,1)));
 D(j+1,1) = D(1,j+1);
end

for l = 2 : len+1 %
       % if(i~=j)
    for c = 2 : len+1 
    D(l,c)=Dist_target_target(tourRobot{ri}(l-1,1),tourRobot{ri}(c-1,1));
    D(c,l)=D(l,c);
    end
end
%D
[p, cost] = greedy(1,D);
%p
%D(1,p(len+1))
cost = cost + D(1,p(len+1));
end

function [cost] = computeTotalTourCost(ri, costTourRobot, oldTourCost)
 global nbRobots
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
