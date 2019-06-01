function flmtsp(hObject, eventdata,pfig)
global Targets Robots;
global clr Dim;
global nbrTargets  
global nbrRobots
global Dist_target_target %matrix distances between targets
global Dist_robots_targets %matrix distances between robots and targets
Dist_target_target = zeros(nbrTargets,nbrTargets); 
Dist_robots_targets = zeros(nbrRobots,nbrTargets);
 
%read targets and robots positions
nbrTargets=size(Targets, 1)
nbrRobots=size(Robots,2)

if ~(nbrTargets && nbrRobots)
    msgbox('Nbre of Targets and Nbre of Robots must not be zero','Change configuration','Warning');
    return;
end
     
%new figure
%Plot Targets and initial Position
pfig = figure('Name','FLMTSP Solution','Numbertitle','off');
%Position
scnsize = get(0,'ScreenSize');
pos=get(pfig,'position');

plot (Targets(:,1),Targets(:,2),'ro') 
hold on
plot (Robots(1,:),Robots(2,:), 'd') 
axis([0 Dim 0 Dim])
%Initially

clr=hsv(nbrRobots+1);
CurrentPositions=Robots;
for ri=1:nbrRobots
 
    hTxt(ri) = text(CurrentPositions(1,ri), CurrentPositions(2,ri), ... 
        sprintf('R%i:(%.1f,%.1f)', ri,CurrentPositions(1,ri), CurrentPositions(2,ri)), ...
    'Color',clr(ri,:), 'FontSize',12, ...
    'HorizontalAlignment','left', 'VerticalAlignment','top');
    
end
%% FMOMTS algorithm
      %[Matching, tourCost] =  FMOMTSP(Robots, Targets);

       
 %%%construct matrix of distances
 for ri = 1 : nbrRobots
         dist=cost_dist(reshape(Robots(:,ri),1,2),Targets);%return one column with nbrTargets line
         Dist_robots_targets(ri,:)=reshape(dist,1,nbrTargets);
 end 
   
 for ti = 1 : nbrTargets
     dist=cost_dist(Targets(ti,:),Targets);%return one column with nbTargets line(Targets(ti,:),Targets);
     Dist_target_target(ti,:)=reshape(dist,1,nbrTargets);
 end
 
 %%%compute thresholds values for the fuzzy logic system
[maxMin, minMax, meanMin, meanMax] = buildMFValues_greedy();
sumMaxMinMinMax = maxMin+minMax;
sumMeanMinMeanMax = meanMin+meanMax;
%%%declaration
tour = cell(1, nbrRobots);%the list of tours of each robot (targets)
newTour = cell(1, nbrRobots);
tourCost = zeros(1,nbrRobots);%total distance traveled by a robot, the tour cost 
oldTourCost = zeros(1,nbrRobots);%save the cost before addind the new target
cost = zeros(1,nbrRobots);
totalTourCostPerRobot = zeros(1,nbrRobots);

maxTourCost = 0;
totalTourCost=0;%initially totalTour cost is 0
%% Phase 1: task assignment 
%%Step 1: using fyzzy logic system

%add the task t to the tour of each robot
for ti=1:nbrTargets
    for ri=1:nbrRobots
        [tourCost(ri)] = computeTourCost_greedy(ri, ti, tour);
        totalTourCostPerRobot(ri)= computeTotalTourCost(ri, tourCost, oldTourCost);
       
         cost = oldTourCost;
         cost(ri) = tourCost(ri);
       
        [M, I]=max(cost);
    
        %fuzzy logic part
        %build the output of the combination between objectives
         totalTourCostPerRobot(ri)
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
    
    for ri=1:nbrRobots
        if(ri ~= index)
            totalTourCostPerRobot(ri) = totalTourCostPerRobot(index);
            tourCost(ri) = oldTourCost(ri);
        else
            oldTourCost(ri)=tourCost(ri);
        end
        
    end

    totalTourCost = totalTourCostPerRobot(index); 
    
    figure(pfig);
    title(sprintf('Assignment phase, step1'));
    set(pfig,'Name', sprintf('FLMTSP Solution'))
    currentX=Robots(1, index);
    currentY=Robots(2, index);
    xT = Targets(ti,1);
    yT = Targets(ti, 2);
    
    pause(1)
    
    plot([currentX xT],[currentY yT],'-', 'Color',clr(1,:))
            
    %plotting traveled distance (hTxt)
    txt(ti)=text(xT,yT,sprintf('T%i',ti ),  'Color',clr(index,:),...
    'FontSize',12,'HorizontalAlignment','right', 'VerticalAlignment','top') 
%        txt(index)=text(currentX,currentY,sprintf('R%i',index ),  'Color',clr(index,:),...
%     'FontSize',12,'HorizontalAlignment','right', 'VerticalAlignment','top') 
%     set(hTxt(ri), 'Position',[Robots(1, index) Robots(2, index)], ...
%     'String',sprintf('R%i:(%.1f)',index,tourCost(index)),'FontSize',12)
%     drawnow
    
end

pause(1)
%plot result after step1
tfig = figure('Numbertitle','off');
scnsize = get(0,'ScreenSize');
hold on
plot (Targets(:,1),Targets(:,2),'ro') %
hold on
plot (Robots(1,:),Robots(2,:), 'd') %, 'rs'

%Plotting purpose     
title(sprintf('Assignment phase, step1\n Total Tour cost = %1.3f \t Maximum Tour cost = %1.3f ',...
    sum(tourCost),max(tourCost)));
set(pfig,'Name', sprintf('FLMTSP Solution'))

for ri=1:nbrRobots
    if(~isempty(tour{ri}))
        % xR = Robots(1, ri);
        %    yR = Robots(2, ri);
        currentX=Robots(1, ri);
        currentY=Robots(2, ri);
        for ti = 1 : length(tour{ri})
            xT = Targets(tour{ri}(ti,1),1);
            yT = Targets(tour{ri}(ti,1), 2);
            hold on
            pause(1)
            plot([currentX xT],[currentY yT],'-', 'Color',clr(ri,:))
            currentX=xT;
            currentY=yT;
            txt(ti)=text(xT,yT,sprintf('T%i',tour{ri}(ti,1) ),  'Color',clr(ri,:),...
    'FontSize',12,'HorizontalAlignment','right', 'VerticalAlignment','top') 
        end
        pause(1)
        plot([currentX Robots(1, ri)],[currentY Robots(2, ri)],'-', 'Color',clr(ri,:))
        
               
        %plotting traveled distance (hTxt)
         txt(ri)=text(Robots(1, ri), Robots(2, ri),sprintf('R%i:(%.1f)',ri,tourCost(ri)),  'Color',clr(ri,:),...
    'FontSize',12,'HorizontalAlignment','right', 'VerticalAlignment','top') 
%         set(hTxt(ri), 'Position',[Robots(1, ri) Robots(2, ri)], ...
%                    'String',sprintf('R%i:(%.1f)',ri,tourCost(ri)))
%                drawnow
    else
        txt(ri)=text(Robots(1, ri), Robots(2, ri),sprintf('R%i:(%.1f)',ri,tourCost(ri)),  'Color',clr(ri,:),...
    'FontSize',12,'HorizontalAlignment','right', 'VerticalAlignment','top') 
    end
end

%%Step2: check if the targets are well distributed between the robots
avg_targets_per_robot = nbrTargets/nbrRobots;% average number of targets per robot

len = cellfun('length',tour);%nb element in each cell array
[B,I] = sort (len,'descend');
% [maxcellsize,maxcellind] = max(cellfun(@numel,tour));
% maxcellsize
% maxcellind
for i = 1 : nbrRobots
    ri = I(i);
    list_targets = tour{ri};
    
    l = length(list_targets);
    ri
    while( l > avg_targets_per_robot)
        %give the farthest target to the other robots.
        [farTarget, index] = select_farthest_target(tour,ri, list_targets);
       
        nearRobot = select_closest_robot(farTarget);
%         nearRobot = select_closest_robot_m(farTarget,ri);
        farTarget
        nearRobot
        %length(list_targets)
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

% for ri=1:nbrRobots
% tourCost(ri) = computeTourCost(ri, tour);
% end


tfig = figure('Numbertitle','off');
scnsize = get(0,'ScreenSize');
%plot result after step2
 
% plot (Targets(:,1),Targets(:,2),'ro') %
% hold on
% plot (Robots(1,:),Robots(2,:), 'd') %, 'rs'
for ri=1:nbrRobots
    if(~isempty(tour{ri}))
        % xR = Robots(1, ri);
        %    yR = Robots(2, ri);
        currentX=Robots(1, ri);
        currentY=Robots(2, ri);
       
        for ti = 1 : length(tour{ri})
            xT = Targets(tour{ri}(ti),1);
            yT = Targets(tour{ri}(ti), 2);
            hold on
            plot([currentX xT],[currentY yT],'-', 'Color',clr(ri,:))
            currentX=xT;
            currentY=yT;
            txt(ti)=text(xT,yT,sprintf('T%i',tour{ri}(ti) ),  'Color',clr(ri,:),...
       'HorizontalAlignment','right', 'VerticalAlignment','top') 
        end
        hold on
        plot([currentX Robots(1, ri)],[currentY Robots(2, ri)],'-', 'Color',clr(ri,:))

    end
       txt(ri)=text(Robots(1,ri),Robots(2,ri),sprintf('R%i',ri ),  'Color',clr(ri,:),...
       'HorizontalAlignment','right', 'VerticalAlignment','top')  
   
end
hold on
plot (Targets(:,1),Targets(:,2),'ro') %
hold on
plot (Robots(1,:),Robots(2,:), 'd') %, 'rs'
%Plotting purpose     
title(sprintf('Assignment phase, step2'));
set(tfig,'Name', sprintf('FLMTSP Solution'))
pause(1)

%%
%% Phase 2: tour construction

[assignment, tourCost] = robotTourConstruction(tour);
totalTourCost = sum(tourCost);
[maxTourCost, I]=max(tourCost);
%% writeStatisticsWord(tour, tourCost, totalTourCost, maxTourCost, tElapsed);
writeStatisticsExcel(1, tour, tourCost, totalTourCost, maxTourCost, 1);
%%
for ri = 1:nbrRobots
%     ri
%     assignment{ri}
%     tour{ri}
    nb = 0;
    pos = find(assignment{ri}==1);
    
    for j=(pos+1):length(assignment{ri})
         nb=nb+1;
         newTour{ri}(nb,1) = tour{ri}(assignment{ri}(j)-1);
     end
    for j=1:pos-1
        nb=nb+1;
        newTour{ri}(nb,1) = tour{ri}(assignment{ri}(j)-1);
    end
end
%%Plot results
tfig = figure('Numbertitle','off');
scnsize = get(0,'ScreenSize');
%plot result after phase 2
 
for ri=1:nbrRobots
    if(~isempty(newTour{ri}))
        % xR = Robots(1, ri);
        %    yR = Robots(2, ri);
        currentX=Robots(1, ri);
        currentY=Robots(2, ri);
       
        for ti = 1 : length(newTour{ri})
            xT = Targets(newTour{ri}(ti),1);
            yT = Targets(newTour{ri}(ti), 2);
            hold on
            plot([currentX xT],[currentY yT],'-', 'Color',clr(ri,:))
            currentX=xT;
            currentY=yT;
            txt(ti)=text(xT,yT,sprintf('T%i',newTour{ri}(ti) ),  'Color',clr(ri,:),...
       'HorizontalAlignment','right', 'VerticalAlignment','top') 
        end
        hold on
        plot([currentX Robots(1, ri)],[currentY Robots(2, ri)], 'Color',clr(ri,:))

    end
       txt(ri)=text(Robots(1,ri),Robots(2,ri),sprintf('R%i',ri ),  'Color',clr(ri,:),...
       'HorizontalAlignment','right', 'VerticalAlignment','top')  
   
end
hold on
plot (Targets(:,1),Targets(:,2),'ro') %
hold on
plot (Robots(1,:),Robots(2,:), 'd') %, 'rs'
%Plotting purpose     
title(sprintf('Tour Construction phase \n Total Tour cost = %1.3f \t Maximum Tour cost = %1.3f ',...
    totalTourCost,maxTourCost));
set(tfig,'Name', sprintf('FLMTSP Solution'))

 
 %%   %%%plotting result
% % %     cost =0;
% % %     for ri=1:nbrRobots
% % %         if(~isempty(Matching{ri}))
% % %             xR = Robots(1, ri);
% % %             yR = Robots(2, ri);
% % %             currentX=Robots(1, ri);
% % %             currentY=Robots(2, ri);
% % %             for ti = 1 : length(Matching{ri})
% % %                 xT = Targets(Matching{ri}(ti,1),1);
% % %                 yT = Targets(Matching{ri}(ti,1), 2);
% % %                 plot([currentX xT],[currentY yT], 'Color',clr(ri,:))
% % %                 currentX=xT;
% % %                 currentY=yT;
% % %             end
% % %              plot([currentX xR],[currentY yR], 'Color',clr(ri,:))
% % %              hold on
% % %              %plot Tour Distance
% % % %    txt(ri)=text(Robots(ri,1),Robots(ri,2),sprintf(' (%.3f)',tourCost(ri)), 'Color',clr(ri,:),...
% % % %        'HorizontalAlignment','left', 'VerticalAlignment','top')
% % %         end
% % %     end
% % %        
% % %     title(sprintf('Total Tour cost = %1.3f \t Maximum Tour cost = %1.3f',sum(tourCost),max(tourCost)));
% % %     set(pfig,'Name', sprintf('FMOMTS Solution'))
% % %  
end

function [cost] = computeTourCost_greedy(ri, ti, tourRobot)
global Dist_target_target
global Dist_robots_targets
len = length(tourRobot{ri})+1;
tourRobot{ri}(len,1) = ti;
%construct matrix
D = zeros(len+1, len+1); 

for j = 1 : len
    
 D(1,j+1)=Dist_robots_targets(ri, (tourRobot{ri}(j,1)));
 D(j+1,1) = D(1,j+1);
end

for l = 2 : len+1 %
    for c = 2 : len+1 
    D(l,c)=Dist_target_target(tourRobot{ri}(l-1,1),tourRobot{ri}(c-1,1));
    D(c,l)=D(l,c);
    end
end
%D
[p, cost] = greedy(1,D);

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
