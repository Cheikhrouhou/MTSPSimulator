%Date: wed 16-02-2016
%Function of the Market based approach

function [rte, brk, tourcost, approachCosts]=momdmtsp_MarketBased(Targets, depots, compMat, show_prog)


%compute cost for each robot
%
global  tourcost tour time energy param;

globalCost=Inf;
approachCosts=[];
%%%%%%%% screen scize%%%%%%%%%%%%%%%%%
scnsize = get(0,'ScreenSize');
edge=30;
pos1 = [scnsize(4)-edge, scnsize(3)/3-3*edge,...
        scnsize(3)/2 - 3*edge, scnsize(4)/2 - 3*edge];
pos2 = [scnsize(4)-edge, edge*2,...
        pos1(3),  pos1(4)];

% 
% scrsz=scrsz-[0  0   160 160]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k = nargin:3
    switch k
        case 0
            global  Targets ;
            Targets = 100*rand(15,2);
        case 1
            global  Robots;
            nbrRobots=3;
            depots = 100*rand(nbrRobots,2); 
            Robots=depots';
        case 2
%             compMat=[1  1/2 3
%                      2  1   2
%                      1/3 1/2 1];
             compMat=[1  1/2   1/3; 
                      2   1    1/2; 
                      3   2    1]
        case 3
            show_prog=1;
        otherwise
    end
end

nbrTargets=size(Targets, 1)
nbrRobots=size(depots,1)
if ~(nbrTargets && nbrRobots)
    msgbox('Nbre of Targets and Nbre of Robots must not be zero','Change configuration','Warning');
    return;
end

%initialization
clr=hsv(nbrRobots);
%alpha1=1;
%[wd wmt wt we wv]=read_weight()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Computing weight based on AHP %%%%%%%%%%

 w=ahp(compMat);
 %weight=[w' zeros(1,5-size(w))]
weight=[w(1),w(2), 0, 0, w(3)]
%unassigned targets firstly = all target
%weight=[wd wmt wt we wv]
param=[nbrTargets nbrRobots weight];
[wd wmt wt we wv]=deal(weight(1),weight(2), weight(3), weight(4), weight(5));

%global nb_run Dim 
%plot final tour
if(show_prog)
    %create_newfigure();
    fig_name=sprintf('Market-Based Allocation, Result-%iT-%iR-%iwd-%iwmt-%iwt-%iwe-%iwv',param);%-%s, cputime

    newfig = figure('Name',fig_name,'Numbertitle','off','menubar', 'none', 'position', pos1);
    
    plot (Targets(:,1),Targets(:,2),'ro')
    hold on
    plot (depots(:,1),depots(:,2), 'd')
    
end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%  GREEDY APPROAH ALLOCATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%     Market Based            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Initialization
for ri=1:nbrRobots
        
   tour{ri}=[depots(ri,:)];
   tourcost(ri)=0;
   time(ri)=0;
   energy(ri)=0;
   speed(ri)=ri ;
   NbATargets(ri)=1;
   changeFlag(ri)=1;
end
%case of 1 robot is tsp (lkh) problem
if nbrRobots==1
    tour{1}=vertcat(Targets,depots);
    [tourindex, tourcost(1)]=tsp_solver(tour{1});
    tour{1}=tour{1}(tourindex,:);
    globalCost=(sum(tourcost)); %wd*
%     globalCost=wd*(sum(tourcost))+ wmt*newmaxtour(1)...
%        +we*(sum(energy)-energy(ri)+newEnergy(ri))+wt*newmaxTime(ri)+wv*newVariance(ri)
   
    if show_prog
        plot([tour{1}(:,1); tour{1}(1,1)],[tour{1}(:,2);tour{1}(1,2)], 'Color','red');
     
        title(sprintf('TTD = %1.3f \t MaxTour = %1.3f \t \n GlobalCost:%1.3f ',...
                    sum(tourcost),max(tourcost), globalCost));
    end
    
    rte{1}=tourindex;
    brk(1)=size(rte{1},2);
    BestTourCost=tourcost;
     %, globalCost]    
    
    return;
end
%case of more than 1 robot
UTargets=Targets;
%UTargets=[];
options=struct('tsp_approach', 'lkh' );
while(UTargets)
%each robots compute Costlocal=?1distance+ ?2Enegy + //?3Time  
%and send a bid for the selected targets
for ri=1:nbrRobots
    %distance
    %distance=cost_dist(Robots(:,ri)', Targets)
    %compute new tour costs when getting this target
    %if tour of ri changed
    if (changeFlag(ri))
        costTable{ri}=[];
        candtour{ri}=[];
        changeFlag(ri)=0;
        for ti=1:size(UTargets,1)
            %candidate tour      
            candtour{ri}{ti}=vertcat(tour{ri}, UTargets(ti,:));
            [ind{ri}{ti},candtourcost(ti)]=tsp_solver(candtour{ri}{ti},options);

            costTable{ri}=vertcat(costTable{ri}, [ti, candtourcost(ti)]);
        end
        
       %choose the best targets

        [newtourcost(ri), ti]=min(costTable{ri}(:,2))
        
        %energy
        newEnergy(ri)=newtourcost(ri)/2
        %Time
        newTime(ri)=speed(ri)*newtourcost(ri)
        %localcost
        %costTable{ri}(:,2)=alpha1*costTable{ri}(:,2)
        %number of asigned targets
        newNbATargets(ri)=size(tour{ri},1)+1;

        %make attention between target and index
        bidding(ri,:)=[ti, newtourcost(ri), newEnergy(ri), newTime(ri), newNbATargets(ri)]        
    else %remove the allocated targets
        
        candtour{ri}(lastAllocatedTarget)=[];
        costTable{ri}(lastAllocatedTarget,:)=[];
        ind{ri}(lastAllocatedTarget)=[];
        bidTarget(ri)=bidding(ri,1:1)
        
        if bidTarget(ri)==lastAllocatedTarget
            %search for other target
            %choose another  best targets

                [newtourcost(ri), ti]=min(costTable{ri}(:,2))

                %energy
                newEnergy(ri)=newtourcost(ri)/2
                %Time
                newTime(ri)=speed(ri)*newtourcost(ri)
                %localcost
                %costTable{ri}(:,2)=alpha1*costTable{ri}(:,2)
                %number of asigned targets
                newNbATargets(ri)=size(tour{ri},1)+1;

                %make attention between target and index
                bidding(ri,:)=[ti, newtourcost(ri), newEnergy(ri), newTime(ri), newNbATargets(ri)]   
  
        elseif(bidTarget(ri)>lastAllocatedTarget)
            %only ti index changed
            bidTarget(ri)=bidTarget(ri)-1;
            bidding(ri,:)=[bidTarget(ri), newtourcost(ri), newEnergy(ri), newTime(ri), newNbATargets(ri)]  
        end
    end
 
    
end
%%%%%%%%%%%%%%%%%%%%%% TO BE DONE AT SERVER SIDE %%%%%%%%%%%%%%%%%%%%%%%%
%based on bidding the Server select the one target to assign to a robot
%he selects the target with low cost
%compute global cost correspending to each tour local cost

for ri=1:nbrRobots        
   
     
   tc=tourcost;
   tc(ri)=newtourcost(ri)
   newVariance(ri)=std(tc,1)
   
   %global cost
   %Costglobal=?1distanceGlobal+ ?2EnegyGlobal + ?3MaxTime+ ?4MaxTour+?5Variance.
   newGlobalCost(ri)=wd*sum(tc)+ wmt*max(tc)+wv*std(tc,1)...
       %+we*(sum(energy)-energy(ri)+newEnergy(ri))+wt*newmaxTime(ri)
   
end

%choose the best assignment (1 target)
[globalCost,winner]=min(newGlobalCost)

%remove the targets from list of available targets
%remove(UTargets(ti,:), UTargets)
%assignet
bidTarget(winner)=bidding(winner,1:1)
lastAllocatedTarget=bidTarget(winner)
UTargets(lastAllocatedTarget,:)=[];
%vertcat(tour{ri
    %the possible new tour of ri
    tourOrder=circshift(ind{winner}{bidTarget(winner)}, [0 -(find(ind{winner}{bidTarget(winner)}==1)-1)])
    %newtour{ri}=candtour{ri}{ti}(ind{ti},:)
    tour{winner}=candtour{winner}{bidTarget(winner)}(tourOrder,:) %newtour{winner}

changeFlag(winner)=1;
tourcost(winner)=newtourcost(winner)
NbATargets(winner)=newNbATargets(winner)

    %update plot the new tour
    if show_prog
        try
            delete(tr(winner))
            tr(winner)=plot([tour{winner}(:,1); tour{winner}(1,1)],[tour{winner}(:,2); tour{winner}(1,2)], 'Color',clr(winner,:))
        catch
            %winner
            tr(winner)=plot([tour{winner}(:,1); tour{winner}(1,1)],[tour{winner}(:,2); tour{winner}(1,2)], 'Color',clr(winner,:))    
        end
    title(sprintf('TTD = %1.3f \t MaxTour = %1.3f \t \n GlobalCost:%1.3f ',sum(tourcost),max(tourcost), globalCost));
   pause(0.1)
    end
end

celldisp(tour)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% END OF GREEDY APPROACH 
%%%%%%%%%%%%%%%%      AND BEGIN OF
%%%%%%%%%%%%%%%%      Improvement 	%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        approachCosts=[approachCosts, globalCost]
[wasImproved, tour, tourcost, globalCost]=improvement(Targets, depots, tour, tourcost, weight, show_prog, 'Market-Based Allocation')
        approachCosts=[approachCosts, globalCost]
        %BestTour=tour;
        

%%% Return variables rtes%%%
[rte, brk]=return_rte(tour, Targets)


end

