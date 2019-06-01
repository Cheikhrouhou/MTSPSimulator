%function [min_dist,best_tour,generation] = mdmtsp_ga_mo2 (xy,max_salesmen,depots,CostType,weight,pop_size,num_iter,show_prog,show_res,dmat)
function [rte, brk, globalCost]=mdmtsp_greedy_mo(Targets, Robots,weight, show_prog)


%compute cost for each robot
%global   Targets Robots;
global  tourcost tour time energy param;
% global debug
% debug=  strcmp(get(findobj(gcf,'Type','uimenu','Label','Step-by-Step'), 'Checked'),'on')

clear tourcost;

nbrTargets=size(Targets, 1)
nbrRobots=size(Robots,1)
if ~(nbrTargets && nbrRobots)
    msgbox('Nbre of Targets and Nbre of Robots must not be zero','Change configuration','Warning');
    return;
end

%initialization
alpha1=1;
%[wd wmt wt we wv]=read_weight()
[wd wmt wt we wv]=deal(weight(1),weight(2), weight(3), weight(4), weight(5))
%unassigned targets firstly = all target
%weight=[wd wmt wt we wv]
param=[nbrTargets nbrRobots weight]

UTargets=Targets
for ri=1:nbrRobots
        
   tour{ri}=[Robots(ri,:)];
   tourcost(ri)=0;
   time(ri)=0;
   energy(ri)=0;
   speed(ri)=ri ;
   NbATargets(ri)=1;
   changeFlag(ri)=1;
end

%global nb_run Dim 
%plot final tour
if(show_prog)
    %create_newfigure();
    fig_name=sprintf('Multi-Robot Simulator, Result-%iT-%iR-%iwd-%iwmt-%iwt-%iwe-%iwv',param);%-%s, cputime

    newfig = figure('Name',fig_name,'Numbertitle','off','menubar', 'none');
    
    plot (Targets(:,1),Targets(:,2),'ro')
    hold on
    plot (Robots(:,1),Robots(:,2), 'd')
    
end

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
            [ind{ri}{ti},candtourcost(ti)]=tsp(candtour{ri}{ti},10,100,1,0);

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% TO BE DONE AT SERVER SIDE %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%based on bidding the Server select the one target to assign to a robot
%he selects the target with low cost
% [cost, ri]=min(bidding(:,2))
% ti=bidding(ri, 1)
%compute global cost correspending to each local cost
clr=hsv(nbrRobots);
for ri=1:nbrRobots        
   
   %compute the new max tour
   newmaxtour(ri)=max([tourcost,newtourcost(ri)])
   
   %compute the new time
   newmaxTime(ri)=max([time,newTime(ri)])
   %new energy
   
   %compute the new variance
   x=NbATargets
   x(ri)=newNbATargets(ri)
   newVariance(ri)=var(x)
   
   %global cost
   %Costglobal=?1distanceGlobal+ ?2EnegyGlobal + ?3MaxTime+ ?4MaxTour+?5Variance.
   newGlobalCost(ri)=wd*(sum(tourcost)-tourcost(ri)+newtourcost(ri))+ wmt*newmaxtour(ri)...
       +we*(sum(energy)-energy(ri)+newEnergy(ri))+wt*newmaxTime(ri)+wv*newVariance(ri)
   
end

%choose the best assignment (1 target)
[globalCost,winner]=min(newGlobalCost)

%remove the targets from list of available targets
%remove(UTargets(ti,:), UTargets)
%assignet
bidTarget(winner)=bidding(winner,1:1)
lastAllocatedTarget=bidTarget(winner)
UTargets(bidTarget(winner),:)=[];
%vertcat(tour{ri
    %the possible new tour of ri
    tourOrder=circshift(ind{winner}{bidTarget(winner)}, [0 -(find(ind{winner}{bidTarget(winner)}==1)-1)])
    %newtour{ri}=candtour{ri}{ti}(ind{ti},:)
    tour{winner}=candtour{winner}{bidTarget(winner)}(tourOrder,:) %newtour{winner}

changeFlag(winner)=1;
tourcost(winner)=newtourcost(winner)
NbATargets(winner)=newNbATargets(winner)

    %plot the 
    if show_prog
        try
            delete(tr(winner))
            tr(winner)=plot(tour{winner}(:,1),tour{winner}(:,2), 'Color',clr(winner,:))
        catch
            %winner
            tr(winner)=plot(tour{winner}(:,1),tour{winner}(:,2), 'Color',clr(winner,:))    
        end

    end
end

celldisp(tour)



for rj=1:nbrRobots
    %plot(BestTour{rj}(:,1),BestTour{rj}(:,2), 'Color',clr(rj,:))
    
    if show_prog
        plot(tour{rj}(:,1),tour{rj}(:,2), 'Color',clr(rj,:))
    end
    [logic, loc]=ismember(tour{rj},Targets)
    rte{rj}=loc(2:end,1)'
    if(rj==1)
        brk(rj)=size(rte{rj},2)
    elseif rj<nbrRobots
        brk(rj)=size(rte{rj},2)+brk(rj-1)
    end
end

%print statistics data
if show_prog
    title(sprintf('TTD = %1.3f \t MaxTour = %1.3f \t \n GlobalCost:%1.3f ',sum(tourcost),max(tourcost), globalCost));
    %set(pfig,'Name', sprintf('Final Solution: %s :Total Traveled Distance = %1.4f',cost_type,Cost_hung))
    %gl_cost=wd*sum(tourcost)+wmt*maxtour+we*sum(energy)+wt*maxTime+wv*Variance
    %gl_cost=wd*sum(tourcost)+wmt*newmaxtour(ri)+we*sum(energy)+wt*newmaxTime(ri)+wv*newVariance(ri)
end

end