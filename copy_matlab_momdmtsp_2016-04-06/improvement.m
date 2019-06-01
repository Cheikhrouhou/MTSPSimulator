%updated from (of v6)
%we take consideration for new global cost
%added BT as objective function
function [wasImproved, tour, tourcost, globalCost]=improvement(Targets, depots, tour, tourcost, weight, show_prog, fgname)
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%                        Improvement based on bidding on worst targets
%Targets: set of targets
%tour:cell of tours
%wasImproved=1--> if tour are improved 0 otherwise
%show_prog: plot if 1
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Initialization
 nbrRobots=size(depots,1);
 nbrTargets=size(Targets ,1);
 clr=hsv(nbrRobots);
 options=struct('tsp_approach', 'lkh' );
 param=[nbrTargets nbrRobots weight];
 [wd wmt wt we wv]=deal(weight(1),weight(2), weight(3), weight(4), weight(5));
 %compute the present globalCost
 globalCost=wd*sum(tourcost)+wmt*max(tourcost)+wv*std(tourcost,1)
 %plot original status
 if(show_prog)
     scnsize = get(0,'ScreenSize');
     edge=30;
     pos1 = [scnsize(4)-edge, scnsize(3)/3-3*edge,...
        scnsize(3)/2 - 3*edge, scnsize(4)/2 - 3*edge];
    pos2 = [scnsize(4)-edge, edge*2,...
        pos1(3),  pos1(4)];
     %create_newfigure();
     fig_name=sprintf('%s Improvement phase : Worst Targets %s',fgname, num2str(param));%-%s, cputime
     
     figure('Name',fig_name,'Numbertitle','off','menubar', 'none', 'position', pos2);
     
     plot (Targets(:,1),Targets(:,2),'ro')
     hold on
     plot (depots(:,1),depots(:,2), 'd')
     for rj=1:nbrRobots
         tr(rj)= plot([tour{rj}(:,1); tour{rj}(1,1)],[tour{rj}(:,2); tour{rj}(1,2)], 'Color',clr(rj,:))
     end
     
     title(sprintf('TTD = %1.3f \t MaxTour = %1.3f \t \n GlobalCost:%1.3f ',sum(tourcost),max(tourcost), globalCost));
 end

for ri=1:nbrRobots
    worstTarget{ri}=[];
end

 %bidding worst target %for each robot
 wasImproved=0;
 improv=1;
 while improv
     improv=0;
     for ri=1:nbrRobots
        %compute worst target for ri
        if isempty(worstTarget{ri})
        [worstTarget{ri} BidTourIndex{ri,ri} BidTourDist(ri,ri) ]=worst_target(tour{ri}, depots(ri,:))        
        %bid{ri,ri}=[worstTarget{ri}, BidTourDist(ri,ri), energy(ri), time(ri), NbATargets(ri)]   
        bid{ri,ri}=[worstTarget{ri}, BidTourDist(ri,ri)]
        end
        %all robots send bid on this target
        for rj=setdiff(1:nbrRobots, ri)
            %if rj==ri, continue, end
                bidtour{rj,ri}=vertcat(tour{rj}, worstTarget{ri})
                %compute value of cost when adding the target to rj
                [BidTourIndex{rj,ri}, BidTourDist(rj,ri)]=tsp_solver(bidtour{rj,ri},options);
                %bid{rj,ri}=[worstTarget{ri}, BidTourDist(rj,ri), energy(rj), time(rj), NbATargets(rj)] 
                bid{rj,ri}=[worstTarget{ri}, BidTourDist(rj,ri)] 
        end
        
        %%compute resulted global cost
        for rj=1:nbrRobots        
   
           if rj==ri
               %globalcost is the same as old one
               newGlobalCost(ri)=globalCost
           else
           tc= tourcost;
           tc(ri)=BidTourDist(ri,ri)
           tc(rj)=BidTourDist(rj,ri)
           %compute the new max tour
           %rest_index=setdiff(1:nbrRobots, [ri rj])
           newmaxtour(rj)=max([tourcost(setdiff(1:end, [ri rj])), BidTourDist(rj,ri), BidTourDist(ri,ri)])
           
           %global cost
           %Costglobal=?1distanceGlobal+ ?2EnegyGlobal + ?3MaxTime+ ?4MaxTour+?5Variance.
           newGlobalCost(rj)=wd*sum(tc)+ wmt*newmaxtour(rj)+wv*std(tc,1)
           end

        end
        % choose the best robots to visit this target 
        
        [globalCost,winner]=min(newGlobalCost)
        if winner ~= ri
            tour{ri}=remove_pt(worstTarget{ri}, tour{ri})
            tour{ri}=tour{ri}(BidTourIndex{ri,ri},:)
            tourcost(ri)=BidTourDist(ri,ri)
            worstTarget{ri}=[];
            tour{winner}=bidtour{winner, ri}(BidTourIndex{winner,ri},:)
            tourcost(winner)=BidTourDist(winner,ri)
            worstTarget{winner}=[];
            improv=1
            wasImproved=1; %this is one time enough
            %update plot the new tour
            if show_prog
                try
                    delete(tr(winner))
                    delete(tr(ri))
                    tr(winner)=plot([tour{winner}(:,1); tour{winner}(1,1)],[tour{winner}(:,2); tour{winner}(1,2)], 'Color',clr(winner,:))                
                    tr(ri)= plot([tour{ri}(:,1); tour{ri}(1,1)],[tour{ri}(:,2); tour{ri}(1,2)], 'Color',clr(ri,:))
                catch
                    %winner
                    %tr(winner)=plot(tour{winner}(:,1),tour{winner}(:,2), 'Color',clr(winner,:)) 
                    tr(winner)=plot([tour{winner}(:,1); tour{winner}(1,1)],[tour{winner}(:,2); tour{winner}(1,2)], 'Color',clr(winner,:))                
                    tr(ri)= plot([tour{ri}(:,1); tour{ri}(1,1)],[tour{ri}(:,2); tour{ri}(1,2)], 'Color',clr(ri,:))
                end
                title(sprintf('TTD = %1.3f \t MaxTour = %1.3f \t \n GlobalCost:%1.3f ',...
                    sum(tourcost),max(tourcost), globalCost));
            end
        end

  
     end
 end
 
end