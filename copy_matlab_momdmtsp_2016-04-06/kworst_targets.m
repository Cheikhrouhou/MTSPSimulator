%worst_target except depot
function  [kworstTargets worst_ind cost_value]=kworst_targets(Tour, depot)

    %minDist=Inf;
    kworstTargets=[];
   % TourIndex=[1:size(Tour,1)];
   % cost_table=[];
    if size(Tour,1)>2
        %we take target 
        Targets=setdiff(Tour, depot,'rows');
        for i=1:size(Targets,1)
                pt=Targets(i,:);
                %if pt~=depot
                newTour=remove_pt(pt, Tour);
                
                [NewTourIndex, NewTourDist(i)]=tsp_solver(newTour);
                    
                   % cost_table=[cost_table;i NewTourDist(i)]
  
                %end
        end
        %%% sot target from worst to best
        [cost_value, worst_ind]=sort(NewTourDist)
        kworstTargets=Targets(worst_ind,:)
    elseif size(Tour,1)==2
       
        kworstTargets=remove_pt(depot, Tour);
        worst_ind=1;
        cost_value=0;
    end
    
   
end


