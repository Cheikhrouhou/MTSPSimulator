%worst_target except depot
function  [worstTarget TourIndex minDist]=worst_target(Tour, depot)

    minDist=Inf;
    worstTarget=[];
    TourIndex=[1:size(Tour,1)];

    if size(Tour,1)>2
        for i=1:size(Tour,1)
                pt=Tour(i,:);
                if pt~=depot
                    newTour=remove_pt(pt, Tour);
  
                    [NewTourIndex, NewTourDist]=tsp_solver(newTour);
                    if NewTourDist<minDist
                        minDist=NewTourDist
                        worstTarget=pt
                        TourIndex=NewTourIndex
  
                    end
                end
        end
        
    elseif size(Tour,1)==2
        minDist=0;
        worstTarget=remove_pt(depot, Tour);
        TourIndex=1;
    end
    
   
end


