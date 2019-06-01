%function that returns the routes (index of targets T1 T5 ..)
%input : tour (assigned targets coordinates )
%Targets: 
function [rte, brk]=return_rte(tour, Targets)

nbrRobots=size(tour, 1)
for rj=1:nbrRobots
    [logic, loc]=ismember(tour{rj},Targets, 'rows')
    rte{rj}=loc(loc ~=0)'
    if(rj==1)
        brk(rj)=size(rte{rj},2)
    elseif rj<nbrRobots
        brk(rj)=size(rte{rj},2)+brk(rj-1)
    end
end
end