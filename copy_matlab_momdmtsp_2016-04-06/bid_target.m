function [gain, bidtour1, bidtour2]=bid_target(tour1, tour2, target, weight, cost1, cost2)

%if globalcost not inputed we compute
[wd wmt wt we wv]=deal(weight(1),weight(2), weight(3), weight(4), weight(5));
if nargin<6
    [index1, cost1]=tsp_solver(tour1)
    [index2, cost2]=tsp_solver(tour2)    
    
end
globalcost=wd*(cost1+cost2)+wmt*max(cost1, cost2)
%if there is a target
if find(ismember(target, tour1))
    %tour1- target
    bidtour1=setdiff(tour1,target, 'rows')
    %to avoid adding same target twice
    target=setdiff(target, tour2, 'rows');
    bidtour2=vertcat(tour2,target)
elseif find(ismember(target, tour2))
    %tour2-target
    bidtour2=setdiff(tour2,target,'rows')
    
    target=setdiff(target, tour1, 'rows')
    bidtour1=vertcat(tour1,target)
else
    %not in tour1 not in tour 2
    bidtour1=vertcat(tour1,target)
    [bidindex1, bidcost1]=tsp_solver(bidtour1)
    bidglobalcost1=wd*(bidcost1+cost2)+wmt*max(bidcost1, cost2)
    
    bidtour2=vertcat(tour2,target)
    [bidindex2, bidcost2]=tsp_solver(bidtour2)
    bidglobalcost2=wd*(cost1+bidcost2)+wmt*max(cost1, bidcost2)
    
    gain= (bidglobalcost1-bidglobalcost2)
    return;
end

[bidindex1, bidcost1]=tsp_solver(bidtour1)
[bidindex2, bidcost2]=tsp_solver(bidtour2)

bidglobalcost=wd*(bidcost1+bidcost2)+wmt*max(bidcost1, bidcost2)

gain= (globalcost-bidglobalcost)
end