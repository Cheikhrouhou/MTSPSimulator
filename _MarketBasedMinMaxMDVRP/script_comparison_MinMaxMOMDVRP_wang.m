%%code that permits to compare our Market Based approach based on the
%%multi-objective mtsp code (adapted--1 objective MaxTour)
%%Compared to Wang approach approach described on paper
%%The min-max multi-depot vehicle routing problem: heuristics and computational results
%%using the same instances of this paper
%%....
% DATE: 06-04-2016

function script_comparison_MinMaxMOMDVRP_wang(covi, covf, ns, stp)




if nargin<4
    stp=30;
end
if nargin<3
    ns=10;
end
if nargin<2
    covf=100;
end
if nargin<1
    covi=10;
end

%Objective Maxtour only
%w5=0   1   0
compMat=[0     0     0; 
         0     1    0; 
         0     0    0]
weight=[0 1 0]
filename=strcat('comparison-MinMaxAHPGreedy-wang-ns30-', datestr(date,'yyyy-mm-dd'),'.txt');
for i=2:43 %i=[1 7 10]
    %read problem MMi conf
    [Targets Depots]=load_config_wang(strcat('MS', int2str(i)))                             
                                 
    %we use DIST as cost_type    
    %we optimize max tour
    
    tic
    %TourDist=mdmtsp_move_improve_multiCost(Targets, Robots,CommRange,MoveUnit,MaxT, DIST);
    %TourDist=mdmtsp_move_improve_multiCost_multiBidding(Targets, Depots,CommRange,MoveUnit,MaxT, 'cost_dist','RmvSelect');
    [rtes, brk, tourcost1, approachCosts]=momdmtsp_ahpGreedyImproved8(Targets, Depots, compMat, 0)
    telapsed=toc
  
    %best result from 3 approaches
    save_result_motsp(Targets, Depots, weight, rtes, tourcost1, min(approachCosts),filename)
    %all results
    dlmwrite(strcat('Greedyapproaches_comparison_3OF-','wangInstances','.txt'),...
        [size(Targets,1) size(Depots,1) approachCosts], '-append','delimiter', '\t')

       
end
  


   
 
 
 %%read data and plot curve
    msgbox('Execution Termin�','Information','help');
    %return;
    
 
end




