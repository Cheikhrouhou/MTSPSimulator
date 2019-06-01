%%code that permits to compare our Market Based approach to Elad Market
%%approach
%%number of targets and fixed number of robots
%%based on script_mdmtsp_multiCost

% DATE: 30-03-2016

function script_comparison_MinMaxMDVRP_wang(covi, covf, ns, stp)
%Input
%cov
%
%
global Targets
global negociate_type negociate_all negociate_pt_by_pt
global optimize_type optimize_total_distance optimize_max_tour;

 negociate_all=1;
 negociate_pt_by_pt=2;
 negociate_type=negociate_all;
 

optimize_total_distance=1;
optimize_max_tour=2;
optimize_type=optimize_max_tour; % Min Max Problem

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

Dim=1000;
global MoveUnit;
MoveUnit=30;
DIST=1;
RTMA=2;
RTMAM=3;
TTD=1;
MaxT=2;

%parameters of GA
pop_size = 160; %default value 80
num_iter = 1e3; %default value 1e3

for i=1:43
    %read problem MMi conf
    [Targets Depots]=load_config_wang(strcat('MS', int2str(i)))                             
                                 
    %we use DIST as cost_type    
    %we optimize max tour
    
    tic
    %TourDist=mdmtsp_move_improve_multiCost(Targets, Robots,CommRange,MoveUnit,MaxT, DIST);
    TourDist=mdmtsp_move_improve_multiCost_multiBidding(Targets, Depots,CommRange,MoveUnit,MaxT, 'cost_dist','RmvSelect');
    telapsed=toc
  
    
    Line=[DIST,MaxT,nbrRobots,nbrTargets,CommRange, MoveUnit,telapsed, cpuelapsed,...
        sum(TourDist), max(TourDist), TourDist]
    dlmwrite(improvement_filename, Line, '-append', 'delimiter', '\t');
       
   end
  


   
 
 
 %%read data and plot curve
    msgbox('Execution Terminé','Information','help');
    %return;
    
 
end




