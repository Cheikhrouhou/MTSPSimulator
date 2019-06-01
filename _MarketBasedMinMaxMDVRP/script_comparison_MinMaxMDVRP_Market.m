%%code that permits to compare our Market Based approach to Elad Market
%%approach
%%number of targets and fixed number of robots
%%based on script_mdmtsp_multiCost

% DATE: 30-03-2016

function script_comparison_MinMaxMDVRP_Market(covi, covf, ns, stp)
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



% improvement_filename=strcat('mdmtsp-move_MultiCost_NewCost3',date, '.txt');
% fid_move=fopen(improvement_filename, 'w');

%Line=['Cost','Optimize','nbrRobots','nbrTargets','CommRange', 'MoveUnit','telapsed', 'cpuelapsed',...
%                                   'sum(TourDist)', 'max(TourDist)', 'TourDist_1...ri'];
%dlmwrite(improvement_filename, Line, '-append');

% fprintf(fid_move,'%s\t %s %s %s %s %s %s %s %s %s %s\n', ...
%     'Cost','Optimize','nbrRobots','nbrTargets','CommRange', 'MoveUnit','telapsed', 'cpuelapsed',...
%      'sum(TourDist)', 'max(TourDist)', 'TourDist_1...ri');
%Ct Opt NR  NT  ComR MU  Time    cpuTim  TTD     MaxTour TourDist_1...ri 
% fprintf(fid_move,'%s %s %s %s %s %s %s %s %s %s %s\n', ...
%     'Ct','Opt','NR ','NT ','ComR', 'MU ','Time  ', 'CpuT    ',...
%      'TTD    ', 'MaxTour', 'TourDist_1...ri'); 
%                               
% mdmtsp_filename=strcat('mdmtsp_ga-popSize160',date, '.txt');
% fid_ga=fopen(mdmtsp_filename, 'w');
% 
%                  %full comm range
%                    CommRange=1000     

for nbrRobots =[5, 10, 20, 30, 100] %
     
     for nbrTargets=[50, 100, 200, 300]       
        
         %scenario 
        for ns=1:30
                  Targets=random('unif',0,Dim,nbrTargets,2);
                  Robots=random('unif',0,Dim,2,nbrRobots);
                  Depots=Robots';
                  
                  %put conf in file
                  param=[size(Robots,1) size(Targets,1) ] %we can add other param
                  scriptName = mfilename('fullpath')
                  [currentpath, filename, fileextension]= fileparts(scriptName)
                
                 file_name=sprintf('d%iv%ic%i.txt',size(Depots,1), size(Depots,1), size(Targets,1));
                 %filename=fullfile(currentpath, 'scenario', file_name);
                 filename=file_name
                 save_config_mdmtsp(Depots, Targets, filename)
               %%%%%%%%%%%%%%%%%%%%% CALL TO Maket Main Elad %%%%%%%%%%%%
                
                %!MarketMain.exe filename
                MarketMain='MarketMain.exe'
                system([MarketMain ' ' filename]);
                %%%%%%%%%%%% CALL To MinMaxMarket Proposed
        end
     end
end


% 
%                       for MoveUnit=[10, 50]                       
%                                  
%                           %we use DIST as cost_type
%                             
%                               %we optimize max tour
%                               tdeb=cputime
%                               tic
%                               %TourDist=mdmtsp_move_improve_multiCost(Targets, Robots,CommRange,MoveUnit,MaxT, DIST);
%                               TourDist=mdmtsp_move_improve_multiCost_multiBidding(Targets, Robots,CommRange,MoveUnit,MaxT, 'cost_dist','RmvSelect');                              
%                               toc
%                               telapsed=toc
%                               cpuelapsed=cputime-tdeb
%     %                           fprintf(fid_move,'%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\n',nbrTargets,nbrRobots,CommRange,...
%     %                               sum(TourDist), max(TourDist), telapsed, cpuelapsed,2,DIST);
% 
%                               Line=[DIST,MaxT,nbrRobots,nbrTargets,CommRange, MoveUnit,telapsed, cpuelapsed,...
%                                    sum(TourDist), max(TourDist), TourDist]
%                               dlmwrite(improvement_filename, Line, '-append', 'delimiter', '\t');                         
%        
%                       end
%   
% 
% 
%              end
%         
%      end
%  end
    
 
 
 %%read data and plot curve
    msgbox('Execution Terminé','Information','help');
    %return;
    
 
end




