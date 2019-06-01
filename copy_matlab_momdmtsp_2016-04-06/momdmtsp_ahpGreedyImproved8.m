%Date: wed 16-02-2016
%here we make call to the three proposed approach as external functions 
%return
        %

function [Bestrte, Bestbrk, BestTourCost, approachCosts]=momdmtsp_ahpGreedyImproved8(Targets, depots, compMat, show_prog)


%compute cost for each robot
%
global  param;
%close all
clc
%clear tourcost;
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
            Targets = 100*rand(10,2);
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

 save_config_motsp(Targets, depots, weight, datestr(clock,'yyyy-mm-dd-HH-MM-SS'))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%   Market Based ALLOCATION    %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[Bestrte, Bestbrk, BestTourCost, approachCosts]=momdmtsp_MarketBased(Targets, depots, compMat, show_prog)

BestGlobalCost=min(approachCosts);
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%   BEGIN RTMA ALLOCATION    %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
 [RTMArte, RTMAbrk, RTMATourCost, RTMACosts]=momdmtsp_RTMA(Targets, depots, compMat, show_prog)
 
 approachCosts=[approachCosts, RTMACosts]
 if min(RTMACosts)< BestGlobalCost
     BestGlobalCost=min(RTMACosts)
     BestTourCost=RTMATourCost
     Bestrte=RTMArte
 end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%   Balanced ALLOCATION    %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 [BLrte, BLbrk, BLTourCost, BLCosts]=momdmtsp_Balanced(Targets, depots, compMat, show_prog)
approachCosts=[approachCosts, BLCosts]
 if min(BLCosts) <BestGlobalCost
     BestGlobalCost=min(BLCosts)
     BestTourCost=BLTourCost
     BestTour=BLrte
 end
 

%tile figures to window
if show_prog
    tilefig
end

end