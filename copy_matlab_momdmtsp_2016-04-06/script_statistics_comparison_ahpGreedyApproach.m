%Script to generate statistic comparison result
%Date (17-02-2015)
% comparison between two multi-objective solution based on new version 8
%separation between file code of each approach

function script_statistics_comparison_ahpGreedyApproach(ni, nf, ns, stp, dim, showPlot)
%ni: initial number 
%nf: final number 
%ns: number of scenario
%stp: step
%global Targets

if nargin<5
    showPlot=false;
end
if nargin<5
    dim=1000;
end
if nargin<4
    stp=1;
end
if nargin<3
    ns=30;
    %ns=1;
end
if nargin<2
    nf=10;
end
if nargin<1
    ni=1;
end
ext='v8-w3';

%;

%Original weight (as paper) W=0.16342,0.29696,0,0,0.53961


filename=strcat('comparison-ahpGreedy-Fuzzy-statistic1_3OF-ns30-',ext,'-', datestr(date,'yyyy-mm-dd'),'.txt');
%W = 0.1634    0.2970         0         0    0.5396.
%    compMat=[1  1/2   1/3; 
%            2   1    1/2; 
%            3   2    1]

%w2=0.5472      0.1897      0.2631
% compMat=[1     2     3; 
%          1/2   1    1/2; 
%          1/3     2    1]
     
%w3=[0.2631  0.1897 0.5472  0  0]
%TTD=2*MT
compMat=[1     2     1/3; 
         1/2   1    1/2; 
         3     2    1]

%w4=1   0   0   
% compMat=[1     0     0; 
%          0     0    0; 
%          0     0    0]

%w5=0   1   0
% compMat=[0     0     0; 
%          0     1    0; 
%          0     0    0]

%w6=0   0   1     
% compMat=[0     0     0; 
%          0     0    0; 
%          0     0    1]
weight=ahp(compMat) 
weight=weight'
for nr=[3 5 10 15 20] %ni:stp:nf
    nt=3*nr;
    for s=1:ns
        
        Robots=random('unif',0,dim,nr,2);
        Targets=random('unif',0,dim,nt,2);%nt=4*nr
        fname=sprintf('s%d',ns);
        save_config_motsp(Targets, Robots, weight, fname)
        
        %call to functions
        depots=Robots;
        tic
        [rtes, brk, tourcost1, approachCosts]=momdmtsp_ahpGreedyImproved8(Targets, Robots, compMat, showPlot)
        time1=toc
        %save in files
        save_result_motsp(Targets, Robots, weight, rtes, tourcost1, min(approachCosts),filename)
        dlmwrite(strcat('Greedyapproaches_comparison_3OF-',ext,'.txt'),[nt nr approachCosts], '-append','delimiter', '\t')
       

        
    end
end

    msgbox('Finished....!!!', 'Comparison scripts Execution');
    
end



