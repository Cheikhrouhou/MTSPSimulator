%Script to generate statistic comparison result
%Date (17-11-2015)
% comparison between two multi-objective solution
%ahpGreedyImproved (Omar) and Fuzzy (Sahar)

function script_statistics_comparison_ahp_fuzzy(ni, nf, ns, stp, dim, showPlot)
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
ext='w2';

%weight=[0.2631  0.1897 0.5472  0  0];
weight=[0  1  0  0  0];
filename=strcat('comparison-ahpGreedy-Fuzzy-statistic1_3OF-ns30-w5', datestr(date,'yyyy-mm-dd'),'.txt');

%w3
% compMat=[1     2     1/3; 
%          1/2   1    1/2; 
%          3     2    1]
%w4
compMat=[0     0     0; 
         0   1    0; 
         0     0    0]
     
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
        [rtes, brk, tourcost1, approachCosts]=momdmtsp_ahpGreedyImproved7(Targets, Robots, compMat, showPlot)
        time1=toc
        %save in file
        save_result_motsp(Targets, Robots, weight, rtes, tourcost1, filename)
        dlmwrite('Greedyapproaches_comparison_3OF-w5.txt',[nt nr approachCosts], '-append','delimiter', '\t')
       
%         tic
%         [rtes, tourcost2]=FLMTSP(depots, Targets)
%         time2=toc
%         save_result_motsp(Targets, Robots, weight, rtes, tourcost2, filename)
%         
%         glbcost1=weight(1)*sum(tourcost1)+weight(2)*max(tourcost1)
%         glbcost2=weight(1)*sum(tourcost2)+weight(2)*max(tourcost2)
%         
%         line=[nt nr sum(tourcost1) max(tourcost1) glbcost1 sum(tourcost2) max(tourcost2) glbcost2]
%         dlmwrite('motsp_comparison_3OF-w2.txt', line, '-append', 'delimiter', ' ')
%         dlmwrite('time_comparison_3OF-w2.txt', [nt nr time1 time2], '-append', 'delimiter', ' ')
        
    end
end

    msgbox('Finished....!!!', 'Comparison scripts Execution');
    
end



