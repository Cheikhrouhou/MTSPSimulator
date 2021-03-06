%Script to generate statistic comparison result
%Date (17-11-2015)
% comparison between two multi-objective solution
%ahpGreedyImproved (Omar) and Fuzzy (Sahar)

function script_plotting_statistics_comparison_ahp_fuzzy_CI(ni, nf, ns, stp, dim, showPlot)

ext='_3OF-w5';
%ns=30;
if nargin<3
    ns=30;
    %ns=1;
end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%  read data and plot curve   %%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clr=hsv(4);


    GA=dlmread(strcat('Greedyapproaches_comparison',ext,'.txt'))
    M=dlmread(strcat('motsp_comparison',ext,'.txt'))
    TR=dlmread(strcat('time_comparison',ext,'.txt'))
    

     AvgM=mean_mat(M,1)
     AvgGA=mean_mat(GA,1)
     AvgTR=mean_mat(TR,1)
     
     %compute the error
     for i=1:5
      ErGA(i,:)= 1.96*std(GA(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      ErGA(i,1:2)=GA(ns*i,1:2)
      
      ErM(i,:)= 1.96*std(M(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      ErM(i,1:2)=M(ns*i,1:2)
      
      ErTR(i,:)= 1.96*std(TR(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      ErTR(i,1:2)=TR(ns*i,1:2)
      
     end
     
     %figure time comparison
      figure('name','Time comparison', 'number', 'off');
    hold on
    plot(TR(:,1),TR(:,3),'--','Color',clr(1,:))
    
    hold on
    plot(TR(:,1),TR(:,4),'--','Color',clr(2,:))
    
    legend('Market Based','FLMTSP')
    xlabel('Nbre of Targets');
    ylabel('Execution Time');
    
    %fig1 greedy approach comaprion
    figure('name','Greedy approach comparison', 'number', 'off');
    hold on
    errorbar(AvgGA(:,1),AvgGA(:,4),ErGA(:,4),'--','Color',clr(1,:))
    
%     cent=[AvgGA(:,4)-ErGA(:,4), AvgGA(:,4),AvgGA(:,4)+ErGA(:,4)];
%     bar(cent(:,1),AvgGA(:,4))
    
    hold on
    errorbar(AvgGA(:,1),AvgGA(:,6),ErGA(:,6),'--','Color',clr(2,:))
    
    hold on
    errorbar(AvgGA(:,1),AvgGA(:,8),ErGA(:,8),'--','Color',clr(3,:))
    
    legend('Market Based','RTMA', 'Balanced')
    xlabel('Nbre of Targets');
    ylabel('Global cost lamda = {0.66;0.33}');
    
    %hold on
    %errorbar(AvgGA(:,1),AvgGA(:,4),ErGA(:,4))
    
    %figure 2 : comp my sol with FLMTSP
    
    figure('name','Greedy approach(Omar) and FLMTSP(Sahar) comparison', 'number', 'off');
    hold on
    %Greedy result
    errorbar(AvgM(:,1),AvgM(:,3),ErM(:,3),'o-','Color',clr(1,:)) 
    hold on
    errorbar(AvgM(:,1),AvgM(:,4),ErM(:,4),'--','Color',clr(1,:))
     hold on
    errorbar(AvgM(:,1),AvgM(:,5),ErM(:,5),'+-','Color',clr(1,:))
    
    %FLMTSP result
    hold on
    errorbar(AvgM(:,1),AvgM(:,6),ErM(:,6),'o-','Color',clr(2,:))
        hold on
    errorbar(AvgM(:,1),AvgM(:,7),ErM(:,7),'--','Color',clr(2,:))
        hold on
    errorbar(AvgM(:,1),AvgM(:,8),ErM(:,8),'+-','Color',clr(2,:))
    
    
    legend('TTD-Market Based','MT-Market Based', 'Global Cost-Market Based', ...
        'TTD-Fuzzy','MT-Fuzzy', 'Global Cost-Fuzzy')
    %legend('Market Based', 'Fuzzy')
    xlabel('Nbre of Targets');
    ylabel('TTD, MaxTour and Global costs');
    
    
    return ;
 
    
end

function R=mean_mat(mat,p)
R=[];
for n=[unique(mat(:,p))]'
    grp=mat(mat(:,p)==n,:);
    grp=mean(grp,p);
    R=vertcat(R,grp);
end
end



