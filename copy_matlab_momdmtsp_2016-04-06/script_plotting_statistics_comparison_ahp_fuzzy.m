%Script to generate statistic comparison result
%Date (17-11-2015)
% comparison between two multi-objective solution
%ahpGreedyImproved (Omar) and Fuzzy (Sahar)

function script_plotting_statistics_comparison_ahp_fuzzy(ni, nf, ns, stp, dim, showPlot)

%ext='_3OF';
ext='';
%%% result are in the follwoing filename %%%%%%%%%%%%%%
filename=strcat('comparison-ahpGreedy-Fuzzy-statistic1',ext, datestr(date,'yyyy-mm-dd'),'.txt');

% dlmwrite('Greedyapproaches_comparison.txt',approachCosts, '-append')
% dlmwrite('motsp_comparison.txt', line, '-append', 'delimiter', ' ')
% dlmwrite('time_comparison.txt', [time1 time2], '-append', 'delimiter', ' ')

  

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%  read data and plot curve   %%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clr=hsv(4);


    GA=dlmread(strcat('Greedyapproaches_comparison',ext,'.txt'))
    M=dlmread(strcat('motsp_comparison',ext,'.txt'))
    TR=dlmread(strcat('time_comparison',ext,'.txt'))
    
%     Rtma=M(M(:,1)==3,:)
%     Rtma=mean_mat(Rtma)
%     
%     Dist=M(M(:,1)==4,:)
%     Dist=mean_mat(Dist)
%     
     M=mean_mat(M,1)
     GA=mean_mat(GA,1)
     TR=mean_mat(TR,1)
     
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
    plot(GA(:,1),GA(:,4),'--','Color',clr(1,:))
    
    hold on
    plot(GA(:,1),GA(:,6),'--','Color',clr(2,:))
    
    hold on
    plot(GA(:,1),GA(:,8),'--','Color',clr(3,:))
    
    legend('Market Based','RTMA', 'Balanced')
    xlabel('Nbre of Targets');
    ylabel('Global cost lamda = {0.66;0.33}');
    
    %figure 2 : comp my sol with FLMTSP
    
    figure('name','Greedy approach(Omar) and FLMTSP(Sahar) comparison', 'number', 'off');
    hold on
    %Greedy result
    plot(M(:,1),M(:,3),'o-','Color',clr(1,:))
    
    hold on
    plot(M(:,1),M(:,4),'--','Color',clr(1,:))
     hold on
    plot(M(:,1),M(:,5),'+-','Color',clr(1,:))
    
    %FLMTSP result
    hold on
    plot(M(:,1),M(:,6),'o-','Color',clr(2,:))
        hold on
    plot(M(:,1),M(:,7),'--','Color',clr(2,:))
        hold on
    plot(M(:,1),M(:,8),'+-','Color',clr(2,:))
    
    
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



