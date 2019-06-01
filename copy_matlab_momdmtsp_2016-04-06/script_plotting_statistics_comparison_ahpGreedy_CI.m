%Script to generate statistic comparison result
%Date (15-02-2015)
% comparison between the 3 Greedy approch 

function script_plotting_statistics_comparison_ahpGreedy_CI(ni, nf, ns, stp, dim, showPlot)

ext='_3OF-w3';
%
%w3=[0.2631  0.1897 0.5472
W=' W=\{0.26, 0.19, 0.54\}'
%W={TTD, MT, BT}';
%W=' W=\{1,0,0\}';
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
        
    AvgGA=mean_mat(GA,1)
   
     
     %compute the error
     for i=1:5
      ErGA(i,:)= 1.96*std(GA(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      ErGA(i,1:2)=GA(ns*i,1:2)      
      
     end
     
    
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
    ylabel(strcat('Global cost, ', W));
    
  
end

function R=mean_mat(mat,p)
R=[];
for n=[unique(mat(:,p))]'
    grp=mat(mat(:,p)==n,:);
    grp=mean(grp,p);
    R=vertcat(R,grp);
end
end



