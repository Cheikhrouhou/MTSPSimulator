%Script to generate statistic comparison result
%Date (15-02-2015)
% comparison between the 3 Greedy approch 

function script_plotting_statistics_comparison_ahpGreedyObjectives_CI(ni, nf, ns, stp, dim, showPlot)

%ext='_3OF-w5';
%W={TTD, MT, BT}';
%W=' W=\{0,1,0\}';

% ext='_3OF-w5';
% %W={TTD, MT, BT}';
% W=' W=\{0,1,0\}';
% 
% ext='_3OF-w5';
% %W={TTD, MT, BT}';
% W=' W=\{0,1,0\}';

%ns=30;
if nargin<3
    ns=30;
    %ns=1;
end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%  read data and plot curve   %%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clr=hsv(4);
   %fig1 greedy approach comaprion
    figure('name','Objectives functions comparison', 'number', 'off');
    
    %%%%%%%%%% w3=0.26  0.19    0.54 %%%%%%%%%%%%%%%%%%%%%%%%%%
        
    GA=dlmread(strcat('Greedyapproaches_comparison_3OF-w3.txt'))
    BestGA =   [GA(:,1), GA(:,2), min(GA(:,3:end),[],2)]
    AvgGA=mean_mat(BestGA,1)
        
     %compute the error
     for i=1:5
      ErGA(i,:)= 1.96*std(BestGA(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      ErGA(i,1:2)=BestGA(ns*i,1:2)      
      
     end   
 
    hold on
    errorbar(AvgGA(:,1),AvgGA(:,3),ErGA(:,3),'--','Color',clr(1,:))
    
    %W3--TTD, MT and BT
    ot=dlmread('TTD-MT-GC-BT-w3.txt')
    ot=[GA(:,1), GA(:,2), ot]
    Avgot=mean_mat(ot,1)
    
    %compute the error
     for i=1:5
      Erot(i,:)= 1.96*std(ot(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      Erot(i,1:2)=ot(ns*i,1:2)      
      
     end   
     
    hold on
    errorbar(Avgot(:,1),Avgot(:,3),Erot(:,3),'-','Color',clr(3,:))
    errorbar(Avgot(:,1),Avgot(:,4),Erot(:,4),'+-','Color',clr(3,:))
    %errorbar(Avgot(:,1),Avgot(:,5),Erot(:,5),'-','Color',clr(3,:))
    errorbar(Avgot(:,1),Avgot(:,6),Erot(:,6),'o-','Color',clr(3,:))
    
%    return;

    %%%%%%%%%%%%% w5= 0 1 0  %%%%%%%%%%%%%%%%%%%%%%%%%
    GA=dlmread(strcat('Greedyapproaches_comparison_3OF-w5.txt'))
    BestGA =   [GA(:,1), GA(:,2), min(GA(:,3:end),[],2)]
    AvgGA=mean_mat(BestGA,1)
        
     %compute the error
     for i=1:5
      ErGA(i,:)= 1.96*std(BestGA(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      ErGA(i,1:2)=BestGA(ns*i,1:2)      
      
     end   
 
    hold on
    %MT
    errorbar(AvgGA(:,1),AvgGA(:,3),ErGA(:,3),'--','Color',clr(1,:))
    
    %W5--TTD
    ot=dlmread('TTD-w5.txt')
    ot=[GA(:,1), GA(:,2), ot]
    Avgot=mean_mat(ot,1)
    
         %compute the error
          Erot=[];
     for i=1:5
      Erot(i,:)= 1.96*std(ot(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      Erot(i,1:2)=ot(ns*i,1:2)      
      
     end   
     
    hold on
    errorbar(Avgot(:,1),Avgot(:,3),Erot(:,3),'-','Color',clr(1,:))
    
    %GC=0.26  0.19 
    %GC=
     
%%%%%%%%%%  w4  =1  0   0   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    GA=dlmread(strcat('Greedyapproaches_comparison_3OF-w4.txt'))
    BestGA =   [GA(:,1), GA(:,2), min(GA(:,3:end),[],2)]
    AvgGA=mean_mat(BestGA,1)
        
     %compute the error
     for i=1:5
      ErGA(i,:)= 1.96*std(BestGA(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      ErGA(i,1:2)=BestGA(ns*i,1:2)      
      
     end   
 
    hold on
    errorbar(AvgGA(:,1),AvgGA(:,3),ErGA(:,3),'--','Color',clr(2,:))

 %w4--MT
    ot=dlmread('MT-w4.txt')
    ot=[GA(:,1), GA(:,2), ot]
    Avgot=mean_mat(ot,1)
    
         %compute the error
     for i=1:5
      Erot(i,:)= 1.96*std(ot(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      Erot(i,1:2)=ot(ns*i,1:2)      
      
     end   
     
    hold on
    errorbar(Avgot(:,1),Avgot(:,3),Erot(:,3),'-','Color',clr(2,:))

 
 %%%%%%%%%%%%%%%%% label %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 
 
    legend('Global cost(MT) w=\{0,1,0\}', 'TTD w=\{0,1,0\}', 'Global cost(TTD) w=\{1,0,0\}', 'MT w=\{1,0,0\}')
    %legend('Global cost', 'TTD')
    xlabel('Nbre of Targets (=3 x Nbre of Robots)');
    ylabel(strcat('Cost'));
    
  
end

function R=mean_mat(mat,p)
R=[];
for n=[unique(mat(:,p))]'
    grp=mat(mat(:,p)==n,:);
    grp=mean(grp,p);
    R=vertcat(R,grp);
end
end



