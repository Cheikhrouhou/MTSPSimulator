%Script to generate statistic comparison result
%Date (15-02-2015)
% comparison between the 3 Greedy approch 

function script_plotting_statistics_comparison_subplot(ni, nf, ns, stp, dim, showPlot)

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

w1=0.2631
w2=0.1897
w3=0.5472

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%  read data and plot curve   %%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clr=hsv(4);
   %fig1 greedy approach comaprion
    figure('name','Objectives functions comparison', 'number', 'off');
    
    %%%%%%%%%% w3=0.26  0.19    0.54 %%%%%%%%%%%%%%%%%%%%%%%%%%
        
     GA=dlmread(strcat('Greedyapproaches_comparison_3OF-w3.txt'))

    %W3--TTD, MT and GC
    ot=dlmread('TTD-MT-GC-BT-w3.txt')
    ot=[GA(:,1), GA(:,2), ot]
    Avgot=mean_mat(ot,1)
    
    %compute the error
     for i=1:5
      Erot(i,:)= 1.96*std(ot(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      Erot(i,1:2)=ot(ns*i,1:2)      
      
     end   
     
    
    %subplot(1,3,1)
    hold on
    errorbar(Avgot(:,1),Avgot(:,3),Erot(:,3),'-.','Color','black')
    errorbar(Avgot(:,1),Avgot(:,4),Erot(:,4),'--','Color','black')
    errorbar(Avgot(:,1),Avgot(:,5),Erot(:,5),'-','Color','black')
    errorbar(Avgot(:,1),Avgot(:,6),Erot(:,6),'o-','Color','black')
    
%    return;
%%%%%%%%%%  w4  =1  0   0   %%%%%%%%%%%%%%%%%%%%%%%%%%%%


    GA=dlmread(strcat('Greedyapproaches_comparison_3OF-w4.txt'))


    ot=dlmread('TTD-MT-BT-GC-w4.txt')
    ot=[GA(:,1), GA(:,2), ot]
    Avgot=mean_mat(ot,1)
    
    %compute the error
     for i=1:5
      Erot(i,:)= 1.96*std(ot(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      Erot(i,1:2)=ot(ns*i,1:2)      
      
     end   
     
    %
    %subplot(1,3,2)
    hold on
    errorbar(Avgot(:,1),Avgot(:,3),Erot(:,3),'-.','Color','g')
    errorbar(Avgot(:,1),Avgot(:,4),Erot(:,4),'--','Color',clr(2,:))
    errorbar(Avgot(:,1),Avgot(:,5),Erot(:,5),'-','Color',clr(2,:))
    errorbar(Avgot(:,1),Avgot(:,6),Erot(:,6),'o-','Color',clr(2,:))
    
 legend('TTD, w=\{0.26, 0.19, 0.54\}','MT, w=\{0.26, 0.19, 0.54\}','BT, w=\{0.26, 0.19, 0.54\}','GC, w=\{0.26, 0.19, 0.54\}',...
        'TTD, w=\{1,0,0\}', 'MT w=\{1,0,0\}', 'BT w=\{1,0,0\}','GC, w=\{1,0,0\}')
        xlabel('Nbre of Targets (=3 x Nbre of Robots)');
    ylabel(strcat('Cost'));
    %%%%%%%%%%%%% w5= 0 1 0  %%%%%%%%%%%%%%%%%%%%%%%%%
    %return
        GA=dlmread(strcat('Greedyapproaches_comparison_3OF-w3.txt'))

    %W3--TTD, MT and GC
    ot=dlmread('TTD-MT-GC-BT-w3.txt')
    ot=[GA(:,1), GA(:,2), ot]
    Avgot=mean_mat(ot,1)
    
    %compute the error
     for i=1:5
      Erot(i,:)= 1.96*std(ot(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      Erot(i,1:2)=ot(ns*i,1:2)      
      
     end   
     
    
    %subplot(1,3,2)
    figure
    hold on
    errorbar(Avgot(:,1),Avgot(:,3),Erot(:,3),'-.','Color','black')
    errorbar(Avgot(:,1),Avgot(:,4),Erot(:,4),'--','Color','black')
    errorbar(Avgot(:,1),Avgot(:,5),Erot(:,5),'-','Color','black')
    errorbar(Avgot(:,1),Avgot(:,6),Erot(:,6),'o-','Color','black')
    
    GA=dlmread(strcat('Greedyapproaches_comparison_3OF-w5.txt'))
    %GC=
     
    ot=dlmread('TTD-MT-BT-GC-w5.txt')
    ot=[GA(:,1), GA(:,2), ot]
    Avgot=mean_mat(ot,1)
    
    %compute the error
     for i=1:5
      Erot(i,:)= 1.96*std(ot(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      Erot(i,1:2)=ot(ns*i,1:2)      
      
     end   
     
    %hold on
    %subplot(1,3,2)
    hold on
    errorbar(Avgot(:,1),Avgot(:,3),Erot(:,3),'-.','Color',clr(1,:))
    errorbar(Avgot(:,1),Avgot(:,4),Erot(:,4),'--','Color',clr(1,:))
    errorbar(Avgot(:,1),Avgot(:,5),Erot(:,5),'-','Color',clr(1,:))
    errorbar(Avgot(:,1),Avgot(:,6),Erot(:,6),'o-','Color',clr(1,:))
      legend('TTD, w=\{0.26, 0.19, 0.54\}','MT, w=\{0.26, 0.19, 0.54\}','BT, w=\{0.26, 0.19, 0.54\}','GC, w=\{0.26, 0.19, 0.54\}',...
                'TTD w=\{0,1,0\}', 'MT w=\{0,1,0\}','BT w=\{0,1,0\}', 'GC, w=\{0,1,0\}' )
     xlabel('Nbre of Targets (=3 x Nbre of Robots)');
    ylabel(strcat('Cost'));
    %%%%%%%%%% w6 %%%%%%%%%%%%%%%%%%
    
        GA=dlmread(strcat('Greedyapproaches_comparison_3OF-w3.txt'))

    %W3--TTD, MT and GC
    ot=dlmread('TTD-MT-GC-BT-w3.txt')
    ot=[GA(:,1), GA(:,2), ot]
    Avgot=mean_mat(ot,1)
    
    %compute the error
     for i=1:5
      Erot(i,:)= 1.96*std(ot(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      Erot(i,1:2)=ot(ns*i,1:2)      
      
     end   
     
    
    %subplot(1,3,3)
    figure
    hold on
    errorbar(Avgot(:,1),Avgot(:,3),Erot(:,3),'-.','Color','black')
    errorbar(Avgot(:,1),Avgot(:,4),Erot(:,4),'--','Color','black')
    errorbar(Avgot(:,1),Avgot(:,5),Erot(:,5),'-','Color','black')
    errorbar(Avgot(:,1),Avgot(:,6),Erot(:,6),'o-','Color','black')
    
    ot=dlmread('TTD-MT-BT-GC-w66.txt')
    ot=[GA(:,1), GA(:,2), ot]
    Avgot=mean_mat(ot,1)
    
    %compute the error
     for i=1:5
      Erot(i,:)= 1.96*std(ot(ns*(i-1)+1:ns*i,:))/sqrt(ns)
      Erot(i,1:2)=ot(ns*i,1:2)      
      
     end   
     
    %hold on
    %subplot(2,2,4)
    hold on
    errorbar(Avgot(:,1),Avgot(:,3),Erot(:,3),'-.','Color',clr(4,:))
    errorbar(Avgot(:,1),Avgot(:,4),Erot(:,4),'--','Color',clr(4,:))
    errorbar(Avgot(:,1),Avgot(:,5),Erot(:,5),'-','Color',clr(4,:))
    errorbar(Avgot(:,1),Avgot(:,6),Erot(:,6),'o-','Color',clr(4,:))
    
    
 
    legend('TTD, w=\{0.26, 0.19, 0.54\}','MT, w=\{0.26, 0.19, 0.54\}','BT, w=\{0.26, 0.19, 0.54\}','GC, w=\{0.26, 0.19, 0.54\}',...
             'TTD w=\{0,0,1\}', 'MT w=\{0,0,1\}', 'BT w=\{0,0,1\}','GC, w=\{0,0,1\}' )
   % legend('boxoff')
    
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



