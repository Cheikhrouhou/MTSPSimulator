
%multi-objective mstsp Simulator -- the GU interface that call other functions
%date: 14-11-2015: solver tsp activated
%update of load config
%mogreedy improv2-->6

%%%%%%%%%%%%%%%%%%%%%%%%%


function varargout = multiObjectiveTSP(hObject, eventdata, handles)

%clear workspace
clear 
clear global


global DefaultNbreTargets DefaultNbreRobots Dim;
global solver nb_run;

nb_run=1
solver='tsp';

DefaultNbreTargets=10;
DefaultNbreRobots=3;
Dim=1000;
DefaultCommRange=10*Dim;
global negociate_all negociate_pt_by_pt optimize_total_distance optimize_max_tour Targets InitRobotPos Robots;
global speed comm_range ;
speed=50;
comm_range=200;
%global sl_speed sl_comm_range label_speed_value label_comm_value;
global plot_target last_plot;
global filename;
filename='';
plot_target=[];
last_plot=[];

Targets=[];
InitRobotPos=[];
Robots=[];

negociate_all=1;
negociate_pt_by_pt=2;
%negociate_type=negociate_pt_by_pt;

optimize_total_distance=1;
optimize_max_tour=2;
%optimize_type=optimize_max_tour;
global nbrTargets nbrRobots ATRobot

if nargin==0
    nbrTargets=DefaultNbreTargets;
    
end
if nargin<2
    nbrRobots=DefaultNbreRobots    
end
if nargin<3
    CommRange=DefaultCommRange;
end


%Plot and initial Position
global pfig;
pfig = figure('Name','Multi-Robot Simulator: Multi-Objectif TSP',...
    'Numbertitle','off','menubar', 'none');
global p;
p = uipanel('Parent', pfig);
%hc =layout.GridBagLayout(p, 'HorizontalGap', 5, 'VerticalGap', 5);

%plot();
%ha=get(pfig,'CurrentAxes');
% axis([0 100 0 100])
h=axes('Xlim',[0 Dim],'Ylim',[0 Dim],'Position', [0.2 0.10 0.75 0.8],'parent',p)


%%%%%Adding control Button
%put_button();


put_option_point();
%put_option_weight();
%put_option_cost();
%put_option_bid();


  
global bt_launch mn_tc;
bt_launch=uicontrol('Style', 'pushbutton', 'string','Launch','Callback',...
            {@run}, 'Position', [5 10 70 20])
set(bt_launch, 'enable', 'off');

uicontrol('Style', 'pushbutton', 'string','DrawTarget','Callback',...
            {@draw_target, pfig},'Position', [5 50 70 20])        
%nbrTargets,nbrRobots, CommRange, Targets, InitRobotPos,negociate_type, optimize_type)

uicontrol('Style', 'pushbutton', 'string','DrawRobot','Callback',...
            {@draw_robot, pfig},'Position', [5 30 70 20])        

bt_draw=uicontrol('Style', 'pushbutton', 'string','DrawAll','Callback',...
            {@drawAll}, 'Position', [5 70 70 20]);        
%uicontrol('Style', 'pushbutton', 'string','Stop','Callback', 'close','Position', [5 60 70 20]);  


%uicontrol('Style', 'pushbutton', 'string','Clear Target','Callback', {@delete_target},'Position', [5 40 70 20]);        
%uicontrol('Style', 'pushbutton', 'string','Undo','Callback', {@undo},'Position', [5 20 70 20]);        


%Position
scnsize = get(0,'ScreenSize');
pos=get(pfig,'position');
% set(pfig,'OuterPosition',[scnsize(3)/2 30 scnsize(3)/2 scnsize(4)/2]) 
set(pfig,'OuterPosition',[scnsize(3)/2 scnsize(4)/3.5 scnsize(3)/2 scnsize(4)/1.5]) %1.4
%put_slider(pfig)
put_menu()
put_contextmenu();
%set(gca, 'ButtonDownFcn', @view_pt_coordinates);
%set(gcf, 'WindowButtonDownFcn', @view_click_coordinates);
%set(f, 'WindowButtonDownFcn', @clicker);

%solver_mrcoord();
%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%  TEMPORARY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%drawAll(gcf);
%bt_draw_callback(bt_draw,eventdata,handles)
%drawAll(bt_draw, 'push')

global menu_viewnames
%set (menu_viewnames, 'checked', 'on');
view_names(menu_viewnames, 'checked')

%run(bt_launch, 'push')


%run

end


%function mtsp_auction(Targets_, Robots_,CommRange_, cost_type_)
function run(hObject, eventdata)
global nb_run
global solver;
global mo_approach tsp_approach Targets Robots;

    switch solver
        case 'tsp'
            options=struct('tsp_approach', tsp_approach, 'ShowProg', 1)
            tsp_solver(Targets, options)
        case 'assign'
            assign_auction(hObject, eventdata,gcf)
        case 'mdmtsp'
            mtsp_auction(hObject, eventdata,gcf)
        case 'mrcoord'
            multiRobotCoord(hObject, eventdata,gcf)
        case 'momdmtsp'
            %read the approach (algorithm)
            if isempty(mo_approach)
                mo_approach='greedy'
            end
            switch mo_approach
                
                case 'greedy'
                    multiobjectif_tsp(hObject, eventdata,gcf)
                case 'ahpGrdyImprov'
                    [rte, brk, globalCost]=momdmtsp_ahpGreedyImproved6(Targets, Robots') %, compMat, show_prog
                case 'ga'
                    multiobjectif_ga(hObject, eventdata,gcf)
                    %mdmtspv_ga_mo()
                    %[min_dist,best_tour,generation] = mdmtspv_ga_mo(Targets,nbrRobots,depots,1,1,pop_size,num_iter)
                case 'gamc'
                    multiobjectif_ga(hObject, eventdata,gcf)
                case 'nsga'
                    multiobjectif_nsga(hObject, eventdata,gcf)
                case 'flmtsp'
                    flmtsp(hObject, eventdata,gcf)
                    %flmtsp
            end
    end
nb_run=nb_run+1

end
% function [TourIndex, TourDist]=tsp_solver(Targets, options)
% %reading parameters
% %global   Targets Robots;
% %global tsp_approach;
% 
% nbrTargets=size(Targets, 1);
% %nbrRobots=size(Robots,2)
% switch nbrTargets 
%     case 0
%         msgbox('Nbre of Targets must not be zero','Change configuration','Warning');
%         return;
%     case 1
%         [TourIndex, TourDist]=[1,0]
%         return;
%     case 2
%         TourIndex=[1 2]
%         TourDist=norm(Targets(1,:)-Targets(2,:))
%         return;
% end
% 
% if nargin>1
%     if isfield(options, 'tsp_approach') && ~isempty(options.tsp_approach)
%         tsp_approach = options.tsp_approach;
%     else
%         tsp_approach = 'lkh';
%     end
%     if isfield(options, 'ShowProg')
%         ShowProg = options.ShowProg;
%     else
%         ShowProg = 0;
%     end
% else
%     tsp_approach = 'lkh';
%     ShowProg = 0;
%     
% end
% 
% switch tsp_approach
%     case 'ga'
%         [popSize, numIter, ShowProg, showResult]=read_option_tsp_ga
%         [TourIndex, TourDist]=tsp_ga(Targets,popSize, numIter, ShowProg, showResult)
%         
%     otherwise 'lkh'
%         
%         %creation of a .tsp file containing problem prameter
%         %options='';
%         [TourIndex, TourDist] = tsp_lkh(Targets);
%         if ShowProg
%             tour=Targets(TourIndex,:)
%             figure('Name','TSP_LKH');
%             %plot(tour(:,1),tour(:,2),'ro')
%             plot(tour(:,1),tour(:,2),'ro-')
%             title(sprintf('Total Distance = %1.2f',TourDist));
%         end
%         
% end
% 
% end
function multiobjectif_ga(hObject, eventdata,gcf)

%reading parameters
global   Targets Robots;
global  globalCost tourcost tour time energy param;
global mo_approach;

%global debug


nbrTargets=size(Targets, 1)
nbrRobots=size(Robots,2)
if ~(nbrTargets && nbrRobots)
    msgbox('Nbre of Targets and Nbre of Robots must not be zero','Change configuration','Warning');
    return;
end

%initialization
ahp_aproach=1;
if ahp_aproach
    compMat=[1 2
            0.5 1];
    w=ahp(compMat);
    weight=[w' zeros(1,5-size(w))]
    [wd wmt wt we wv]=deal(weight(1),weight(2), weight(3), weight(4), weight(5));                

else
[wd wmt wt we wv]=read_weight()
%unassigned targets firstly = all target
weight=[wd wmt wt we wv]
end
param=[size(Targets,1) size(Robots,2) weight]
depots=Robots';
%[min_dist,best_tour,generation] = mdmtspv_ga_mo(xy,max_salesmen,depots,CostType,min_tour,pop_size,num_iter,show_prog,show_res,dmat)
switch mo_approach
    case 'ga'
        [min_dist,best_tour,generation] = mdmtspv_ga_mo(Targets,nbrRobots,depots,3, weight)
    case 'gamc'
        output = mdmtsp_ga_multi_ch_mo(Targets,nbrRobots,depots, weight)
end

end
function [rte, brk]=multiobjectif_tsp(hObject, eventdata,gcf)


%compute cost for each robot
global   Targets Robots;
global  globalCost tourcost tour time energy param;
global debug
debug=  strcmp(get(findobj(gcf,'Type','uimenu','Label','Step-by-Step'), 'Checked'),'on')
      
%debug=1
%debug=0
clear tourcost;

nbrTargets=size(Targets, 1)
nbrRobots=size(Robots,2)
if ~(nbrTargets && nbrRobots)
    msgbox('Nbre of Targets and Nbre of Robots must not be zero','Change configuration','Warning');
    return;
end

%initialization
alpha1=1;
%read weight from the GU Interface 
[wd wmt wt we wv]=read_weight()
%unassigned targets firstly = all target
weight=[wd wmt wt we wv]
param=[size(Targets,1) size(Robots,2) weight]

UTargets=Targets;
for ri=1:nbrRobots
        
   tour{ri}=[Robots(:,ri)'];
   tourcost(ri)=0;
   time(ri)=0;
   energy(ri)=0;
   speed(ri)=ri ;
   NbATargets(ri)=1;
   changeFlag(ri)=1;
end

global nb_run Dim 
%plot final tour
if(nb_run>1)
    create_newfigure();
end
%build tsp option
options=struct('tsp_approach', 'lkh' );

while(UTargets)
%each robots compute Costlocal=?1distance+ ?2Enegy + //?3Time  
%and send a bid for the selected targets
for ri=1:nbrRobots
    %distance
    %distance=cost_dist(Robots(:,ri)', Targets)
    %compute new tour costs when getting this target
    %if tour of ri changed
    if (changeFlag(ri))
        costTable{ri}=[];
        candtour{ri}=[];
        changeFlag(ri)=0;
        for ti=1:size(UTargets,1)
            %candidate tour      
            candtour{ri}{ti}=vertcat(tour{ri}, UTargets(ti,:));
            [ind{ri}{ti},candtourcost(ti)]=tsp_solver(candtour{ri}{ti},options);

            costTable{ri}=vertcat(costTable{ri}, [ti, candtourcost(ti)]);
        end
        
       %choose the best targets

        [newtourcost(ri), ti]=min(costTable{ri}(:,2))
        
        %energy
        newEnergy(ri)=newtourcost(ri)/2
        %Time
        newTime(ri)=speed(ri)*newtourcost(ri)
        %localcost
        %costTable{ri}(:,2)=alpha1*costTable{ri}(:,2)
        %number of asigned targets
        newNbATargets(ri)=size(tour{ri},1)+1;

        %make attention between target and index
        bidding(ri,:)=[ti, newtourcost(ri), newEnergy(ri), newTime(ri), newNbATargets(ri)]        
    else %remove the allocated targets
        
        candtour{ri}(lastAllocatedTarget)=[];
        costTable{ri}(lastAllocatedTarget,:)=[];
        ind{ri}(lastAllocatedTarget)=[];
        bidTarget(ri)=bidding(ri,1:1)
        
        if bidTarget(ri)==lastAllocatedTarget
            %search for other target
            %choose another  best targets

                [newtourcost(ri), ti]=min(costTable{ri}(:,2))

                %energy
                newEnergy(ri)=newtourcost(ri)/2
                %Time
                newTime(ri)=speed(ri)*newtourcost(ri)
                %localcost
                %costTable{ri}(:,2)=alpha1*costTable{ri}(:,2)
                %number of asigned targets
                newNbATargets(ri)=size(tour{ri},1)+1;

                %make attention between target and index
                bidding(ri,:)=[ti, newtourcost(ri), newEnergy(ri), newTime(ri), newNbATargets(ri)]   
  
        elseif(bidTarget(ri)>lastAllocatedTarget)
            %only ti index changed
            bidTarget(ri)=bidTarget(ri)-1;
            bidding(ri,:)=[bidTarget(ri), newtourcost(ri), newEnergy(ri), newTime(ri), newNbATargets(ri)]  
        end
    end
    
    
 
 
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% TO BE DONE AT SERVER SIDE %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%based on bidding the Server select the one target to assign to a robot
%he selects the target with low cost
% [cost, ri]=min(bidding(:,2))
% ti=bidding(ri, 1)
%compute global cost correspending to each local cost
clr=hsv(nbrRobots);
for ri=1:nbrRobots        
   
   %compute the new max tour
   newmaxtour(ri)=max([tourcost,newtourcost(ri)])
   
   %compute the new time
   newmaxTime(ri)=max([time,newTime(ri)])
   %new energy
   
   %compute the new variance
   x=NbATargets
   x(ri)=newNbATargets(ri)
   newVariance(ri)=var(x)
   
   %global cost
   %Costglobal=?1distanceGlobal+ ?2EnegyGlobal + ?3MaxTime+ ?4MaxTour+?5Variance.
   newGlobalCost(ri)=wd*(sum(tourcost)-tourcost(ri)+newtourcost(ri))+ wmt*newmaxtour(ri)...
       +we*(sum(energy)-energy(ri)+newEnergy(ri))+wt*newmaxTime(ri)+wv*newVariance(ri)
   
end

%choose the best assignment (1 target)
[globalCost,winner]=min(newGlobalCost)

%remove the targets from list of available targets
%remove(UTargets(ti,:), UTargets)
%assignet
bidTarget(winner)=bidding(winner,1:1)
lastAllocatedTarget=bidTarget(winner)
UTargets(bidTarget(winner),:)=[];
%vertcat(tour{ri
    %the possible new tour of ri
    tourOrder=circshift(ind{winner}{bidTarget(winner)}, [0 -(find(ind{winner}{bidTarget(winner)}==1)-1)])
    %newtour{ri}=candtour{ri}{ti}(ind{ti},:)
    tour{winner}=candtour{winner}{bidTarget(winner)}(tourOrder,:) %newtour{winner}

changeFlag(winner)=1;
tourcost(winner)=newtourcost(winner)
NbATargets(winner)=newNbATargets(winner)

%plot the 

try
    delete(tr(winner))
    tr(winner)=plot(tour{winner}(:,1),tour{winner}(:,2), 'Color',clr(winner,:))
catch
    %winner
    tr(winner)=plot(tour{winner}(:,1),tour{winner}(:,2), 'Color',clr(winner,:))    
end


    if(debug)
        %uiwait
        w = waitforbuttonpress;
    else 
        pause(0.1)
    end
end

celldisp(tour)



for rj=1:nbrRobots
    %plot(BestTour{rj}(:,1),BestTour{rj}(:,2), 'Color',clr(rj,:))
    
    plot(tour{rj}(:,1),tour{rj}(:,2), 'Color',clr(rj,:))
    [logic, loc]=ismember(tour{rj},Targets)
    rte{rj}=loc(2:end,1)'
    if(rj==1)
        brk(rj)=size(rte{rj},2)
    elseif rj<nbrRobots
        brk(rj)=size(rte{rj},2)+brk(rj-1)
    end
end

%print statistics data

title(sprintf('TTD = %1.3f \t MaxTour = %1.3f \t \n GlobalCost:%1.3f ',sum(tourcost),max(tourcost), globalCost));
%set(pfig,'Name', sprintf('Final Solution: %s :Total Traveled Distance = %1.4f',cost_type,Cost_hung))
%gl_cost=wd*sum(tourcost)+wmt*maxtour+we*sum(energy)+wt*maxTime+wv*Variance
%gl_cost=wd*sum(tourcost)+wmt*newmaxtour(ri)+we*sum(energy)+wt*newmaxTime(ri)+wv*newVariance(ri)

%
%%% we construct a population rote to input to GA
%pop_rte=
global menu_viewtours
set (menu_viewtours, 'Enable', 'on');
end
function create_newfigure()
global Targets Robots Dim
global menu_viewnames param
fig_name=sprintf('Multi-Robot Simulator, Result-%iT-%iR-%iwd-%iwmt-%iwt-%iwe-%iwv',param);%-%s, cputime
%if(nb_run>1)
    newfig = figure('Name',fig_name,'Numbertitle','off','menubar', 'none');
    p = uipanel('Parent', newfig);    
    h=axes('Xlim',[0 Dim],'Ylim',[0 Dim],'Position', [0.2 0.10 0.75 0.8],'parent',p)
    %drawAll(bt_draw, 'push')
    plot (Targets(:,1),Targets(:,2),'ro')
    hold on
    plot (Robots(1,:),Robots(2,:), 'd')
    
    put_menu()
    %set (menu_viewnames, 'checked', 'on');
    %view_names(hObject, eventdata, handles)
    view_names(menu_viewnames, 'checked')
    
%end
end
function assign_auction(hObject, eventdata,pfig)

%declaration of status
Free=0;
Searching=1;
Moving=2;
Allocated=3;

global   cost_type Targets Robots;
global clr Dim;
global nbrTargets  


read_option_cost();
% CommRange=get(comm_range,'value')
% DELAY=get(speed,'value')

%read targets and robots positions

nbrTargets=size(Targets, 1)
nbrRobots=size(Robots,2)
if ~(nbrTargets && nbrRobots)
    msgbox('Nbre of Targets and Nbre of Robots must not be zero','Change configuration','Warning');
    return;
end
%new figure
%Plot Targets and initial Position
pfig = figure('Name','MTSPV_Auction Discovering| Current Best Solution','Numbertitle','off');
%Position
scnsize = get(0,'ScreenSize');
pos=get(pfig,'position');
% set(pfig,'OuterPosition',[scnsize(3)/2 10 scnsize(3)/2 scnsize(4)/2]) 

plot (Targets(:,1),Targets(:,2),'ro') 
hold on
plot (Robots(1,:),Robots(2,:), 'd') 
axis([0 Dim 0 Dim])
%Initially

clr=hsv(nbrRobots);
CurrentPositions=Robots;
for ri=1:nbrRobots
 
    ATRobot{ri}=[];
    status(ri)=Free;
    TraveledDist(ri)=0;
    
    hTxt(ri) = text(CurrentPositions(1,ri), CurrentPositions(2,ri), ... 
        sprintf('R%i:(%.1f,%.1f)', ri,CurrentPositions(1,ri), CurrentPositions(2,ri)), ...
    'Color',clr(ri,:), 'FontSize',8, ...
    'HorizontalAlignment','left', 'VerticalAlignment','top');
    
end

%%%%%%%%%%%%%%%%%%%%%Hungarian start%%%%%%%%%%%%%%%%%%%%%
 if strcmp(cost_type,'cost_hung' )
     
    Perf= distmat(Robots', Targets)
    [Matching,Cost_hung]=Hungarian(Perf) 
    
    %%%plotting result
    for ri=1:size(Matching,2)
        ti=find(Matching(:,ri))
        plot([Robots(1,ri),Targets(ti,1)],[Robots(2,ri),Targets(ti,2)], 'Color',clr(ri,:))
        
        title(sprintf('Traveled Distance = %1.3f \t',Cost_hung));
        set(pfig,'Name', sprintf('Final Solution: %s :Total Traveled Distance = %1.4f',cost_type,Cost_hung))
    end
    %write_statistics(size(Matching,2), cost_type,Cost_hung)
    return
 end

 
 if strcmp(cost_type,'cost_munk' )
    Perf= distmat(Robots', Targets)
    %Perf=round(Perf)
    [Matching,Cost_munk]=munkres(Perf) 
    
    %%%plotting result
    for ri=1:size(Matching,2)
        ti=Matching(ri)
        plot([Robots(1,ri),Targets(ti,1)],[Robots(2,ri),Targets(ti,2)], 'Color',clr(ri,:))
        
        title(sprintf('Traveled Distance = %1.3f \t',Cost_munk));
        set(pfig,'Name', sprintf('Final Solution: %s :Total Traveled Distance = %1.4f',cost_type,Cost_munk))
    end
    %write_statistics(size(Matching,2), cost_type,Cost_munk)
    return
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%Hungarian end%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%DMB start%%%%%%%%%%%%%%%%%%%%%

 if strcmp(cost_type,'cost_DMB') 
     
     Matching = DMB (Robots, Targets);
     %disp(length(Matching));
     %disp(size(Matching));
     %disp(size(Matching,2));
    % disp(length(Robots));
    for ri=1:length(Matching)
        disp(Matching(1,ri));
        disp(Matching(2,ri));
        disp('--------------------------');
    end
    %%%plotting result
    cost =0;
    for ri=1:size(Matching,2)%10columns
       xR = Robots(1, Matching(1,ri));
       yR = Robots(2, Matching(1,ri));
       xT = Targets(Matching(2,ri), 1);
       yT = Targets(Matching(2,ri), 2);
       plot([xR xT],[yR yT], 'Color',clr(ri,:))
      % end
      cost = cost + cost_DMB(xR,yR,xT,yT);
       % title(sprintf('Traveled Distance = %1.3f \t',cost_DMB(xR,yR,xT,yT)));
        %set(pfig,'Name', sprintf('Final Solution: %s :Total Traveled Distance = %1.4f',cost_type,cost_DMB))
    end
    title(sprintf('Traveled Distance = %1.3f \t',cost));
    set(pfig,'Name', sprintf('Final Solution: %s :Total Traveled Distance = %1.4f',cost_type,cost))
    %write_statistics(size(Matching,2), cost_type,Cost_hung)
    return
 end
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%DMB end%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%RTMA start%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tk=0;
%cost='cost_rtma';
    %initially all Targets are Unallocated
    UTargets=Targets;
    SearchingTargets=Targets;
    
while find(status==Free)
    

    %%%%%%%%%%%%Pre-selection Phase%%%%%%%%%%%%%%%%%%
    %assign targets to all robots
    for ri=1:nbrRobots
        SearchingTargets=intersect(SearchingTargets,UTargets,'rows')
        if status(ri)==Free && ~isempty(SearchingTargets)
            if strcmp(cost_type,'cost_dist')
                Cost{ri}=cost_dist(reshape(CurrentPositions(:,ri),1,2),SearchingTargets);
            elseif strcmp(cost_type,'cost_rtma')
                Cost{ri}=cost_rtma(reshape(CurrentPositions(:,ri),1,2),SearchingTargets);
            end
                
            %choosing the best targets
            [MinT(ri),IT(ri)]=min(Cost{ri})

            ATRobot{ri}=vertcat(ATRobot{ri},UTargets(IT,:))
            SelectedTarget(:,ri)=SearchingTargets(IT(ri),:)
            status(ri)=Searching;
        end
    end 
    
    %%%%%Communication Between Robots%%%%%%%%%%%
    %Find  Neigbors robots Dist<CommRange
    
    for ri=1:nbrRobots
        if status(ri)==Searching 
                 
                 %negociate on selected targets
                 %n=1;
                for n=1:nbrRobots 
                  
                    %here we do bidding on task 
                    %even if they do not have same selected target
                    if SelectedTarget(:,ri)==SelectedTarget(:,n) 
                        %compare cost    
                        if MinT(ri)>MinT(n) && n~=ri
                              %the robots must search for new target
                              %remove targets from the neighbor list of
                              %SearchingTargets
                              neigh=n
                              bid_target=SelectedTarget(:,ri)
                              %[mb,loc]=ismember(reshape(SelectedTarget(:,ri),1,2),SearchingTargets, 'rows')
                              %if  loc>0 && loc<=size(SearchingTargets,1) ,SearchingTargets(loc,:)=[], end
                              status(ri)=Free
                              break
                        end

                    
                    end
                end
                    
                    %n=n+1;
                    %There is non conflict on the selected targets
                    if status(ri)==Searching, 
                        status(ri)=Allocated, 
                        SearchingTargets=remove(SelectedTarget(:,ri)', SearchingTargets);
                        plot([Robots(1,ri),SelectedTarget(1,ri)],[Robots(2,ri),SelectedTarget(2,ri)], 'Color',clr(ri,:))
                        TraveledDist(ri)=pdist([CurrentPositions(:,ri)' ;SelectedTarget(:,ri)'])
                        text('Position',SelectedTarget(:,ri), ...
                             'String',sprintf('R%i:(%.1f)',ri,TraveledDist(ri)),...
                             'Color',clr(ri,:), 'FontSize',8, 'HorizontalAlignment',...
                             'left', 'VerticalAlignment','top')
                        
                    end
                    %remove the targets from neighbor UTargets

                   
     end 
    
    end
end




sprintf('Allocated Targets to Robots ')
celldisp(ATRobot)
sprintf('Travelled Distance by each Robot')
TraveledDist
TotalTravDist=sum(TraveledDist)


%Plotting purpose     
title(sprintf('Traveled Distance = %1.3f \t',TraveledDist));
set(pfig,'Name', sprintf('Final Solution: %s :Total Traveled Distance = %1.4f',cost_type, TotalTravDist))

%write_statistics(nbrTargets, cost_type, TotalTravDist)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

function multiRobotCoord(hObject, eventdata,pfig)

%declaration of status
Free=0;
Searching=1;
Moving=2;
Allocated=3;

global   cost_type Targets Robots;
global clr Dim;
global nbrTargets  


read_option_cost();
% CommRange=get(comm_range,'value')
% DELAY=get(speed,'value')

%read targets and robots positions

nbrTargets=size(Targets, 1)
nbrRobots=size(Robots,2)
if ~(nbrTargets && nbrRobots)
    msgbox('Nbre of Targets and Nbre of Robots must not be zero','Change configuration','Warning');
    return;
end
%new figure
%Plot Targets and initial Position
pfig = figure('Name','MTSPV_Auction Discovering| Current Best Solution','Numbertitle','off');
put_menu();
%Position
scnsize = get(0,'ScreenSize');
pos=get(pfig,'position');
%set(pfig,'OuterPosition',[scnsize(3)/2 10 scnsize(3)/2 scnsize(4)/2]) 

plot (Targets(:,1),Targets(:,2),'ro') 
hold on
plot (Robots(1,:),Robots(2,:), 'd') 
axis([0 Dim 0 Dim])
%Initially

clr=hsv(nbrRobots);
CurrentPositions=Robots;
for ri=1:nbrRobots
 
    ATRobot{ri}=[];
    %SearchingTargets{ri}=Targets;
    
    status(ri)=Free;
    TraveledDist(ri)=0;
    
    hTxt(ri) = text(CurrentPositions(1,ri), CurrentPositions(2,ri), ... 
        sprintf('R%i:(%.1f,%.1f)', ri,CurrentPositions(1,ri), CurrentPositions(2,ri)), ...
    'Color',clr(ri,:), 'FontSize',8, ...
    'HorizontalAlignment','left', 'VerticalAlignment','top');
    
end

%%%%%%%%%%%%%%%%%%%%MultiRobot Coordintion start%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Multi-Robot Coordination Solver');
disp('Number of Robots');
nbrRobots
disp('Number of Target');
nbrTargets

tk=0;
%cost='cost_rtma';
    %initially all Targets are Unallocated
    disp('List of Unallocated Targets');
    UTargets=Targets
    %cost_type='cost_rtmam';
while UTargets %find(status==Free)
    
    nbrTargets=size(UTargets, 1);
    for ti=1:nbrTargets
        BidTarget{ti}=[];
    end
    %%%%%%%%%%%%    Phase 2. Auctioning and Bidding     %%%%%%%%%%%%%%%%%
    %Each robot selects a target and send Bid
    for ri=1:nbrRobots
        %SearchingTargets{ri}=intersect(SearchingTargets{ri},UTargets,'rows')
        if status(ri)==Free %&& ~isempty(SearchingTargets{ri})
            if strcmp(cost_type,'cost_rtma')
                Cost{ri}=cost_rtma(reshape(CurrentPositions(:,ri),1,2),UTargets);
            elseif strcmp(cost_type,'cost_rtmam')
                Cost{ri}=cost_rtmam(reshape(CurrentPositions(:,ri),1,2),UTargets);  %Unallocted Targets
                
            else %strcmp(cost_type,'cost_dist') %the default
                Cost{ri}=distmat(reshape(CurrentPositions(:,ri),1,2),UTargets);                
            
            end
                
            %choosing the best targets
            [MinT(ri),IT(ri)]=min(Cost{ri})

            %ATRobot{ri}=vertcat(ATRobot{ri},UTargets(IT,:))
            SelectedTarget(:,ri)=UTargets(IT(ri),:);
            status(ri)=Searching;
            
            %RobotSelectingTarget{loc}=horizcat(RobotSelectingTarget{loc} ,ri)
            [mb,loc]=ismember(reshape(SelectedTarget(:,ri),1,2),UTargets, 'rows')
            BidTarget{loc}=vertcat(BidTarget{loc},[MinT(ri) ri])
        end
    end
    
    %%%%%%%%        Phases 3. Assignment Phase      %%%%%%%%%%%%
    %The Control Station select the best Bid for each target
    for ti=1:nbrTargets
        if size(BidTarget{ti},1)>=2
            [MinR(ti),IR]=min(BidTarget{ti}(:,1))
            IR=BidTarget{ti}(IR,2)
            status(IR)=Allocated;
            %remove targets from available targets
            %UTargets=remove(UTargets(ti,:),UTargets);
            
        elseif size(BidTarget{ti},1)
            
            IR=BidTarget{ti}(1,2)
            status(IR)=Allocated;
            %remove targets from available targets
            %UTargets=remove(UTargets(ti,:),UTargets);
            
        end
        
    end
    
               %Plotting Allocated Targets
               for ri=1:nbrRobots
                    %There is non conflict on the selected targets
                    if status(ri)== Allocated
                        %SearchingTargets=remove(SelectedTarget(:,ri)', SearchingTargets);
                        plot([CurrentPositions(1,ri),SelectedTarget(1,ri)],[CurrentPositions(2,ri),SelectedTarget(2,ri)], 'Color',clr(ri,:))
                        TraveledDist(ri)=TraveledDist(ri)+pdist([CurrentPositions(:,ri)' ;SelectedTarget(:,ri)'])
                        text('Position',SelectedTarget(:,ri), ...
                             'String',sprintf('R%i:(%.1f)',ri,TraveledDist(ri)),...
                             'Color',clr(ri,:), 'FontSize',8, 'HorizontalAlignment',...
                             'left', 'VerticalAlignment','top')
                         
                         UTargets=remove(SelectedTarget(:,ri)',UTargets);
                         CurrentPositions(:,ri)=SelectedTarget(:,ri);
                        
                    end
                    
                    status(ri)=Free;
               end


                   
end 
    
    
sprintf('Allocated Targets to Robots ')
celldisp(ATRobot)
sprintf('Travelled Distance by each Robot')
TraveledDist
TotalTravDist=sum(TraveledDist)


%Plotting purpose     
title(sprintf('Total Traveled Distance = %1.3f MaxPath= %.3f\t',sum(TraveledDist), max(TraveledDist)));
set(pfig,'Name', sprintf('Final Solution: %s :Total Traveled Distance = %1.4f',cost_type, TotalTravDist))

%write_statistics(nbrTargets, cost_type, TotalTravDist)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

function mtsp_auction(hObject, eventdata,pfig)

%declaration of status
Free=0;
Searching=1;
Moving=2;

global speed comm_range cost_type bid_type;
global Targets Robots CommRange MoveUnit ATRobot;
global clr Dim filename;

read_parameter();
%write targets and robots positions

CommRange=comm_range;
MoveUnit=speed;
DELAY=1; %%for annimation purpose
TIME_LIMIT=1000;
%read parameter
%read_option_cost();
%CommRange=get(comm_range,'value')
%DELAY=get(speed,'value')

%read targets and robots positions
%InitRobotPos=Robots;
nbrTargets=size(Targets, 1)
nbrRobots=size(Robots,2)
if ~(nbrTargets && nbrRobots)
    msgbox('Nbre of Targets and Nbre of Robots must not be zero','Change configuration','Warning');
    return;
end
%new figure
%Plot Targets and initial Position
%pfig = figure('Name','MTSPV_Auction Discovering| Current Best Solution','Numbertitle','off');
pfig = figure('Name',strcat('MTSPV_Auction Discovering| Bidding Type:',bid_type),...
    'Numbertitle','off','menubar', 'none');
%Position
scnsize = get(0,'ScreenSize');
pos=get(pfig,'position');
% parent=get(pfig,'parent');
% set(pfig,'OuterPosition',[scnsize(3)/2 10 scnsize(3)/2 scnsize(4)/2]) 
% set(pfig,'OuterPosition', get(parent,'OuterPosition'));

%h=axes('Xlim',[0 Dim],'Ylim',[0 Dim],'Position', [0.08 0.10 0.78 0.800])
if nbrTargets
    plot (Targets(:,1),Targets(:,2),'ro', 'Tag', 'Targets') 
end
hold on
if nbrRobots
    plot (Robots(1,:),Robots(2,:), 'd','Tag', 'Robots' ) 
end
%axis([0 Dim 0 Dim])

%put_slider(pfig)
%Initially

clr=hsv(nbrRobots)
CurrentPositions=Robots;
for ri=1:nbrRobots
    %initially all Targets are Unallocated
    UTargets{ri}=Targets;
    SearchingTargets{ri}=Targets;
    
    ATRobot{ri}=[];
    status(ri)=Free;
    TraveledDist(ri)=0;
    
    hTxt(ri) = text(CurrentPositions(1,ri), CurrentPositions(2,ri), ... 
        sprintf('R%i:(%.1f,%.1f)', ri,CurrentPositions(1,ri), CurrentPositions(2,ri)), ...
    'Color',clr(ri,:), 'FontSize',8, ...
    'HorizontalAlignment','left', 'VerticalAlignment','top');
    
end


tk=0;
%cost='cost_rtma';
% Allocation Phase %%
%%%%%%%%%%%%%%%%%%%%%%%%%

 

while tk<TIME_LIMIT && (sum(~cellfun(@isempty,SearchingTargets))~=0 || sum(status)~=Free)
    
    %%%%Increment counter
    tk=tk+1;
%     DELAY=get(speed,'value')
%     CommRange=get(comm_range,'value')
%     pause(1/DELAY);
    sprintf('tk %i', tk)
    %%%%%%%%%%%%Pre-selection Phase%%%%%%%%%%%%%%%%%%
    %assign targets to all robots
    for ri=1:nbrRobots
        SearchingTargets{ri}=intersect(SearchingTargets{ri},UTargets{ri},'rows')
        if status(ri)==Free && ~isempty(SearchingTargets{ri})
            
            if strcmp(cost_type,'cost_rtma')
                Cost{ri}=cost_rtma(reshape(CurrentPositions(:,ri),1,2),SearchingTargets{ri});
            elseif strcmp(cost_type,'cost_rtmam')
                Cost{ri}=cost_rtmam(reshape(CurrentPositions(:,ri),1,2),SearchingTargets{ri}); 
                
            else %strcmp(cost_type,'cost_dist') %the default
                Cost{ri}=distmat(reshape(CurrentPositions(:,ri),1,2),SearchingTargets{ri});                
            
            end
                
            %choosing the best targets
            [MinT(ri),IT(ri)]=min(Cost{ri})

            %ATRobot{ri}=vertcat(ATRobot{ri},UTargets(IT,:))
            SelectedTarget(:,ri)=SearchingTargets{ri}(IT(ri),:)
            status(ri)=Searching;
        end
    end 
    
    %%%%%Communication Between Robots%%%%%%%%%%%
    %Find  Neigbors robots Dist<CommRange
    
    for ri=1:nbrRobots
        if status(ri)==Searching || status(ri)==Moving
            INR = nearestneighbour(CurrentPositions(:,ri), CurrentPositions, 'Radius', CommRange);
            INR=INR(find(INR~=ri))
            %INR=nonzeros(IN(:,ri))
            %if size(INR,2)>=1 %There are neighbors
                %compare selected targets of each neighbor robots
                for n=1:size(INR,2)
                    %echange the list of available targets with each
                    %neighbor
                    UTargets{ri}=intersect(UTargets{ri},UTargets{INR(n)}, 'rows')
                    UTargets{INR(n)}=intersect(UTargets{ri},UTargets{INR(n)}, 'rows')
                end
                %verify that the selected targets is still availabe
                 [mb,loc]=ismember(reshape(SelectedTarget(:,ri),1,2),UTargets{ri}, 'rows')
                 if  loc==0 %the target is no longer available
                      status(ri)=Free 
                      %break;     
                 end
                 
                 %negociate on selected targets
                 n=1;
                while status(ri)~=Free && n<=size(INR,2)
                  
                    %here we do bidding on task 
                    %even if they do not have same selected target
                    %if SelectedTarget(:,ri)==SelectedTarget(:,INR(n))
                    %compute the cost to do this target
                    %cost(currentPosition, selectedTargets)
                    neigh=INR(n)
                    
                    if strcmp(cost_type, 'cost_rtma')
                        %Cost{INR(n)}=cost_rtma(reshape(CurrentPositions(:,INR(n)),1,2),UTargets{ri});
                        %cost_targ=cost_mat(IT(ri))
                        %recompute the cost matrix
                        %Cost{INR(n)}=cost_rtma(reshape(CurrentPositions(:,INR(n)),1,2),UTargets{INR(n)});
                        Cost{ri}=cost_rtma(reshape(CurrentPositions(:,ri),1,2),Targets);
                        Cost{INR(n)}=cost_rtma(reshape(CurrentPositions(:,INR(n)),1,2),Targets);
                        
                        [mb,loc]=ismember(reshape(SelectedTarget(:,ri),1,2),Targets, 'rows')
                        %if  loc>0 && loc<=size(UTargets{INR(n)},1) 
                            cost_ri=Cost{ri}(loc), 
                            cost_neigh=Cost{INR(n)}(loc),
                    elseif strcmp(cost_type, 'cost_rtmam')
                        Cost{ri}=cost_rtmam(reshape(CurrentPositions(:,ri),1,2),Targets);
                        Cost{INR(n)}=cost_rtmam(reshape(CurrentPositions(:,INR(n)),1,2),Targets);
                        
                        [mb,loc]=ismember(reshape(SelectedTarget(:,ri),1,2),Targets, 'rows')
                        %if  loc>0 && loc<=size(UTargets{INR(n)},1) 
                            cost_ri=Cost{ri}(loc), 
                            cost_neigh=Cost{INR(n)}(loc),
                    else strcmp(cost_type,'cost_dist')
                        cost_ri=cost_dist(reshape(CurrentPositions(:,ri),1,2),reshape(SelectedTarget(:,ri),1,2));
                        cost_neigh=cost_dist(reshape(CurrentPositions(:,INR(n)),1,2),reshape(SelectedTarget(:,ri),1,2));                            
                         

                    end
                    
                    %compare cost    
                    if cost_ri>cost_neigh || (cost_ri==cost_neigh && INR(n) <ri) 
                        % in case of same cost, the robot with small ID receive the targets
                          %the robots must search for new target
                          %remove targets from the neighbor list of
                          %SearchingTargets
                          %neigh=INR(n)
                          %bid_target=SelectedTarget(:,ri)
                          %[mb,loc]=ismember(reshape(SelectedTarget(:,ri),1,2),SearchingTargets{ri}, 'rows')
                          %if  loc>0 && loc<=size(SearchingTargets{ri},1) ,SearchingTargets{ri}(loc,:)=[], end
                          status(ri)=Free
                          if strcmp(bid_type,'RmSelect')
                              SearchingTargets{ri}=remove(SelectedTarget(:,ri)',SearchingTargets{ri});
                              %add to neighbor SearchingTargets
                              SearchingTargets{neigh}=add(SelectedTarget(:,ri)',SearchingTargets{neigh});
                          end
                    elseif strcmp(bid_type,'RmSelect')
                        SearchingTargets{neigh}=remove(SelectedTarget(:,ri)',SearchingTargets{neigh});
                    end
                    
                    n=n+1;
                end               
            
            %There is non conflict on the selected targets
            if status(ri)==Searching, status(ri)=Moving, end
            %remove the targets from neighbor UTargets
            
        end   
     end 
    
    
    
    
    %%%%%%%%%%Moving%%%%%%%%%%%%%%%%%
    for ri=1:nbrRobots
        if status(ri)==Moving
            %move ri from current positions to the selected targets
            %CurrentPositions(:,ri)
            if norm(SelectedTarget(:,ri)-CurrentPositions(:,ri))>MoveUnit
                theta=atan2((SelectedTarget(2,ri)-CurrentPositions(2,ri)),(SelectedTarget(1,ri)-CurrentPositions(1,ri)));
            
%             CurrentPositions(1,ri)=CurrentPositions(1,ri)+MoveUnit*cosd(theta)
%             CurrentPositions(2,ri)=CurrentPositions(2,ri)+MoveUnit*sind(theta)
            
            
               NewX=CurrentPositions(1,ri)+MoveUnit*cos(theta);
               NewY=CurrentPositions(2,ri)+MoveUnit*sin(theta);
               

               
            else %it reaches (aproxiamately) the target
                %CurrentPositions(:,ri)=SelectedTarget(:,ri)
                NewX=SelectedTarget(1,ri)
                NewY=SelectedTarget(2,ri)
                status(ri)=Free

                %remove targets from my list of available targets
                [mb,loc]=ismember(reshape(SelectedTarget(:,ri),1,2),UTargets{ri}, 'rows')
                if  loc>0 && loc<=size(UTargets{ri},1) , UTargets{ri}(loc,:)=[], end
                
                %add target to the list of allocated targets
                ATRobot{ri}=vertcat(ATRobot{ri},reshape(SelectedTarget(:,ri),1,2))
                
               
            end
               figure(pfig); 
               plot ([CurrentPositions(1,ri),NewX], [CurrentPositions(2,ri),NewY],'-','Color',clr(ri,:))
               
               TraveledDist(ri)=TraveledDist(ri)+pdist([CurrentPositions(1,ri) CurrentPositions(2,ri);NewX NewY])
               CurrentPositions(1,ri)=NewX;
               CurrentPositions(2,ri)=NewY;
               %set(htext(NewX,NewY,sprintf('(%.3f,%.3f)',[NewX NewY]))
               %Plotting coordinate
%                set(hTxt(ri), 'Position',[NewX NewY], ...
%                    'String',sprintf('(%.1f,%.1f)',[NewX NewY]))
               
               %plotting traveled distance (hTxt)
               set(hTxt(ri), 'Position',[NewX NewY], ...
                   'String',sprintf('R%i:(%.1f)',ri,TraveledDist(ri)))
               drawnow
            
            
        end
        
    end

end


%print exiting code
%sprintf('tk=%i ', tk)
% sprintf('sum(~cellfun(@isempty,UTargets))~=0 is %i ',sum(~cellfun(@isempty,UTargets))~=0)
% sprintf(' sum(status)~=Free is %i', sum(status)~=Free)
%print statistics
sprintf('Initial Robots Position')
Robots

sprintf('Allocated Targets to Robots ')
celldisp(ATRobot)
sprintf('Travelled Distance by each Robot')
TraveledDist
TotalTravDist=sum(TraveledDist)

%%%%saving allocation results %%%%
%file_name=strcat('config-',size(Targets,1),'T-', size(Robots,2),'R-', comm_range,'CR-', speed,'S.mrs');
% param=[size(Targets,1) size(Robots,2) comm_range speed] %we can add other param
% filename=sprintf('Result-%iT-%iR-%iCR-%iS-%s-%s.mrs',param,date);%, cputime
% %[filename,PathName,FilterIndex] = uiputfile(file_name)
% dlmwrite(filename,param);%,'-append'
save_result(filename, ATRobot, TraveledDist, 'Allocation')
%for ri 
    
%end
%Plotting purpose     
title(sprintf('TotalDistance = %1.3f \t MaxTour = %1.3f \t AvgTour = %1.3f',...
    sum(TraveledDist),max(TraveledDist), mean(TraveledDist)));
set(pfig,'Name', sprintf('After Allocation Phase with %s Bidding:TTD = %1.4f',bid_type, TotalTravDist))
 

tour_construction()
 
end
function tour_construction(hObject, eventdata, handles)
%%%%%%%%%%%%%%%%%%%%%%%%%%After each robots determine its allocated targets
%%%%%%%%%%%%%%%%%%%%%%%%%%Each Robot launch TSP on its local allocated targets
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%if figure closed return
if isempty(findobj('type','figure')), return, end
%uicontrol('Style', 'pushbutton', 'string','Continue','Callback', 'uiresume(gcbf)','Position', [10 120 60 20]);
global depots Robots Targets ATRobot nbrRobots nbrTargets filename comm_range speed clr 

depots=permute(Robots, [2 1])
%global tfig;
global txt partfig;
for ri=1:nbrRobots
    %if ~isempty(ATRobot{ri})
        ATRobot{ri}=vertcat(depots(ri,:),ATRobot{ri})
        [BestTourIndex{ri}, TourDist(ri)]=tsp(ATRobot{ri},10,100,1,0);
%         tsp_ga(xy,dmat,popSize,numIter,showProg,showResult)
        %copyobj(findobj(tfig(ri)),findobj(tfig));
        
    %end
end

for ri=1:nbrRobots
    sprintf('Best TSP Tour to Robot %i:', ri)
    BestTour{ri}=ATRobot{ri}(BestTourIndex{ri},:)
end
%%% save result of tour construction

    
save_result(filename, BestTour, TourDist, 'Best Tour after Tour Construction')

tfig = figure('Numbertitle','off');
scnsize = get(0,'ScreenSize');
% set(tfig,'OuterPosition',[scnsize(3)/2 scnsize(4)/2+10 scnsize(3)/2
% scnsize(4)/2]) 
%set(tfig,'OuterPosition',[scnsize(3)/2 10 scnsize(3)/2 scnsize(4)/2]) 
%axis([0 Dim 0 Dim])
% plot tsp tour for each robot
for ri=1:nbrRobots

    %plotting the tour
    plot(BestTour{ri}(:,1),BestTour{ri}(:,2), 'Color',clr(ri,:))
    hold on
    plot(BestTour{ri}([1,end],1),BestTour{ri}([1,end],2), 'Color',clr(ri,:))
    %plot Tour Distance
    txt(ri)=text(depots(ri,1),depots(ri,2),sprintf(' (%.3f)',TourDist(ri)), 'Color',clr(ri,:),...
        'HorizontalAlignment','left', 'VerticalAlignment','top')
end


hold on
plot (Targets(:,1),Targets(:,2),'ro') %
hold on
plot (Robots(1,:),Robots(2,:), 'd') %, 'rs'
%axis([0 Dim 0 Dim])
sprintf('After Calling TSP on Allocated Targets ')
TourDist
TotalTourDist=sum(TourDist)
%title(sprintf('Tour Distance = %1.3f \t',TourDist));
title(sprintf('TotalDistance = %1.3f \t MaxTour = %1.3f \t AvgTour = %1.3f',...
    sum(TourDist),max(TourDist), mean(TourDist)));
set(tfig,'Name', sprintf('After TSP ::Total Tour Distance = %1.4f',TotalTourDist))



%order Best Tour to begin from Depots (which has index 1)
for ri=1:nbrRobots
    %sprintf('Best TSP Tour to Robot %i:', ri)
    nb_shift=size(BestTourIndex{ri},2)-find(BestTourIndex{ri}==1)+1
    BestTourIndex{ri}=circshift(BestTourIndex{ri}, [0 nb_shift])
    BestTour{ri}=ATRobot{ri}(BestTourIndex{ri},:)
end


% Create the button group
%textwrap('Negociation Type')
 put_option()
% global negbuttongroup optbuttongroup;


uicontrol('Style', 'pushbutton', 'string','Moving','Callback',...
            {@eliminate_common_targets,tfig, nbrTargets,nbrRobots, comm_range, BestTour, Robots, TourDist, filename},'Position', [10 125 60 20])

put_button()

put_menu()


 
end

%%phase: eliminate common targets (overlapped targets)
function eliminate_common_targets(hObject, eventdata, tfig, nbrTargets,nbrRobots, CommRange, BestTour, InitRobotPos, TourDist, filename)

clr=hsv(nbrRobots)
global depots Dim;
global MoveUnit negociate_all negociate_pt_by_pt optimize_total_distance optimize_max_tour Targets;
global negociate_type optimize_type;
global txt  ;
global speed comm_range;
%global negbuttongroup optbuttongroup;
% for ri=1:nbrRobots
%     
%       [BestTourIndex{ri}, TourDist(ri),partfig(ri)]=tsp(BestTour{ri},10,100,1,0);
% end
 read_option()

sprintf('negociation=%i',negociate_type)
sprintf('optimize_type=%i',optimize_type)

current_pos=InitRobotPos';
%delete old robots position 
delete(findobj('Marker','p'));
for ri=1:nbrRobots
    %current_pos(ri,:)=BestTour{ri}(1,:)
    next_target(ri)=1; % next_target=mod(next_target,size(targets,1))+1;
    robot(ri)=line('XData',BestTour{ri}(1,1),'YData',BestTour{ri}(1,2),...
        'Marker','p', 'color',clr(ri,:),'MarkerFaceColor',clr(ri,:),...
                'MarkerSize',12)
%    tourfig(ri)=partfig(ri)
    ATRobot{ri}=remove(InitRobotPos(:,ri)', BestTour{ri});
    
    %variable that indicate if a tour is done
    TourDone(ri)=0;
    %begin moving
    %[current_pos(ri,:) next_target(ri)]=move_robot(ri, current_pos(ri,:),next_target(ri), BestTour{ri},MoveUnit);

end

%CommRange=CommRange*10;
CommRange=comm_range;
MoveUnit=speed;

cfig=tfig;
step=1;
Improved=true;
while Improved || ~all(TourDone>=2) 
    %if isempty(findobj('type','figure','name',sprintf('Final Solution After TSP ::Total Tour Distance'))), return, end
    Improved=0;
    %DELAY=get(speed,'value')
    pause(0.1);
    for ri=1:nbrRobots
        %if isempty(findobj('type','figure','name',sprintf('Final Solution After TSP ::Total Tour Distance'))), return, end
        
        [current_pos(ri,:) next_target(ri)]=move_robot(ri, current_pos(ri,:),next_target(ri), BestTour{ri},MoveUnit);
        set(robot(ri),'XData',current_pos(ri,1),'YData',current_pos(ri,2));
        %if next_target(ri)==1 , TourDone(ri)=1; end
        if(current_pos(ri,:)==InitRobotPos(:,ri)'), TourDone(ri)=TourDone(ri)+1; end
        %%%%%%Communicate with other Robots%%%%%%%%%%%
        %%%%%%In order to eliminate common targets
        CurrentPositions=current_pos';
        INR = nearestneighbour(CurrentPositions(:,ri), CurrentPositions, 'Radius', CommRange);
        INR=INR(INR~=ri);
        %A=INR(find(INR~=ri))
         if size(INR,2)>=1 %There are neighbors
                %compare selected targets of each neighbor robots
                for n=1:size(INR,2)
%                     if find(INR==ri)
%                         sprintf('there is problem')
%                         disp('There is problem in find function')
%                         %error('there is a problem');
%                         waitforbuttonpress 
%                         break;
%                     end
                    %echange the list of Allocated targets with each
                    %neighbor
                    [CTargets, i1, i2]=intersect(ATRobot{ri},ATRobot{INR(n)}, 'rows');
                    if ~isempty(CTargets)
                        %%%compute gain if i remove this target
                        %Perphaps we need to do a new tsp on new targets
                        %pause(0.1);
                        %negotiation on all points
                        if (negociate_type==negociate_all)
                            newtour1=ATRobot{ri}
                            newtour1(i1,:)=[]
                            newtour1=vertcat(depots(ri,:),newtour1)

                            newtour2=ATRobot{INR(n)}
                            newtour2(i2,:)=[]
                            newtour2=vertcat(depots(INR(n),:),newtour2)

                            [NewBestTourIndex{ri}, NewTourDist(ri)]=tsp(newtour1,10,100,1,0);
                            [NewBestTourIndex{INR(n)}, NewTourDist(INR(n))]=tsp(newtour2,10,100,1,0);

                            if optimize_type==optimize_total_distance
                                g1=TourDist(ri)-NewTourDist(ri)
                                g2=TourDist(INR(n))-NewTourDist(INR(n))
                                
                            else
                                g1=max(NewTourDist(ri),TourDist(INR(n)))
                                g2=max(TourDist(ri),NewTourDist(INR(n)))
                            end

                            if (optimize_type==optimize_total_distance && g1>g2) || (optimize_type==optimize_max_tour && g1<g2)
                                rm=ri;
                                BestTour{ri}=newtour1(NewBestTourIndex{ri},:);
                                %ATRobot{ri}(i1,:)=[]
                                %reinitize robot movement
                                [current_pos(ri,:) next_target(ri)]=move_robot(ri, BestTour{ri}(1,:),1, BestTour{ri},MoveUnit);
                                %%%%updating plot
                                if(ishandle(txt(rm)))
                                    set(txt(rm),'String',sprintf('(%.3f) ->\t (%.3f):%.1f',TourDist(rm), NewTourDist(rm),g1))
                                    set(txt(INR(n)),'String',sprintf('(%.3f) <-\t (%.3f):%.1f',TourDist(INR(n)), NewTourDist(INR(n)),g2))
                                else
                                    figure(cfig)
                                    txt(rm)=text(depots(ri,1),depots(ri,2),...
                                        sprintf('(%.3f) ->\t (%.3f):%.1f',TourDist(rm), NewTourDist(rm),g1),...
                                        'Color',clr(rm,:),'HorizontalAlignment','left', 'VerticalAlignment','top')
                                    txt(INR(n))=text(depots(INR(n),1),depots(INR(n),2),...
                                        sprintf('(%.3f) <-\t (%.3f):%.1f',TourDist(INR(n)), NewTourDist(INR(n)),g2),...
                                        'Color',clr(INR(n),:),'HorizontalAlignment','left', 'VerticalAlignment','top')
                               
                                end
                            else
                                rm=INR(n)
                                BestTour{rm}=newtour2(NewBestTourIndex{INR(n)},:)
                                %ATRobot{rm}(i2,:)=[]
                                [current_pos(INR(n),:) next_target(INR(n))]=move_robot(INR(n), BestTour{INR(n)}(1,:),1, BestTour{INR(n)},MoveUnit);
                                if(ishandle(txt(rm)))
                                    set(txt(rm),'String',sprintf('(%.3f) ->\t (%.3f):%.1f',TourDist(rm), NewTourDist(rm),g2))
                                    set(txt(ri),'String',sprintf('(%.3f) <-\t (%.3f):%.1f',TourDist(ri), NewTourDist(ri),g1))
                                else
                                    hold on
                                    figure(cfig)
                                    txt(rm)=text(depots(rm,1),depots(rm,2),...
                                        sprintf('(%.3f) ->\t (%.3f):%.1f',TourDist(rm), NewTourDist(rm),g2),...
                                        'Color',clr(rm,:),'HorizontalAlignment','left', 'VerticalAlignment','top')
                                    txt(ri)=text(depots(ri,1),depots(ri,2),...
                                        sprintf('(%.3f) <-\t (%.3f):%.1f',TourDist(ri), NewTourDist(ri),g1),...
                                        'Color',clr(ri,:),'HorizontalAlignment','left', 'VerticalAlignment','top',...
                                        'parent',gca)
                                    drawnow
                                end
                            end
                        
                        end
                        
                        %negocaition point by point
                        if negociate_type==negociate_pt_by_pt
                            for pti=1:size(CTargets,1)
                                pt=CTargets(pti,:)
                                newtour1=BestTour{ri}
                                newtour2=BestTour{INR(n)}
                                newtour1=remove(pt, BestTour{ri})
                                newtour2=remove(pt, BestTour{INR(n)})

                                [NewBestTourIndex{ri}, NewTourDist(ri)]=tsp(newtour1,10,100,1,0);
                                [NewBestTourIndex{INR(n)}, NewTourDist(INR(n))]=tsp(newtour2,10,100,1,0);


                                if optimize_type==optimize_total_distance
                                    g1=TourDist(ri)-NewTourDist(ri) %the gain of traveled distance when Modification of ri
                                    g2=TourDist(INR(n))-NewTourDist(INR(n)) %the gain of traveled distance when Modification of neighbor of ri

                                else
                                    
                                    g1=max(NewTourDist(ri),TourDist(INR(n))) %the max when Modification of ri
                                    g2=max(TourDist(ri),NewTourDist(INR(n))) %the max when Modification of neighbor of ri
                                end

                                %we wish to minimize the max or to maximize
                                %the gain
                                if (optimize_type==optimize_total_distance && g1>g2) || (optimize_type==optimize_max_tour && g1<g2) 
                                    rm=ri;
                                    BestTour{ri}=newtour1(NewBestTourIndex{ri},:)
                                    
                                    %reinitize robot movement
                                    [current_pos(ri,:) next_target(ri)]=move_robot(ri, BestTour{ri}(1,:),1, BestTour{ri},MoveUnit);
                                    %%%%updating plot
                                    set(txt(rm),'String',sprintf('(%.3f) ->\t (%.3f):%.1f',TourDist(rm), NewTourDist(rm),g1))
                                    set(txt(INR(n)),'String',sprintf('(%.3f) <-\t (%.3f):%.1f',TourDist(INR(n)), NewTourDist(INR(n)),g2))

                                else
                                    rm=INR(n)
                                    BestTour{rm}=newtour2(NewBestTourIndex{INR(n)},:)
                                    [current_pos(INR(n),:) next_target(INR(n))]=move_robot(INR(n), BestTour{INR(n)}(1,:),1, BestTour{INR(n)},MoveUnit);
                                    set(txt(rm),'String',sprintf('(%.3f) ->\t (%.3f):%.1f',TourDist(rm), NewTourDist(rm),g2))
                                    set(txt(ri),'String',sprintf('(%.3f) <-\t (%.3f):%.1f',TourDist(ri), NewTourDist(ri),g1))
                                end
                                

                            end
                            
                        end
                        
                        %update allocated targets
                        ATRobot{rm}=remove(depots(rm,:),BestTour{rm})
                        
                        newfig = figure('Numbertitle','off');
                        scnsize = get(0,'ScreenSize');
                        %set(newfig,'OuterPosition',[scnsize(3)/2 20 scnsize(3)/2 scnsize(4)/2]) 
                        %h=axes('Xlim',[0 Dim],'Ylim',[0 Dim],'Position', [0.15 0.10 0.72 0.800])
                        plot (Targets(:,1),Targets(:,2),'ro') %
                        hold on
                        plot (InitRobotPos(1,:),InitRobotPos(2,:), 'd') %, 'rs'
                        
                        cfig=newfig;
                        figure(cfig)

                        drawnow 
                        Improved=1;
                        
                        %plotting 
%                          hold on
%                          set(get(Newpartfig(rm),'children'),'LineStyle','--','LineWidth',2) %get(Newpartfig(rm),'children'),'color',clr(rm,:),
%                          tourfig(rm)=Newpartfig(rm)
                  
                        %plot not modified Tour
                        TourDist(rm)=NewTourDist(rm)
                        for rj=1:nbrRobots
                            %if rj~=rm
%                                 set(get(tourfig(rj),'children'),'color',clr(rj,:))
%                                 copyobj(get(tourfig(rj),'children'),gca);  
                                %plot tours
                                plot(BestTour{rj}(:,1),BestTour{rj}(:,2), 'Color',clr(rj,:))
                                hold on
                                plot(BestTour{rj}([1,end],1),BestTour{rj}([1,end],2), 'Color',clr(rj,:))
                                
                                txt(rj)=text(depots(rj,1),depots(rj,2),sprintf(' (%.3f)',TourDist(rj)), 'Color',clr(rj,:),...
                                    'HorizontalAlignment','left', 'VerticalAlignment','top')
                            %end
%                            TourDone(rj)=0;%we can re-initialize only rm and neighbor
                        end
                        
                        NewTotalTourDist=sum(TourDist)
                        set(cfig,'Name', sprintf('Repeated Moving step=%i:TTD= %1.4f MaxTour = %1.3f neg=%i opt=%i',...
                            step,NewTotalTourDist, max(TourDist), negociate_type, optimize_type))
                        %title(sprintf('Traveled Distance = %1.3f \t',TourDist));
                        title(sprintf('TotalDistance = %1.3f \t MaxTour = %1.3f \t AvgTour = %1.3f',...
                            sum(TourDist),max(TourDist), mean(TourDist)));
                        %axis([0 Dim 0 Dim]);
                        step=step+1
                        
                        %%%Adding Button
                        put_button()
%                         put_option()
                        %%%Plotting Robot as pentagon
%                         for rj=1:nbrRobots
%                             robot(rj)=line('XData',current_pos(rj,1),'YData',current_pos(rj,2),...
%                                 'Marker','p', 'color',clr(rj,:),'MarkerFaceColor',clr(rj,:),'MarkerSize',12)
%                         end

                    end
                end
         end
                    
                    
            
    end

end

    if  isempty(findobj(cfig,'type','uicontrol','string','MoreImprovement'))
        
        put_option()
        %Adding Slider
        put_slider(cfig)
        
        %Adding Menu
        put_menu()
        %adding mouse click callback
        %set(gcf, 'WindowButtonDownFcn', @view_click_coordinates);        
       
        bt_improv=uicontrol('Style', 'pushbutton', 'string','MoreImprovement','Callback',...
            {@more_optimization,cfig, nbrTargets,nbrRobots, CommRange, BestTour,...
            InitRobotPos, TourDist, filename},'Position', [10 125 60 20])

        bt_mov=uicontrol('Style', 'pushbutton', 'string','Moving','Callback',...
            {@eliminate_common_targets,cfig, nbrTargets,nbrRobots, CommRange, BestTour,...
            InitRobotPos, TourDist, filename},'Position', [10 100 60 20])

    end
save_result(filename, BestTour, TourDist, 'Tour after Eliminating Overlapped Targets')
 
end

 %%%%%%%%%%%%%%%Making robot Moving and communicating
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function more_optimization(hObject, eventdata, tfig, nbrTargets,nbrRobots, CommRange, BestTour, InitRobotPos, TourDist, filename)

global clr Dim;
global MoveUnit   optimize_total_distance optimize_max_tour Targets;
global comm_range;
global negociate_type optimize_type;
global txt;
%global negbuttongroup optbuttongroup;



max_iteration=100;
step=1;
%tourfig=partfig;
%%%Move and Improve 
comm_range=CommRange;
read_option()
read_slider();
CommRange=comm_range;

CurrentPositions=InitRobotPos;
depots=InitRobotPos';
current_pos=CurrentPositions';
%delete old robots position 
delete(findobj('Marker','p'));

%order Best Tour to begin from Depots
for ri=1:nbrRobots
    %sprintf('Best TSP Tour to Robot %i:', ri)
    [tf,loc]=ismember(depots(ri,:),BestTour{ri}, 'rows')
    nb_shift=size(BestTour{ri},1)-loc+1
    nb_shift=mod(nb_shift,size(BestTour{ri},1))
    BestTour{ri}=circshift(BestTour{ri}, [nb_shift 0])
    
end
for ri=1:nbrRobots
    current_pos(ri,:)=BestTour{ri}(1,:)
    next_target(ri)=1;
    robot(ri)=line('XData',current_pos(ri,1),'YData',current_pos(ri,2),...
        'Marker','p', 'color',clr(ri,:),'MarkerFaceColor',clr(ri,:),...
                'MarkerSize',12)
    %tourfig(ri)=partfig(ri)
     TourDone(ri)=0;
    worstTarget{ri}=[];
    NewWorstTarget(ri)=false;
    oldNeigh{ri}=[];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Improved=true;
WasImproved=false;
iteration=0;
while (Improved || ~all(TourDone>=2) )%&& step<max_iteration
    Improved=0;
    iteration=iteration+1
    pause(0.1);
    for ri=1:nbrRobots        
        %%%Move and comm
        [current_pos(ri,:) next_target(ri)]=move_robot(ri, current_pos(ri,:),next_target(ri), BestTour{ri},MoveUnit);
        set(robot(ri),'XData',current_pos(ri,1),'YData',current_pos(ri,2));
        %if next_target(ri)==1, TourDone(ri)=1; end
        if(current_pos(ri,:)==InitRobotPos(:,ri)'), TourDone(ri)=TourDone(ri)+1; end
         %%%each robot must compute some information
        %%such nbr of targets; TourDist; the most worst point 
        nbrTargets(ri)=size(BestTour{ri},1)
        if nbrTargets(ri)>1
            
            %%%search neighbor
            CurrentPositions=current_pos';
            INR{ri} = nearestneighbour(CurrentPositions(:,ri), CurrentPositions, 'Radius', CommRange);
            INR{ri}=INR{ri}(INR{ri}~=ri);
            %A=INR(find(INR~=ri))
            %it is useless to re_bid with old neighbor that we have already
            %bid with them
            INR{ri}=setdiff(INR{ri},oldNeigh{ri}) 
             if size(INR{ri},2)>=1 %There are neighbors
                    %try to exchange targets of each neighbor robots
                    %TourTargets=remove(InitRobotPos(:,ri)',BestTour{ri})
                    %in order to not re-compute worst targets unecessary
                    
                    if isempty(ismember(worstTarget{ri},BestTour{ri}, 'rows'))
                    [worstTarget{ri} NewBestTourIndex{ri} NewTourDist(ri) ]=calc_worst_targets(BestTour{ri}, InitRobotPos(:,ri)')
                    NewWorstTarget(ri)=true;
                    oldNeigh{ri}=[];%neighbor that i have already negociate with then this worst targets
                    end
                    
                    
                    %if ~isempty(worstTarget{ri})
                    %re-initialize loss and gain
                    loss=[];newMax=[];oldMax=[];
                        for n=1:size(INR{ri},2)
                            %if ismember(INR{ri}(n),oldNeigh{ri}) && ~NewWorstTarget(ri), break; end
                            oldNeigh{ri}=[oldNeigh{ri}, INR{ri}(n)]
                            newtour{n}=vertcat(BestTour{INR{ri}(n)}, worstTarget{ri})
                            %compute value of neighbor when adding the targets
                            [NewBestTourIndex{INR{ri}(n)}, NewTourDist(INR{ri}(n))]=tsp(newtour{n},10,100,1,0);
                            %verify which is best for visiting the worst targets
                            if optimize_type==optimize_total_distance
                                  gain_TTD=TourDist(ri)-NewTourDist(ri) %gain of TTD if we remove worst targets from ri
                                  loss(n)=NewTourDist(INR{ri}(n))-TourDist(INR{ri}(n)) %loss
                                  %the winner robot is the robot with the
                                  %small loss in TTD
                                  if all(loss(n)<loss(1:end-1)), win=n, end

                            else %optimize_type==optimize_maxTour
                                  newMax(n)=max(NewTourDist(ri),NewTourDist(INR{ri}(n)))
                                  oldMax(n)=max(TourDist(ri),TourDist(INR{ri}(n)))
                                  %the winner robot is the robot with the
                                  %small new max tour
                                  if all(newMax(n)<newMax(1:end-1)), win=n, end
                                  %we can introduce another parameter
                                  %standard deviation ==>balance load
                                  
                            end
                        end

                            if (optimize_type==optimize_total_distance && gain_TTD>loss(win)) || (optimize_type==optimize_max_tour && newMax(win)<oldMax(win))
                                        rm=INR{ri}(win);
                                        BestTour{ri}=remove(worstTarget{ri},BestTour{ri})
                                        BestTour{ri}=BestTour{ri}(NewBestTourIndex{ri},:);
                                        BestTour{rm}=newtour{win}(NewBestTourIndex{rm},:)
                                        %reinitize robot movement
                                        [current_pos(ri,:) next_target(ri)]=move_robot(ri, BestTour{ri}(1,:),1, BestTour{ri},MoveUnit);
                                        %%%%updating plot
                                        %set(txt(ri),'String',sprintf('(%.3f) ->\t (%.3f):%.1f',TourDist(rm), NewTourDist(rm),g1))
                                        %set(txt(INR(n)),'String',sprintf('(%.3f) <-\t (%.3f):%.1f',TourDist(INR(n)), NewTourDist(INR(n)),g2))
                                        %txt(ri)=text(depots(ri,1),depots(ri,2),sprintf(' (%.3f)',TourDist(ri)), 'Color',clr(ri,:),...
                                         %       'HorizontalAlignment','left', 'VerticalAlignment','top')
                                        %re-initialize worstTarget{ri} and
                                        %rm
                                        worstTarget{ri}=[];
                                        NewWorstTarget(ri)=false;
                                        
                                        
                                        worstTarget{rm}=[];
                                        NewWorstTarget(rm)=false;
                                        oldNeigh{ri}=[];
                                        oldNeigh{rm}=[];
                                        
                                        %remove ri and rm from old neighbor
                                        %list
                                        oldNeigh=cellremove(ri,oldNeigh)
                                        oldNeigh=cellremove(rm,oldNeigh)
                                        %oldNeigh=[]
                                       %cellfun('cellremove(val,C)',oldNeigh)
                                        %NewWorstTarget(ri)=false;
                                        TourDone(ri)=0;
                                        TourDone(rm)=0;
                                        newfig = figure('Numbertitle','off');
                                        scnsize = get(0,'ScreenSize');
%                                         set(newfig,'OuterPosition',[scnsize(3)/2 10 scnsize(3)/2 scnsize(4)/2]) 

                                        plot (Targets(:,1),Targets(:,2),'ro') %
                                        hold on
                                        plot (InitRobotPos(1,:),InitRobotPos(2,:), 'd') %, 'rs'

                                        cfig=newfig;
                                        figure(cfig)

                                        drawnow 
                                        Improved=1;
                                        WasImproved=true;

                                        %plotting 
%                                          hold on
%                                          set(get(Newpartfig(ri),'children'),'LineStyle','--','LineWidth',2) %get(Newpartfig(rm),'children'),'color',clr(rm,:),
%                                          tourfig(ri)=Newpartfig(ri)
% 
%                                          set(get(Newpartfig(rm),'children'),'LineStyle','--','LineWidth',2) %get(Newpartfig(rm),'children'),'color',clr(rm,:),
%                                          tourfig(rm)=Newpartfig(rm)

                                        %plot not modified Tour
                                        dev_bf=std(TourDist)
                                        TourDist(ri)=NewTourDist(ri)
                                        TourDist(rm)=NewTourDist(rm)
                                        dev_af=std(TourDist)
                                        dev_af-dev_bf
                                        for rj=1:nbrRobots
                                            %if rj~=rm
%                                                 set(get(tourfig(rj),'children'),'color',clr(rj,:))
%                                                 copyobj(get(tourfig(rj),'children'),gca);  
                                                plot(BestTour{rj}(:,1),BestTour{rj}(:,2), 'Color',clr(rj,:))
                                                hold on
                                                plot(BestTour{rj}([1,end],1),BestTour{rj}([1,end],2), 'Color',clr(rj,:))
                                                txt(rj)=text(InitRobotPos(1,rj),InitRobotPos(2,rj),sprintf(' (%.3f)',TourDist(rj)), 'Color',clr(rj,:),...
                                                    'HorizontalAlignment','left', 'VerticalAlignment','top')
                                            %end
                                        end

                                        NewTotalTourDist=sum(TourDist)
                                        set(cfig,'Name', sprintf('Improvement Phase-step=%i:TTD = %1.4f neg=%i opt=%i ',...
                                            step,NewTotalTourDist, negociate_type, optimize_type))
                                        %title(sprintf('Traveled Distance = %1.3f \t',TourDist));
                                        title(sprintf('TotalDistance = %1.3f \t MaxTour = %1.3f \t AvgTour = %1.3f',...
                                            sum(TourDist),max(TourDist), mean(TourDist)));
                                        step=step+1;

                                        %%%Adding Button
                                        put_button()
                                        %Adding Slider
                                        %put_slider(pfig)
                                        %Adding Menu
                                        put_menu()
                                        %adding mouse click callback
                                        %set(gcf, 'WindowButtonDownFcn', @view_click_coordinates);
                                        %axis([0 Dim 0 Dim]);
                                        % put_option()
                                        %%%Plotting Robot as pentagon
                                        for rj=1:nbrRobots
                                            robot(rj)=line('XData',current_pos(rj,1),'YData',current_pos(rj,2),...
                                                'Marker','p', 'color',clr(rj,:),'MarkerFaceColor',clr(rj,:),'MarkerSize',12)
                                        end


                            end
                    
                   
             end
        

        end
    end
    

    
end

    if WasImproved
        %if isempty(findobj('name','more')) %&& ~isempty(findobj('name','cfig'))
                figure(cfig)
                put_option()
                more=uicontrol('Style', 'pushbutton', 'string','MoreImprovement','Callback',...
                        {@more_optimization,cfig, nbrTargets,nbrRobots, CommRange, BestTour, InitRobotPos, TourDist},'Position', [10 120 60 20])

                %waitforbuttonpress   
                %Adding Slider
                put_slider(gcf)
                %Adding Menu
                put_menu()
                %adding mouse click callback
                %set(gcf, 'WindowButtonDownFcn', @view_click_coordinates);                
%         end
%     
%%% save result of tour construction
save_result(filename, BestTour, TourDist, 'Best Tour after Improvement')
    end


end
function varargout=calc_worst_targets(Tour, depot)

    minDist=Inf;
    worstTargets=[];
    TourIndex=[1:size(Tour,1)];
%     copyobj(gca,gcf)
fig=gcf;
    if size(Tour,1)>2
        for i=1:size(Tour,1)
                pt=Tour(i,:)
                if pt~=depot
                    newTour=remove(pt, Tour)
                    %[NewBestTourIndex{ri},
                    %NewTourDist(ri),Newpartfig(ri)]=tsp(newtour1,10,100,1,0);
                    [NewTourIndex, NewTourDist]=tsp(newTour,10,100,0,0);
                    if NewTourDist<minDist
                        minDist=NewTourDist
                        worstTargets=pt
                        TourIndex=NewTourIndex
                        %fig=Newpartfig

                    end
                end
        end
        
    elseif size(Tour,1)==2
        minDist=0
        worstTargets=remove(depot, Tour)
        TourIndex=1
    end
    
    varargout{1}=worstTargets
    varargout{2}=TourIndex 
    varargout{3}=minDist
    %varargout{4}=fig

end

%--------------------------------------------------------------------------
%remove(val,C)
function res=cellremove(val, C)

    for i=1:size(C,2)
       C{i}=C{i}(C{i}~=val) 
    end
    res=C
end

function result=add(pt, vector)

    %ismember(pt,vector, 'rows')
    if ~ismember(pt,vector, 'rows')
        result=vertcat(pt,vector)
    else
        result=vector
    end
    
end

function result=remove(pt, vector)

    [mb,loc]=ismember(pt,vector, 'rows')
    if loc>0, vector(loc,:)=[]; , end
    result=vector
    
end
%Moving pt from current_pos to next_target 
% function varargout=move_robot(ri, current_pos, next_target, targets)
% function varargout=move_robot(ri, next_target, targets)
function varargout=move_robot(ri, current_pos, next_target, targets, MoveUnit)

if norm(targets(next_target,:)-current_pos(1,:))>MoveUnit
    theta=atan2((targets(next_target,2)-current_pos(1,2)),(targets(next_target,1)-current_pos(1,1)));
    
    NewX=current_pos(1,1)+MoveUnit*cos(theta);
    NewY=current_pos(1,2)+MoveUnit*sin(theta);
%     robot(ri)=line('XData',NewX,'YData',NewY,'Marker','d', 'color','b','EraseMode','normal')
%     set(robot(ri),'XData',NewX,'YData',NewY)
%     plot(NewX,NewY,'Marker','d', 'color','b')
else
    NewX=targets(next_target,1);
    NewY=targets(next_target,2);
    next_target=mod(next_target,size(targets,1))+1;
%     hold off
end
               
varargout{1}=[NewX NewY];
varargout{2}=next_target;
end

function D = distmat(Robots, Targets)
%DISTMAT Compute euclidian distance matrix from coordinates

nt= size(Targets,1);
nr = size(Robots,1);
%D = zeros(n);
for j = 1:nr
    for k = 1:nt
        D(k,j) = norm(Targets(k,:)-Robots(j,:));
        %Ec(k,j)=sqrt(
    end
end

%D = sqrt(D);

end%   }

%distance between one robot and several targets
function D = cost_dist(Robot, Targets)
%DISTMAT Compute euclidian distance array from coordinates

    nt= size(Targets,1);
    
    for ti = 1:nt
        D(ti,1) = norm(Targets(ti,:)-Robot(1,:));
        
    end
end%   }


%cost using the Robot ad Task mean Allocation algorithm
%one robot and several targets

function C=cost_rtma(Robot, Targets)

    D=cost_dist(Robot, Targets);
    nt= size(Targets,1);
    
    if nt>1
        C=D-mean(D)
    else
        C=D
    end

end

function C=cost_rtmam(Robot, Targets)

    D=cost_dist(Robot, Targets);
  
    nt= size(Targets,1);
    if nt>1
        DistTargets=distmat(Targets, Targets);
        for ti=1:nt                
            C(ti)=D(ti)+mean(nonzeros(DistTargets(ti,:)))
        end
    else
        C=D
    end

end

function C=cost_DMB(RobotX, RobotY, TargetX, TargetY)

%    xR=str2double(RobotX); 
 %  yR=str2double(RobotY);
  %xT=str2double(TargetX);
  %yT=str2double(TargetY);
    C = sqrt((RobotX-TargetX)^2 + (RobotY-TargetY)^2);
  %  C = sqrt((xR-xT)^2 + (yR-yT)^2);

end

function put_button
%         uicontrol('Style', 'pushbutton', 'string','Moving','Callback',...
%             'moving(nbrTargets,nbrRobots, CommRange, Targets, InitRobotPos,negociate_type, optimize_type)','Position', [10 120 60 20])
        % uiwait(gcf,3)
        %uiwait(gcf)
        %if isempty(findobj('type','figure','-regexp','name',sprintf('Final Solution After TSP %s'))), return, end

        uicontrol('Style', 'pushbutton', 'string','Pause','Callback', 'uiwait','Position', [10 20 60 20]);
        uicontrol('Style', 'pushbutton', 'string','Resume','Callback', 'uiresume(gcbf)','Position', [10 50 60 20]);
        uicontrol('Style', 'pushbutton', 'string','Close','Callback', 'close all','Position', [10 80 60 20]);
end
function put_option_bid
global bidbuttongroup ;
bidbuttongroup = uibuttongroup('Title','Bidding Type',...
    'Position',[0.01 0.3 0.12 0.2]);
% Create the radio buttons
uicontrol(bidbuttongroup,'Style','radiobutton',...
    'String','RmSelect',...
    'Units','normalized','Tag','RmSelect',...
    'Position',[0 0 1 0.25]);
uicontrol(bidbuttongroup,'Style','radiobutton',...
    'String','BidSelect',...
    'Units','normalized','Tag','BidSelect',...
    'Position',[0 0.5 1 0.25]);

end

% %function put_option_cost
% global costbuttongroup ;
% costbuttongroup = uibuttongroup('Title','Cost Type',...
%     'Position',[0.01 0.53 0.12 0.25]);
% % Create the radio buttons
% uicontrol(costbuttongroup,'Style','radiobutton',...
%     'String','Distance',...
%     'Units','normalized','Tag','cost_dist',...
%     'Position',[0 0 1 0.25]);
% uicontrol(costbuttongroup,'Style','radiobutton',...
%     'String','RTMA',...
%     'Units','normalized','Tag','cost_rtma',...
%     'Position',[0 0.7 1 0.25]);
% uicontrol(costbuttongroup,'Style','radiobutton',...
%     'String','RTMA+',...
%     'Units','normalized','Tag','cost_rtmam',...
%     'Position',[0 0.35 1 0.25]);
% 
% end
% %function put_option_cost_assign
% global costbuttongroup ;
% 
% costbuttongroup = uibuttongroup('Title','Cost Type',...
%     'Position',[0.01 0.45 0.12 0.3]);
% % Create the radio buttons
% 
% uicontrol(costbuttongroup,'Style','radiobutton',...
%     'String','munkres',...
%     'Units','normalized','Tag','cost_munk',...
%     'Position',[0 0.75 1 0.2]);
% uicontrol(costbuttongroup,'Style','radiobutton',...
%     'String','Hungarian',...
%     'Units','normalized','Tag','cost_hung',...
%     'Position',[0 0.5 1 0.2]);
% uicontrol(costbuttongroup,'Style','radiobutton',...
%     'String','RTMA',...
%     'Units','normalized','Tag','cost_rtma',...
%     'Position',[0 0.25 1 0.2]);
% uicontrol(costbuttongroup,'Style','radiobutton',...
%     'String','Distance',...
%     'Units','normalized','Tag','cost_dist',...
%     'Position',[0 0 1 0.2]);
% uicontrol(costbuttongroup,'Style','radiobutton',...
%     'String','DMB',...
%     'Units','normalized','Tag','cost_DMB',...
%     'Position',[0 0 1.25 0.2]);
% 
% 
% end
% %function read_option_cost
%     global costbuttongroup cost_type ;
%     
%     costbuttongroup=findobj(gcf,'Title','Cost Type')
%     if ishandle(costbuttongroup)
%         cost_type=get(get(costbuttongroup,'SelectedObject'),'Tag')
%     end
%                 
% 
%     
% end
% %function read_option_bid
%     global bidbuttongroup bid_type ;
%     
%     bidbuttongroup=findobj(gcf,'Title','Bidding Type')
%     if ishandle(bidbuttongroup(end))
%         bid_type=get(get(bidbuttongroup(end),'SelectedObject'),'Tag')
%     end
%                 
% 
%     
% end
function read_slider()
    global speed comm_range sl_speed sl_comm_range CommRange;
    %read_option_cost();
    sl_speed=findobj('style', 'slider', 'tag', 'speed', 'parent', gcf)
    if (sl_speed)
        speed=round(get(sl_speed,'value'))
    end
    
    sl_comm_range=findobj('style', 'slider', 'tag', 'comm_range', 'parent', gcf)
    if sl_comm_range
        comm_range=round(get(sl_comm_range, 'value'))
    CommRange=comm_range;
    end
end
% function read_parameter
%     
%     read_option_cost();
%     read_option_bid();
%     read_slider();             
% 
%     
% end
function put_option_point
global ptbuttongroup ;
ptbuttongroup = uibuttongroup('Title','Pt-Gen',...
    'Position',[0.01 0.8 0.12 0.2]);
% Create the radio buttons

uicontrol(ptbuttongroup,'Style','radiobutton',...
    'String','Random',...
    'Units','normalized','Tag','random',...
    'Position',[0 0.6 1 0.25]);
uicontrol(ptbuttongroup,'Style','radiobutton',...
    'String','Manual',...
    'Units','normalized','Tag','manual',...
    'Position',[0 0.1 1 0.25]);
end
function [popSize, numIter, showProg, showResult]=read_option_tsp_ga

global tsp_ga_group
%options=findobj('parent',tsp_ga_group,'style','edit');
options=findobj('parent',tsp_ga_group);
try
popSize=str2num(get(findobj(options,'tag','popSize'),'string'))
numIter=str2num(get(findobj(options,'tag','numIter'),'string'))

showProg=get(findobj(options,'tag','showProg'),'value')
showResult=get(findobj(options,'tag','showResult'),'value')

end



end
function put_option_tsp_ga
%popSize,numIter,showProg,showResult
global tsp_ga_group weight_group;
try 
    delete(weight_group)
end
tsp_ga_group = uibuttongroup('Title','GA Option',...
    'Position',[0.01 0.25 0.14 0.5]);

label_popSize=uicontrol(tsp_ga_group,'Style','text',...
    'String','popSize',...
    'Units','normalized',...
    'Position',[0 0.9 0.8 0.1]);

text_popSize=uicontrol(tsp_ga_group,'Style','edit',...
    'String','100','background', 'white','tag','popSize',...
    'Units','normalized',...
    'Position',[0 0.8 0.8 0.1]);

label_numIter=uicontrol(tsp_ga_group,'Style','text',...
    'String','numIter',...
    'Units','normalized',...
    'Position',[0 0.6 0.8 0.1]);
text_numIter=uicontrol(tsp_ga_group,'Style','edit',...
    'String','1000','background', 'white','tag','numIter',...
    'Units','normalized',...
    'Position',[0 0.5 0.8 0.1]);


box_showProg=uicontrol(tsp_ga_group,'Style','checkbox',...
    'String','showProg','value',1,'tag','showProg',...
    'Units','normalized',...
    'Position',[0 0.35 1 0.1]);

box_showResult=uicontrol(tsp_ga_group,'Style','checkbox',...
    'String','showResult','value',0,'tag','showResult',...
    'Units','normalized',...
    'Position',[0 0.2 1 0.1]);

end
function put_option_weight
global weight_group ;
weight_group = uibuttongroup('Title','Weight',...
    'Position',[0.01 0.25 0.14 0.5]);

% Create the sliders

global text_distance  sl_distance
global text_time text_maxtour text_energy text_variance
text_distance=uicontrol(weight_group,'Style','edit',...
    'String','Distance:1',...
    'Units','normalized',...
    'Position',[0 0.9 0.8 0.1]);
sl_distance=uicontrol(weight_group,'Style','slider',...
    'value',1,'callback', {@sl_distance_Callback}, ...
    'Units','normalized','Tag','distance',...
    'Position',[0.8 0.9 0.2 0.1],...
    'Max', 10, 'Min',0, 'sliderstep', [0.1 0.1]);

text_time=uicontrol(weight_group,'Style','edit',...
    'String','Time:0',...
    'Units','normalized',...
    'Position',[0 0.75 0.8 0.1]);
uicontrol(weight_group,'Style','slider',...
    'String','Time','callback', {@sl_time_Callback},...
    'Units','normalized','Tag','time',...
    'Position',[0.8 0.75 0.2 0.1], 'Max', 10, 'Min',0, 'sliderstep', [0.1 0.1]);

text_energy=uicontrol(weight_group,'Style','edit',...
    'String','Energy:0',...
    'Units','normalized',...
    'Position',[0 0.6 0.8 0.1]);
uicontrol(weight_group,'Style','Slider',...
    'String','Energy','callback', {@sl_energy_Callback},...
    'Units','normalized','Tag','energy',...
    'Position',[0.8 0.6 0.2 0.1], 'Max', 10, 'Min',0, 'sliderstep', [0.1 0.1]);

text_maxtour=uicontrol(weight_group,'Style','edit',...
    'String','MaxTour:1',...
    'Units','normalized',...
    'Position',[0 0.45 0.8 0.1]);
uicontrol(weight_group,'Style','Slider',...
    'String','maxtour','callback', {@sl_maxtour_Callback},...
    'Units','normalized','Tag','maxtour','value', 1,...
    'Position',[0.8 0.45 0.2 0.1], 'Max', 10, 'Min',0, 'sliderstep', [0.1 0.1]);

text_variance=uicontrol(weight_group,'Style','edit',...
    'String','Variance:0',...
    'Units','normalized',...
    'Position',[0 0.3 0.8 0.1]);
uicontrol(weight_group,'Style','Slider',...
    'String','maxtour','callback', {@sl_variance_Callback},...
    'Units','normalized','Tag','variance',...
    'Position',[0.8 0.3 0.2 0.1], 'Max', 10, 'Min',0, 'sliderstep', [0.1 0.1]);

end
function sl_distance_Callback(hObject, eventdata)%, handles
    global text_distance sl_distance
    wd=round(get(gcbo,'value'))
    set(text_distance, 'string',strcat('Distance:',num2str(wd)))
    
end
function sl_time_Callback(hObject, eventdata)%, handles
    global text_time 
    wt=round(get(gcbo,'value'))
    set(text_time, 'string',strcat('Time:',num2str(wt)))
    
end
function sl_energy_Callback(hObject, eventdata)%, handles
    global text_energy
    we=round(get(gcbo,'value'))
    set(text_energy, 'string',strcat('Energy:',num2str(we)))
    
end
function sl_maxtour_Callback(hObject, eventdata)%, handles
    global text_maxtour
    wmt=round(get(gcbo,'value'))
    set(text_maxtour, 'string',strcat('MaxTour:',num2str(wmt)))
    
end
function sl_variance_Callback(hObject, eventdata)%, handles
    global text_variance
    wv=round(get(gcbo,'value'))
    set(text_variance, 'string',strcat('Variance:',num2str(wv)))
    
end

function[wd wmt wt we wv]= read_weight()
    global weight_group
    global weight;
%child=get(weight_group,'Children'))
sliders=findobj(get(weight_group,'Children'), 'style', 'slider')

wd=get(findobj(sliders, 'tag', 'distance'), 'value')
wmt=get(findobj(sliders, 'tag', 'maxtour'), 'value')
wt=get(findobj(sliders, 'tag', 'time'), 'value')
we=get(findobj(sliders, 'tag', 'energy'), 'value')
wv=get(findobj(sliders, 'tag', 'variance'), 'value')

weight=[wd wmt wt we wv]
%return weight;

end
function read_option_point
    global ptbuttongroup pt_gen;
    if ishandle(ptbuttongroup)
        pt_gen=get(get(ptbuttongroup,'SelectedObject'),'Tag')
    end
                

    
end
function put_option
global negbuttongroup optbuttongroup;

if isempty(findobj(gcf,'Title','Negociation Type'))
    negbuttongroup = uibuttongroup('Title','Negociation Type',...
        'Position',[0 0.5 0.1 0.2]);
    % Create the radio buttons
    uicontrol(negbuttongroup,'Style','radiobutton',...
        'String','All pts',...
        'Units','normalized','Tag','all_pts',...
        'Position',[0 0 1 0.2]);
    uicontrol(negbuttongroup,'Style','radiobutton',...
        'String','pt By pt',...
        'Units','normalized','Tag','pt_By_pt',...
        'Position',[0 0.5 1 0.2]);
end
%%%Optimization Type
if isempty(findobj(gcf,'Title','Optimization Type'))
    optbuttongroup = uibuttongroup('Title','Optimization Type',...
        'Position',[0 0.8 0.1 0.2]);
    % Create the radio buttons
    uicontrol(optbuttongroup,'Style','radiobutton',...
        'String','Total Dist',...
        'Units','normalized','Tag','total_dist',...
        'Position',[0 0 1 0.2]);
    uicontrol(optbuttongroup,'Style','radiobutton',...
        'String','Max Tour',...
        'Units','normalized','Tag','max_tour',...
        'Position',[0 0.5 1 0.2]);
end

end    
function read_option
    global negbuttongroup optbuttongroup negociate_type optimize_type;

    negbuttongroup=findobj(gcf,'Title','Negociation Type');
    if ishandle(negbuttongroup)
        switch get(get(negbuttongroup,'SelectedObject'),'Tag')
                    case 'all_pts'
                        negociate_type=1;
                    case 'pt_By_pt'
                        negociate_type=2;

        end
    end
    
    optbuttongroup=findobj(gcf,'Title','Optimization Type');
    if ishandle(optbuttongroup)
        switch get(get(optbuttongroup,'SelectedObject'),'Tag')
                    case 'total_dist'
                        optimize_type=1;
                    case 'max_tour'
                        optimize_type=2;

        end
        
    end
end
function varargout=tsp(xy,popSize,numIter,showProg,showResult)
    
%         [varargout{1},varargout{2},varargout{3}]=tsp_ga_(xy,popSize,numIter,showProg,showResult)
        [varargout{1},varargout{2}]=tsp_ga_m(xy,popSize,numIter)
        %ce

end

%%%% Menu Creation %%%%%%%%
function put_menu()

  global mn_tc;
  global menu_viewnames menu_viewtours;
  %disable current menu
  set(gcf,'menubar', 'none');
  f=uimenu('Label', 'File')
  uimenu(f,'Label','Load Configuration','Callback',{@load_config_motsp});
  uimenu(f,'Label','Save Configuration','Callback',{@save_config_motsp});
  uimenu(f,'Label','Save Result','Callback',{@save_result_motsp}, 'Accelerator','s');
  
  uimenu(f,'Label','Close','Callback','close');
  
  e=uimenu('Label', 'Edit')
  uimenu(e,'Label','Undo','Accelerator','Z','enable','off','Callback', {@undo});
  uimenu(e,'Label','Clear All','Callback',{@clear_all});%'cla'
  uimenu(e,'Label','Clear Targets','Callback',{@clear_targets});
  uimenu(e,'Label','Clear Robots','Callback',{@clear_robots});
  
  v=uimenu('Label', 'View')
  uimenu(v,'Label','All Coordinates','Callback',{@view_coordinates});
  menu_viewnames=uimenu(v,'Label','All Names','Callback',{@view_names});
  uimenu(v,'Label','All Coordinates And Names','Callback',{@view_coordinatesAndName});
  uimenu(v,'Label','Robots Names','Callback',{@view_robotsNames});
  uimenu(v,'Label','Robots Coordinates','Callback',{@view_robotsCoordinates});
  menu_viewtours=uimenu(v,'Label','Tours Cost','Callback',{@view_tours}, 'Enable', 'off');
  
  uimenu(v,'Label','Targets Names','Callback',{@view_targetsNames});
  uimenu(v,'Label','Targets Coordinates','Callback',{@view_targetsCoordinates});
 
  r=uimenu('Label', 'Run')
  uimenu(r,'Label','Go','Accelerator',' ','Callback',{@run}) 
  uimenu(r,'Label','Step-by-Step','Accelerator',' ','Callback',{@execution_mode}, 'checked','off') 
  uimenu(r,'Label','Step','Accelerator','g','Callback','uiresume') 
  uimenu(r,'Label', 'Allocation Phase','Accelerator','1','enable','off' )
  
  mn_tc=uimenu(r,'Label', 'Tour Construction','Accelerator','2','enable','off','Callback',{@tour_construction} )
  uimenu(r,'Label', 'Eliminating common Targets','Accelerator','3','enable','off' )
  uimenu(r,'Label', 'Improvement Phase','Accelerator','4','enable','off' )
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%% SOLVER MENU %%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  s=uimenu('Label', 'Solver')
  menu_tsp=uimenu(s,'Label', 'TSP','Accelerator','T', 'enable','on' )
           uimenu(menu_tsp,'Label', 'GA','tag', 'ga','checked', 'on','Callback',{@set_tsp_approach} ) 
           uimenu(menu_tsp,'Label', 'LKH','tag','lkh','Callback',{@set_tsp_approach} ) 
  uimenu(s,'Label', 'MTSP','Accelerator','M','enable','off' )
  uimenu(s,'Label', 'MDMTSP','Accelerator','D', 'checked','off' ,'Callback',{@solver_mdmtsp})
  uimenu(s,'Label', 'Assignment','Accelerator','N','Callback',{@solver_assign} )
  uimenu(s,'Label', 'Multi-Robot Coord','Accelerator','N','Callback',{@solver_mrcoord} )
  mo=uimenu(s,'Label', 'Multi-Objective TSP','Accelerator','O')
      uimenu(mo,'Label', 'Greedy approach','Accelerator','O','tag', 'greedy','Callback', {@set_mo_approach},'checked','on' )
      uimenu(mo,'Label', 'AHP Greedy Improved','Accelerator','O','tag', 'ahpGrdyImprov','Callback', {@set_mo_approach},'checked','off' )
      uimenu(mo,'Label', 'Genetic Algorithm','Accelerator','','tag','ga','Callback',{@set_mo_approach},'checked','off' )
      uimenu(mo,'Label', 'Genetic Algorithm (Multi-chromosome)','Accelerator','','tag','gamc','Callback',{@set_mo_approach},'checked','off' )
      uimenu(mo,'Label', 'Genetic Algorithm (NSGA-II)','tag','nsga','Callback',{@set_mo_approach},'checked','off' )
      uimenu(mo,'Label', 'FLMTSP (Sahar)','tag','flmtsp','Callback',{@set_mo_approach},'checked','off' )
  
  
  h=uimenu('Label', 'Help')
  uimenu(h,'Label','About','Callback',{@about});
%   uimenu(h,'Label','Clear Targets','Callback',{@clear_targets});
%   uimenu(h,'Label','Clear Robots','Callback',);
end
function put_contextmenu()

  global mn_tc;
  global p pfig;
  %disable current menu
  global cm;
  cm=uicontextmenu;

  set(p, 'uicontextmenu',cm);
  set(pfig, 'uicontextmenu',cm)
  
  uimenu(cm,'Label','Undo','Accelerator','Z','enable','off','Callback', {@undo});
  uimenu(cm,'Label','Clear All','Callback',{@clear_all});%'cla'
  uimenu(cm,'Label','Clear Targets','Callback',{@clear_targets});
  uimenu(cm,'Label','Clear Robots','Callback',{@clear_robots});
  uimenu(cm,'Label','Clear This Target','Callback',{@clear_ThisTarget});
  uimenu(cm,'Label','Clear This Robot','Callback',{@clear_ThisRobot});
  %view_click_coordinates
  uimenu(cm,'Label','This Coordinate','Callback',{@view_click_coordinates});
  uimenu(cm,'Label','All Coordinates','Callback',{@view_coordinates});
  uimenu(cm,'Label','All Names','Callback',{@view_names});
  uimenu(cm,'Label','All Coordinates And Names','Callback',{@view_coordinatesAndName});
  uimenu(cm,'Label','Robots Names','Callback',{@view_robotsNames});
  uimenu(cm,'Label','Robots Coordinates','Callback',{@view_robotsCoordinates});
  uimenu(cm,'Label','Tours Cost','Callback',{@view_tours}, 'Enable', 'off');
  
  uimenu(cm,'Label','Targets Names','Callback',{@view_targetsNames});
  uimenu(cm,'Label','Targets Coordinates','Callback',{@view_targetsCoordinates});
 
  r=uimenu(cm,'Label', 'Run')
  uimenu(r,'Label','Go','Accelerator',' ','Callback',{@run}) 
  uimenu(r,'Label','Step-by-Step','Accelerator',' ','Callback',{@execution_mode}, 'checked','off') 
  uimenu(r,'Label','Step','Accelerator','g','Callback','uiresume') 
  uimenu(r,'Label', 'Allocation Phase','Accelerator','1','enable','off' )
  
  mn_tc=uimenu(r,'Label', 'Tour Construction','Accelerator','2','enable','off','Callback',{@tour_construction} )
  uimenu(r,'Label', 'Eliminating common Targets','Accelerator','3','enable','off' )
  uimenu(r,'Label', 'Improvement Phase','Accelerator','4','enable','off' )
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%% SOLVER MENU %%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  s=uimenu(cm,'Label', 'Solver')
  menu_tsp=uimenu(s,'Label', 'TSP','Accelerator','T', 'enable','on' )
           uimenu(menu_tsp,'Label', 'GA','tag', 'ga','checked', 'on','Callback',{@set_tsp_approach} ) 
           uimenu(menu_tsp,'Label', 'LKH','tag','lkh','Callback',{@set_tsp_approach} ) 
  uimenu(s,'Label', 'MTSP','Accelerator','M','enable','off' )
  uimenu(s,'Label', 'MDMTSP','Accelerator','D', 'checked','off' ,'Callback',{@solver_mdmtsp})
  uimenu(s,'Label', 'Assignment','Accelerator','N','Callback',{@solver_assign} )
  uimenu(s,'Label', 'Multi-Robot Coord','Accelerator','N','Callback',{@solver_mrcoord} )
  mo=uimenu(s,'Label', 'Multi-Objective TSP','Accelerator','O')
      uimenu(mo,'Label', 'Greedy approach','Accelerator','O','tag', 'greedy','Callback', {@set_mo_approach},'checked','on' )
      uimenu(mo,'Label', 'AHP Greedy Improved','Accelerator','O','tag', 'ahpGrdyImprov','Callback', {@set_mo_approach},'checked','off' )
      uimenu(mo,'Label', 'Genetic Algorithm','Accelerator','','tag','ga','Callback',{@set_mo_approach},'checked','off' )
      uimenu(mo,'Label', 'Genetic Algorithm (Multi-chromosome)','Accelerator','','tag','gamc','Callback',{@set_mo_approach},'checked','off' )
      uimenu(mo,'Label', 'Genetic Algorithm (NSGA-II)','tag','nsga','Callback',{@set_mo_approach},'checked','off' )
  
  h=uimenu('Label', 'Help')
  uimenu(h,'Label','About','Callback',{@about});
%   uimenu(h,'Label','Clear Targets','Callback',{@clear_targets});
%   uimenu(h,'Label','Clear Robots','Callback',);
end
%%% Menu function %%%%
function execution_mode (hObject, eventdata, handles)
global debug

  if not(strcmp(get(gcbo, 'Type'), 'uimenu'))
      return
  end
  if  strcmp(get(gcbo, 'Checked'),'off')
      
      set(gcbo, 'Checked','on')
      debug=1
  else
      set(gcbo, 'Checked','off')
      debug=0
  end
end
function set_mo_approach(hObject, eventdata, handles)
global mo_approach solver
solver='momdmtsp'
put_option_weight();
if isempty(mo_approach)
    mo_approach='greedy'
end
set(findobj('type','uimenu','tag', mo_approach), 'checked','off')

set(gcbo,'checked','on')
mo_approach= get(gcbo,'tag')

end
function set_tsp_approach(hObject, eventdata, handles)
global tsp_approach solver
solver='tsp'
if isempty(tsp_approach)
    tsp_approach='ga'
end
set(findobj('type','uimenu','tag', tsp_approach), 'checked','off')

set(gcbo,'checked','on')
tsp_approach= get(gcbo,'tag')

if strcmp(tsp_approach,'ga')
    put_option_tsp_ga
end

end
function solver_assign(hObject, eventdata, handles)
    global solver;
    global DefaultNbreTargets DefaultNbreRobots;
    
    solver='assign';
    ms=findobj('type','uimenu','Label', 'Solver')
    
    mds=findobj('type','uimenu','checked','on','parent',ms )
    set(mds,'checked','off');
    
    as=findobj('type','uimenu','label','Assignment')
    set(as,'checked','on');
    
    %reglage option
    op=findobj(gcf,'Title','Bidding Type')
    delete(op)
    
    op=findobj(gcf,'Title','Cost Type')
    delete(op)
    %set(op,'Position',[0.01 0.53 0.12 0.25]);
    put_option_cost_assign();
    
    uicontrol('Style', 'pushbutton', 'string','DrawAll','Callback',...
            {@drawAll}, 'Position', [5 70 70 20]);
        
   DefaultNbreRobots= DefaultNbreTargets; 

end
function solver_mdmtsp(hObject, eventdata, handles)
    global solver;
    
    solver='mdmtsp';
    ms=findobj('type','uimenu','Label', 'Solver')
    
    mds=findobj('type','uimenu','checked','on','parent',ms )
    set(mds,'checked','off');
    
    as=findobj('type','uimenu','label','MDMTSP')
    set(as,'checked','on');
    
    %reglage option
%     op=findobj(gcf,'Title','Bidding Type')
%     delete(op)
    
    op=findobj(gcf,'Title','Cost Type')
    delete(op)
    
    put_option_cost();
    put_option_bid();
    
    uicontrol('Style', 'pushbutton', 'string','DrawAll','Callback',...
            {@drawAll}, 'Position', [5 70 70 20]);    
    

end

function solver_mrcoord(hObject, eventdata, handles)
    global solver;
    global DefaultNbreTargets DefaultNbreRobots;
    
    solver='mrcoord';
    ms=findobj('type','uimenu','Label', 'Solver')
    
    mds=findobj('type','uimenu','checked','on','parent',ms )
    set(mds,'checked','off');
    
    as=findobj('type','uimenu','label','Multi-Robot Coord')
    set(as,'checked','on');
    
    %reglage option
    op=findobj(gcf,'Title','Bidding Type')
    delete(op)
    
    op=findobj(gcf,'Title','Cost Type')
    delete(op)
    if isempty(findobj(gcf,'Title','Cost Type'))
        put_option_cost()
    end
    
    %set(op,'Position',[0.01 0.53 0.12 0.25]);
    %put_option_cost_assign();
    
    uicontrol('Style', 'pushbutton', 'string','DrawAll','Callback',...
            {@drawAll}, 'Position', [5 70 70 20]);
        
   DefaultNbreRobots= DefaultNbreTargets; 

end

function save_config(hObject, eventdata, handles)
global Targets Robots comm_range speed weight;

%uigetfile();
%uiputfile;
%Ct Opt NR  NT  ComR MU  
param=[size(Targets,1) size(Robots,2) comm_range speed] %we can add other param
read_slider()
%file_name=strcat('config-',size(Targets,1),'T-', size(Robots,2),'R-', comm_range,'CR-', speed,'S.mrs');
file_name=sprintf('config-%iT-%iR-%iCR-%iS.mrs',param);

[filename,PathName,FilterIndex] = uiputfile(file_name)
%line 1 contains parameters
dlmwrite(filename,param,'-append');
dlmwrite(filename,Targets,'-append');
%dlmwrite(filename,size(Robots,2),'-append');
dlmwrite(filename,Robots,'-append');


end
function save_config_motsp(hObject, eventdata, handles)
global Targets Robots comm_range speed weight;

%uigetfile();
%uiputfile;
%Ct Opt NR  NT  ComR MU  
param=[size(Targets,1) size(Robots,2) weight] %we can add other param
%read_slider()
%file_name=strcat('config-',size(Targets,1),'T-', size(Robots,2),'R-', comm_range,'CR-', speed,'S.mrs');
scriptName = mfilename('fullpath')
[currentpath, filename, fileextension]= fileparts(scriptName)
%currentpath=fullfile(currentpath, 'TSPLIB', 'burma14.tsp');

file_name=sprintf('config-%iT-%iR-%iwd-%iwmt-%iwt-%iwe-%iwv.motsp',param);
file_name=fullfile(currentpath, 'scenario', file_name);

[filename,PathName,FilterIndex] = uiputfile(file_name)
%line 1 contains parameters
dlmwrite(filename,param,'-append');
dlmwrite(filename,Targets,'-append');
%dlmwrite(filename,size(Robots,2),'-append');
dlmwrite(filename,Robots,'-append');


end
function save_result_motsp(hObject, eventdata, handles)
global Targets Robots weight filename;
global  globalCost tourcost tour time energy;
%weight=[wd wmt wt we wv]
%uigetfile();
%uiputfile;
%NT NR ComR MU  
param=[size(Targets,1) size(Robots,2) weight]
if isempty(filename)
     %we can add other param
    filename=sprintf('Result-%iT-%iR-%iwd-%iwmt-%iwt-%iwe-%iwv--%s.motsp',param, date);%-%s, cputime
    [filename,PathName,FilterIndex] = uiputfile(filename)
    %dlmwrite(filename,param);%,'-append'
end

%read_slider()

%line 1 contains parameters

%dlmwrite(filename,Targets,'-append');
%dlmwrite(filename,size(Robots,2),'-append');
%dlmwrite(filename,Robots,'-append');
fid = fopen(filename,'a');
%line=strcat('Results of the ', ' ', phase);
fprintf(fid,'Results of the Multi-Objective TSP with the following configuration\n')
fprintf(fid,'%iT-%iR-%iwd-%iwmt-%iwt-%iwe-%iwv\n', param);

for ri=1:size(Robots,2)
    %line=strcat('R', ri, ':')
    %dlmwrite(filename,line,'-append');
    line=[]
    fprintf(fid,'R%d(TD %.f):\n',ri, tourcost(ri))
%     for ti=1:size(ATRobots{ri},1)
%     ind=find(Targets(:,1)==ATRobots{ri}(ti,1));
%     line=[line,ind]
%     end
    dlmwrite(filename,line,'-append');
end

fprintf(fid,'TTD:% .f, MaxTour:%.f, GlobalCost:%.f\n',...
    sum(tourcost), max(tourcost),globalCost );

fclose(fid)
end

function save_result(filename, ATRobots, TravelDist, phase)
global Targets Robots comm_range speed filename;

%uigetfile();
%uiputfile;
%NT NR ComR MU  
if isempty(filename)
    param=[size(Targets,1) size(Robots,2) comm_range speed] %we can add other param
    filename=sprintf('Result-%iT-%iR-%iCR-%iS-%s-%s.mrs',param,phase, date);%-%s, cputime
    [filename,PathName,FilterIndex] = uiputfile(filename)
    dlmwrite(filename,param);%,'-append'
end

read_slider()

%line 1 contains parameters

%dlmwrite(filename,Targets,'-append');
%dlmwrite(filename,size(Robots,2),'-append');
%dlmwrite(filename,Robots,'-append');
fid = fopen(filename,'a');
%line=strcat('Results of the ', ' ', phase);
fprintf(fid,'Results of the %s phase, TTD:% .f, MaxTour:%.f, AvgTour:%.f\n', phase, sum(TravelDist), max(TravelDist),mean(TravelDist));

for ri=1:size(Robots,2)
    %line=strcat('R', ri, ':')
    %dlmwrite(filename,line,'-append');
    line=[]
    fprintf(fid,'R%d(TD %.f):',ri, TravelDist(ri))
    for ti=1:size(ATRobots{ri},1)
    ind=find(Targets(:,1)==ATRobots{ri}(ti,1));
    line=[line,ind]
    end
    dlmwrite(filename,line,'-append');
end

fclose(fid)
end
function load_config_motsp(hObject, eventdata, handles)

global Targets Robots Dim nbrRobots nbrTargets bt_launch;
global cm;

%uigetfile();
%uiputfile;
%file_name;
clear_all(hObject, eventdata);
scriptName = mfilename('fullpath')
[currentpath, filename, fileextension]= fileparts(scriptName)
%currentpath=fullfile(currentpath, 'TSPLIB', 'burma14.tsp');

currentpath=fullfile(currentpath, 'scenario', 'cofig');
[configfilename,PathName,FilterIndex] = uigetfile('*.motsp; *.tsp; *.mrs','Select the Problem file', currentpath)
configfilename=fullfile(PathName, configfilename);
conf=importdata(configfilename, '');
if strcmp(conf(2),'TYPE: TSP')
    str_nb=char(conf(4))
    str_nb=str_nb(1,strfind(str_nb,':')+1:end)
    nbrTargets=eval(str_nb)
    cell_target=conf(9:end-1)
    Targets=str2num(cell2mat(cell_target))
    Targets=Targets(:,2:3)
    
    %return;
elseif strcmp(conf(2),'TYPE: MOTSP')

    %conf=dlmread(configfilename);
    %check on config file
    try
    Line=conf(1,:)
    nbrTargets=Line(1,1)
    nbrRobots=Line(1,2)
    t=num2cell(Line(1,3:end))
    [wd wmt wt we wv]=deal(t{:})
    weight=[wd wmt wt we wv]

            if nbrTargets
                Targets=conf([2:nbrTargets+1],[1:2])
            end
            %robot are written horizentally
            if nbrRobots
                Robots=conf([nbrTargets+2:nbrTargets+2+1],[1:nbrRobots])
            end
    catch

        msgbox('Problem in the configuration file', 'error' )
        return;
    end
else
    load_config(configfilename);
    return

end
       %plot the configuration
        %h=axes('Xlim',[0 Dim],'Ylim',[0 Dim],'Position', [0.08 0.10 0.78 0.800])
        %Dim=max([Targets(:,1);Targets(:,2);Robots(1,:);Robots(2,:)])
        %Dim=100;
        cla
        if ~isempty(Targets)
            plot (Targets(:,1),Targets(:,2),'ro', 'UIContextMenu', cm) 
            hold on
            set(bt_launch, 'enable', 'on');
        end
        if ~isempty(Robots)
            plot (Robots(1,:),Robots(2,:), 'd', 'UIContextMenu', cm) 
        end
        %axis([0 Dim 0 Dim])   
        
end
function load_config(configfilename)

global Targets Robots Dim nbrRobots nbrTargets ATRobot;
global bt_launch mn_tc;
global speed comm_range sl_speed sl_comm_range CommRange;
global cm;
%uigetfile();
%uiputfile;
%file_name;
%[configfilename,PathName,FilterIndex] = uigetfile('*.mrs')
%filename='C:\Users\ASUS\Dropbox\iroboapp\Omar\MATLAB\MoveAndImprove\config-14T-3R_real_phase1-TII.mrs';
conf=dlmread(configfilename);
%check on config file
try
Line=conf(1,:)
switch size(Line,2)
    case 2 
        nbrTargets=Line(1,1)
        nbrRobots=Line(1,2)
        comm_range=200;
        speed=200;
        phase=0;
    case 5
        nbrTargets=Line(1,1)
        nbrRobots=Line(1,2)
        comm_range=Line(1,3);
        speed=Line(1,4);
        phase=Line(1,5);
    otherwise
        msgbox('Problem in the configuration file', 'check config file format' )
end
        if nbrTargets
            Targets=conf([2:nbrTargets+1],[1:2])
        end
        %robot are written horizentally
        if nbrRobots
            Robots=conf([nbrTargets+2:nbrTargets+2+1],[1:nbrRobots])
        end
catch
    
    msgbox('Problem in the configuration file', 'error' )
    return;
end
       %plot the configuration
        %h=axes('Xlim',[0 Dim],'Ylim',[0 Dim],'Position', [0.08 0.10 0.78 0.800])
        %Dim=max([Targets(:,1);Targets(:,2);Robots(1,:);Robots(2,:)])
        %Dim=100;
        plot (Targets(:,1),Targets(:,2),'ro', 'UIContextMenu', cm) 
        hold on
        plot (Robots(1,:),Robots(2,:), 'd', 'UIContextMenu', cm) 
        %axis([0 Dim 0 Dim])   
        
switch phase
    case 0

        %enable launch bt
        set(bt_launch, 'enable', 'on');
        %set speeder slider
        set(sl_speed, 'value', speed);
        %sl_speed_Callback(hObject, eventdata)
        %set comm range slider
        set(sl_comm_range, 'value', comm_range);
        %sl_comm_range_Callback(hObject, eventdata)%, handles
    case 1
        %Go to tour construction phase
        for ri=1:nbrRobots
            ind=nonzeros(conf(nbrTargets+nbrRobots+ri,:))
            ATRobot{ri}=  Targets(ind(2:end),:)
        end
        
        set(mn_tc, 'enable', 'on');
        
        tour_construction()
end
        
        

end
function about(hObject, eventdata, handles)
    msgbox('Developped by ocheikhrouhou@coins-lab.org', 'Developer', 'help');
   
end

%View Related function
function view_pt_coordinates(hObject, eventdata, handles)
 
    global Targets Robots;
    point = get(gca,'CurrentPoint')% button down detected 
    %msgbox(sprintf('%s\t',point))
    point=point(1,1:2)
    [isTgt,loc]=ismember(point, Targets, 'rows')
    [isRob,loc]=ismember(point, Robots', 'rows')
    if (isRob)
        text(point,sprintf('R%i',loc))
    end
end
function view_click_coordinates(hObject, eventdata, handles)
 
    global Targets Robots;
    %line = get(gcf,'CurrentObject')% button down detected 
    
    
    if (strcmp(get(gco,'type'),'line')) %msgbox(sprintf('%s\t',point))
        point_x=get(gco,'Xdata')
        point_y=get(gco,'Ydata')
        point=[point_x point_y]
        pos=get(gca,'currentPoint')
        distances = sqrt((pos(1)-point_x).^2+(pos(3)-point_y).^2)
        %point=point(1,1:2)
        [minValue minIndex] = min(distances)

         [isTgt,loc_t]=ismember([point_x(minIndex) point_y(minIndex)], Targets, 'rows')

        [isRob,loc_r]=ismember([point_x(minIndex) point_y(minIndex)], Robots', 'rows')
        if (isRob && isempty(findobj('type','text','string',sprintf('R%i',loc_r))))
            text(point_x(minIndex),point_y(minIndex),sprintf('R%i',loc_r),'Color',[0 0 1], 'FontSize',8)
        elseif isTgt && isempty(findobj('type','text','string',sprintf('T%i',loc_t)))
            text(point_x(minIndex),point_y(minIndex),sprintf('T%i',loc_t),'Color',[1 0 0], 'FontSize',8)
        end
    end
end
%view_tours
function view_tours(hObject, eventdata, handles)

global Targets Robots;
global  globalCost tourcost tour time energy;
es=20;
   for ri=1:size(Robots,2)
        text (Robots(1,ri)+es,Robots(2,ri)+es, sprintf('Tour%i: %.f',ri, tourcost(ri)),...
            'Color',[0 0 1], 'FontSize',8) 
    end

end
function view_names(hObject, eventdata, handles)

    global Targets Robots;
    %global menu_viewnames
    es=9; %DIM/100
  %if strcmp(get(gcbo, 'Checked'),'off') || 
  if not(strcmp(get(gcbo, 'Type'), 'uimenu'))
      return
  end
  if  strcmp(get(gcbo, 'Checked'),'off') %strcmp(get(gcbo, 'Type'), 'uimenu') &&
    for ti=1:size(Targets,1)
        text (Targets(ti,1)+es,Targets(ti,2),sprintf('T%i',ti),...
            'Color',[1 0 0], 'FontSize',8)
    end
    hold on
    for ri=1:size(Robots,2)
        text (Robots(1,ri)+es,Robots(2,ri), sprintf('R%i',ri),...
            'Color',[0 0 1], 'FontSize',8) 
    end
    if(strcmp(get(gcbo,'Type'),'uimenu'))
        set(gcbo, 'Checked', 'on');
    end
  else
      
        for ti=1:size(Targets,1)
            tn=findobj('type','text','string',sprintf('T%i',ti));
            delete(tn)
        end      
       for ri=1:size(Robots,2)
            rn=findobj('type','text','string',sprintf('R%i',ri));
            delete(rn)
       end
       C=strfind(fieldnames(get(gcbo)), 'Checked')
       
        if(find(not(cellfun(@isempty,C))))
            set(gcbo, 'Checked', 'off');  
        end
  end
   
end  
function view_coordinates(hObject, eventdata, handles)
    %msgbox('Developped by ocheikhrouhou@coins-lab.org', 'Developer', 'help');
    %'Color',clr(ri,:), 'FontSize',8,
    % sprintf('R%i:(%.1f,%.1f)', ri,CurrentPositions(1,ri), % CurrentPositions(2,ri)), 
    global Targets Robots;
    es=0.3;

  if strcmp(get(gcbo, 'Checked'),'off')
    for ti=1:size(Targets,1)
    text (Targets(ti,1)+es,Targets(ti,2),sprintf('(%.1f,%.1f)',Targets(ti,1),Targets(ti,2)),'Color',[1 0 0], 'FontSize',8)
    end
    hold on
    for ri=1:size(Robots,2)
    text (Robots(1,ri)+es,Robots(2,ri), sprintf('(%.1f,%.1f)',Robots(1,ri),Robots(2,ri)),'Color',[0 0 1], 'FontSize',8) 
    end
    set(gcbo, 'Checked', 'on');
  else
       for ti=1:size(Targets,1)
            tn=findobj('type','text','string',sprintf('(%.1f,%.1f)',Targets(ti,1),Targets(ti,2)));
            delete(tn)
        end      
       for ri=1:size(Robots,2)
            rn=findobj('type','text','string',sprintf('(%.1f,%.1f)',Robots(1,ri),Robots(2,ri)));
            delete(rn)
        end
        set(gcbo, 'Checked', 'off');             
      
  end
   
end
function view_coordinatesAndName(hObject, eventdata, handles)
    %msgbox('Developped by ocheikhrouhou@coins-lab.org', 'Developer', 'help');
    %'Color',clr(ri,:), 'FontSize',8,
    % sprintf('R%i:(%.1f,%.1f)', ri,CurrentPositions(1,ri), % CurrentPositions(2,ri)), 
    global Targets Robots;
    es=0.3;
  if strcmp(get(gcbo, 'Checked'),'off')
    for ti=1:size(Targets,1)
        text (Targets(ti,1)+es,Targets(ti,2),sprintf('T%i:(%.1f,%.1f)',ti,Targets(ti,1),Targets(ti,2)),...
            'Color',[1 0 0], 'FontSize',8)
    end
    hold on
    for ri=1:size(Robots,2)
        text (Robots(1,ri)+es,Robots(2,ri), sprintf('R%i:(%.1f,%.1f)',ri,Robots(1,ri),Robots(2,ri)),...
            'Color',[0 0 1], 'FontSize',8) 
    end
    set(gcbo, 'Checked', 'on');
  else
      
        for ti=1:size(Targets,1)
            tn=findobj('type','text','string',sprintf('T%i:(%.1f,%.1f)',ti,Targets(ti,1),Targets(ti,2)));
            delete(tn)
        end      
       for ri=1:size(Robots,2)
            rn=findobj('type','text','string',sprintf('R%i:(%.1f,%.1f)',ri,Robots(1,ri),Robots(2,ri)));
            delete(rn)
        end
        set(gcbo, 'Checked', 'off');       
  end
   
end
function view_robotsCoordinates(hObject, eventdata, handles)
    %msgbox('Developped by ocheikhrouhou@coins-lab.org', 'Developer', 'help');
    %'Color',clr(ri,:), 'FontSize',8,
    % sprintf('R%i:(%.1f,%.1f)', ri,CurrentPositions(1,ri), % CurrentPositions(2,ri)), 
    global  Robots;
    es=0.3;
 if strcmp(get(gcbo, 'Checked'),'off')
    for ri=1:size(Robots,2)
        text (Robots(1,ri)+es,Robots(2,ri), sprintf('R%i:(%.1f,%.1f)',ri,Robots(1,ri),Robots(2,ri)),...
            'Color',[0 0 1], 'FontSize',8) 
    end
    
    set(gcbo, 'Checked', 'on');
 else
        for ri=1:size(Robots,2)
            rn=findobj('type','text','string',sprintf('R%i:(%.1f,%.1f)',ri,Robots(1,ri),Robots(2,ri)));
            delete(rn)
        end
        set(gcbo, 'Checked', 'off');     
 end 
   
end
function view_robotsNames(hObject, eventdata, handles)
    %msgbox('Developped by ocheikhrouhou@coins-lab.org', 'Developer', 'help');
    %'Color',clr(ri,:), 'FontSize',8,
    % sprintf('R%i:(%.1f,%.1f)', ri,CurrentPositions(1,ri), % CurrentPositions(2,ri)), 
    global  Robots;
    es=0.3;
    if strcmp(get(gcbo, 'Checked'),'off')
        for ri=1:size(Robots,2)
            text (Robots(1,ri)+es,Robots(2,ri), sprintf('R%i',ri),...
                'Color',[0 0 1], 'FontSize',8) 
        end
        set(gcbo, 'Checked', 'on');
        %set(gcbo, 'Checked', 'off');
    else 
        for ri=1:size(Robots,2)
            rn=findobj('type','text','string',sprintf('R%i',ri));
            delete(rn)
        end
        set(gcbo, 'Checked', 'off');
    end
 
    
end
function view_targetsCoordinates(hObject, eventdata, handles)
    %msgbox('Developped by ocheikhrouhou@coins-lab.org', 'Developer', 'help');
    %'Color',clr(ri,:), 'FontSize',8,
    % sprintf('R%i:(%.1f,%.1f)', ri,CurrentPositions(1,ri), % CurrentPositions(2,ri)), 
    global Targets ;
    %es=0.3;
    es=0;
 if strcmp(get(gcbo, 'Checked'),'off')   
    for ti=1:size(Targets,1)
        text (Targets(ti,1)+es,Targets(ti,2),sprintf('T%i:(%.1f,%.1f)',ti,Targets(ti,1),Targets(ti,2)),...
            'Color',[1 0 0], 'FontSize',8)
    end
    set(gcbo, 'Checked', 'on');
 else
        for ti=1:size(Targets,1)
            tn=findobj('type','text','string',sprintf('T%i:(%.1f,%.1f)',ti,Targets(ti,1),Targets(ti,2)));
            delete(tn)
        end
        set(gcbo, 'Checked', 'off');
 end
   
   
end
function view_targetsNames(hObject, eventdata, handles)
    %msgbox('Developped by ocheikhrouhou@coins-lab.org', 'Developer', 'help');
    %'Color',clr(ri,:), 'FontSize',8,
    % sprintf('R%i:(%.1f,%.1f)', ri,CurrentPositions(1,ri), % CurrentPositions(2,ri)), 
    global Targets ;
    %es=0.3;
    es=0;
    if strcmp(get(gcbo, 'Checked'),'off')     
        for ti=1:size(Targets,1)
            text (Targets(ti,1)+es,Targets(ti,2),sprintf('T%i',ti),...
                'Color',[1 0 0], 'FontSize',8)
        end
        set(gcbo, 'Checked', 'on');
     else
            for ti=1:size(Targets,1)
                tn=findobj('type','text','string',sprintf('T%i',ti));
                delete(tn)
            end
            set(gcbo, 'Checked', 'off');
    end   
   
end

function put_slider(p)
global Dim speed comm_range MoveUnit;
global sl_speed sl_comm_range speed comm_range ;
        sl_speed=uicontrol('Style','slider','Min',1,'Max',1001, 'Value',speed,...
            'parent',p, 'units', 'Normalized','Tag', 'speed',...
            'sliderstep', [0.01 0.1],'Position',[0.95 0.15 0.03 0.65],'Callback', {@sl_speed_Callback}) 
        label_speed = uicontrol('parent',p,'style','text','string','RobotsSpeed', 'units', 'Normalized',...
            'Position',[0.94 0.9 0.065 0.1]);
        label_speed_value = uicontrol('parent',p,'style','text','string',get(sl_speed, 'value'), 'units', 'Normalized',...
            'Position',[0.94 0.8 0.06 0.1]);

        sl_comm_range=uicontrol('Style','slider','Min',1,'Max',10*Dim, 'Value',comm_range,...
            'parent',p, 'units', 'Normalized','Tag', 'comm_range', ...
            'Position',[0.9 0.15 0.03 0.65],'Callback', {@sl_comm_range_Callback}) 
        label_comm = uicontrol('parent',p,'style','text','string','CommRange', 'units', 'Normalized',...
            'Position',[0.88 0.9 0.06 0.1]);
        label_comm_value = uicontrol('parent',p,'style','text','string',get(sl_comm_range, 'value'), 'units', 'Normalized',...
            'Position',[0.88 0.8 0.06 0.1])
        % hc.add(sl_comm_range, 5, 4, 'MinimumWidth', 200, 'MinimumHeight', 20);%, 'Anchor', 'NorthEast'
        % hc.add(label_comm,5,3, 'MinimumWidth', 80);%, 'Fill', 'Center', 'Anchor', 'NorthEast'
 

end
function sl_speed_Callback(hObject, eventdata)%, handles
           global sl_speed label_speed_value
            speed=round(get(sl_speed,'value'))
            set(label_speed_value, 'string',speed)
            MoveUnit=speed
        end
    
 function sl_comm_range_Callback(hObject, eventdata)%, handles
   global sl_comm_range label_comm_value
   comm_range=round(get(sl_comm_range, 'value'))  
   set(label_comm_value, 'string',comm_range)
            
 end

function clear_all(hObject, eventdata)
global Targets Robots;

Targets=[];
Robots=[];
cla

end
function clear_targets(hObject, eventdata)
global Targets ;

%for manual plt
%if pt_gen=='random'
for ti=1:size(Targets,1)
    ht=findobj('type','line','xdata',Targets(ti,1),'ydata',Targets(ti,2))
    delete(ht);
end

%for random plot
    ht=findobj('type','line','xdata',Targets(1:end,1),'ydata',Targets(1:end,2))
    delete(ht);
Targets=[];

for ti=1:size(Targets,1)
    tn=findobj('type','text','string',sprintf('T%i',ti));
    delete(tn)
end

end
function clear_robots(hObject, eventdata)
global Robots ;

%for manual plot --> each point has a handle
for ri=1:size(Robots,2)
    hr=findobj('type','line','xdata',Robots(1,ri),'ydata',Robots(2,ri))
    delete(hr);
end

%for random plot ->> all points have a single handle with multiple xdata
hr=findobj('type','line','xdata',Robots(1,:),'ydata',Robots(2,:))
delete(hr);
%Targets=[];

Robots=[];

%if there is text
%h=findobj('type','text','-regexp', 'string', strcat('R', num2str(ri)))
for ri=1:size(Robots,2)
    rn=findobj('type','text','string',sprintf('R%i',ri));
    delete(rn)
end

end
function clear_ThisTarget(hObject, eventdata)
global Targets ;

%for manual plt
%if pt_gen=='random'
target=[get(gco,'xdata')', get(gco,'ydata')']
Targets=remove_pt(target, Targets);

delete(gco)

end
function clear_ThisRobot(hObject, eventdata)
global Robots ;

robot=[get(gco,'xdata'); get(gco,'ydata')]

Robots=remove_pt(robot', Robots');
% [mb,loc]=ismember(robot,Robots, 'rows');
% if loc>0, vector(loc,:)=[];  end
% result=vector;
Robots=Robots';
delete(gco)


end

function undo(hObject, eventdata)
global last_plot;
if ~isempty(last_plot) && ishandle(last_plot(end))
    delete(last_plot(end));
    last_plot(end)=[];
end
if isempty(last_plot)
     set(findobj('type','uimenu','label','Undo'),'enable','off')
end

end
function delete_target(hObject, eventdata)
global plot_target Targets

for i=1:size(plot_target, 2)
    if ishandle(plot_target(i))
        delete(plot_target(i));      

    end
end
plot_target=[];
Targets=[];

end
function drawAll(hObject, eventdata)

 global   pfig;
 
%     draw_target(hObject, eventdata, pfig, DefaultNbreTargets)
%     draw_robot(hObject, eventdata, pfig, DefaultNbreRobots)
    draw_target(hObject, eventdata, pfig)
    draw_robot(hObject, eventdata, pfig)
end
function draw_target(hObject, eventdata,pfig)
    
    global pt_gen Targets plot_target last_plot Robots bt_launch Dim DefaultNbreTargets;
    global mn_tc;
    global cm;
    read_option_point();
    if pt_gen=='random'
        
        T=random('unif',0,Dim,DefaultNbreTargets,2) %default number of target is 10
        hold on;
        last_plot(end+1)=plot (T(:,1),T(:,2),'ro', 'UIContextMenu', cm) %hf,
        plot_target(end+1)=last_plot(end)
        
        Targets= vertcat(Targets, T);
    else
        [xi,yi,but] = ginput(1);
        %nt=0;
        while but == 1 
            
            hold on
            last_plot(end+1)=plot(xi,yi,'ro', 'UIContextMenu', cm )
            plot_target(end+1)=last_plot(end)
            %nt = nt+1;
            Targets= vertcat(Targets, [xi yi]);
            
            [xi,yi,but] = ginput(1);
        end
    end
        
    if ~isempty(Targets) && ~isempty(Robots)
        set(bt_launch, 'enable', 'on');
    end
    set(findobj('type','uimenu','label','Undo'),'enable','on')
    

end
function draw_robot(hObject, eventdata,pfig)
    
    global pt_gen last_plot Robots Targets bt_launch Dim DefaultNbreRobots;
    global cm;
    read_option_point();
    if pt_gen=='random'
        
        RobotPos=Dim*rand(2,DefaultNbreRobots)
        hold on
        last_plot(end+1)=plot (RobotPos(1,:),RobotPos(2,:), 'd', 'UIContextMenu', cm); %, 'rs'
        Robots=horzcat(Robots, RobotPos);
    else
        [xi,yi,but] = ginput(1);
        %nr=0;
        while but == 1
            hold on;
            last_plot(end+1)=plot(xi,yi,'d', 'UIContextMenu', cm)
            %nr = nr+1;
            Robots=horzcat(Robots, [xi;yi]);
            
            [xi,yi,but] = ginput(1);
        end
    end
        %Robots=InitRobotPos;
        
      if ~isempty(Targets) && ~isempty(Robots)
        set(bt_launch, 'enable', 'on');   
      end
      
       set(findobj('type','uimenu','label','Undo'),'enable','on')

end



