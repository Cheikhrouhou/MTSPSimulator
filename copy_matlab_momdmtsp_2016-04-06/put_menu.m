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
file_name=sprintf('config-%iT-%iR-%iwd-%iwmt-%iwt-%iwe-%iwv.motsp',param);

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


%uigetfile();
%uiputfile;
%file_name;
scriptName = mfilename('fullpath')
[currentpath, filename, fileextension]= fileparts(scriptName)
currentpath=fullfile(currentpath, 'TSPLIB', 'burma14.tsp');
[configfilename,PathName,FilterIndex] = uigetfile('*.motsp; *.tsp','Select the Problem file', currentpath)
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
            plot (Targets(:,1),Targets(:,2),'ro') 
            hold on
            set(bt_launch, 'enable', 'on');
        end
        if ~isempty(Robots)
            plot (Robots(1,:),Robots(2,:), 'd') 
        end
        %axis([0 Dim 0 Dim])   
        
end
function load_config(configfilename)

global Targets Robots Dim nbrRobots nbrTargets ATRobot;
global bt_launch mn_tc;
global speed comm_range sl_speed sl_comm_range CommRange;
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
        plot (Targets(:,1),Targets(:,2),'ro') 
        hold on
        plot (Robots(1,:),Robots(2,:), 'd') 
        %axis([0 Dim 0 Dim])   
        
switch phase
    case 0
        try
            %enable launch bt
            set(bt_launch, 'enable', 'on');
            %set speeder slider
            set(sl_speed, 'value', speed);
            %sl_speed_Callback(hObject, eventdata)
            %set comm range slider
            set(sl_comm_range, 'value', comm_range);
            %sl_comm_range_Callback(hObject, eventdata)%, handles
        catch
            disp('nothing to dofor lack of button interface');
        end
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
    es=0.3;
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
    es=0.3;
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

if Targets
    ht=findobj('type','line','xdata',Targets(1:end,1),'ydata',Targets(1:end,2))
    delete(ht);
end
Targets=[];

%Robots=[];


end
function clear_robots(hObject, eventdata)
global Robots ;

if Robots
    hr=findobj('type','line','xdata',Robots(1,1:end),'ydata',Robots(2,1:end))
    delete(hr);
end
%Targets=[];

Robots=[];


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
    read_option_point();
    if pt_gen=='random'
        
        T=random('unif',0,Dim,DefaultNbreTargets,2) %default number of target is 10
        hold on;
        last_plot(end+1)=plot (T(:,1),T(:,2),'ro') %hf,
        plot_target(end+1)=last_plot(end)
        
        Targets= vertcat(Targets, T);
    else
        [xi,yi,but] = ginput(1);
        %nt=0;
        while but == 1 
            
            hold on
            last_plot(end+1)=plot(xi,yi,'ro')
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
    read_option_point();
    if pt_gen=='random'
        
        RobotPos=Dim*rand(2,DefaultNbreRobots)
        hold on
        last_plot(end+1)=plot (RobotPos(1,:),RobotPos(2,:), 'd'); %, 'rs'
        Robots=horzcat(Robots, RobotPos);
    else
        [xi,yi,but] = ginput(1);
        %nr=0;
        while but == 1
            hold on;
            last_plot(end+1)=plot(xi,yi,'d')
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



