%Function that permits to load configuration (nbre targets, vehcles,%etc.)
%From standard instances format such as TSPLIB
%Exe
%E:\0recherche\_Robotic\MATLAB\Benchmark\real01mod.vrp
function load_config(configfilename)

global Targets Robots nbrRobots nbrTargets ATRobot;
global bt_launch mn_tc;
global speed comm_range sl_speed sl_comm_range ;
global cm;

%configfilename='E:\0recherche\_Robotic\MATLAB\Benchmark\real01modS.vrp'
fid=fopen(configfilename)
%header lines depends on file type
hl=7
conf=textscan(fid,'%d %d %d', 'headerlines', hl);
M=cell2mat(conf)
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