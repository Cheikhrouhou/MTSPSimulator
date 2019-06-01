function load_config_gui()

global Targets Robots nbrRobots nbrTargets bt_launch;
global cm;


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