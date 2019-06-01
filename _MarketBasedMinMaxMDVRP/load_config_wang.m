%Function that permits to load configuration (nbre targets, vehcles,%etc.)
%From standard instances format such as TSPLIB
%Exemple
%E:\0recherche\_Robotic\MATLAB\Benchmark\real01mod.vrp
function [Targets Depots]=load_config_wang(configfilename)

%global Targets Depots nbrRobots nbrTargets ;

try
%configfilename='E:\0recherche\_Robotic\MATLAB\Benchmark\real01modS.vrp'

scriptName = mfilename('fullpath');
[currentpath, filename, fileextension]= fileparts(scriptName);


depotsFile=fullfile(currentpath, '\scenario\', strcat(configfilename, 'd.dat'));
targetsFile=fullfile(currentpath, '\scenario\', strcat(configfilename, 'c.dat'));
catch
    %fopen with gui
    load_config_gui()
    
end


try
      
            Depots=dlmread(depotsFile)
            Depots=Depots(:,2:3)
            %cities are the rest of cordinates
            Targets=dlmread(targetsFile)             
            Targets=Targets(:,2:3)
catch
    
    msgbox('Problem in the configuration file', 'error' )
    return;
end
          

        

end