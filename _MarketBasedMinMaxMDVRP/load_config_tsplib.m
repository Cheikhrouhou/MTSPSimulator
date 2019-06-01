%Function that permits to load configuration (nbre targets, vehcles,%etc.)
%From standard instances format such as TSPLIB
%Exemple
%E:\0recherche\_Robotic\MATLAB\Benchmark\real01mod.vrp
function load_config_tsplib(configfilename, nbdepots)

global Targets Depots nbrRobots nbrTargets ;

try
%configfilename='E:\0recherche\_Robotic\MATLAB\Benchmark\real01modS.vrp'
%header lines depends on file type
%hl=6
cc=importdata(configfilename,'');
hl=strmatch('NODE_COORD_SECTION',cc);
catch
    %fopen with gui
    load_config_gui()
    
end

%read data
fid=fopen(configfilename)
conf=textscan(fid,'%d %d %d', 'headerlines', hl);
M=cell2mat(conf);
%check on config file
try
        if nbdepots
            Depots=M(1:nbdepots,2:end)
        end
        
        %cities are the rest of cordinates
           Targets=M(nbdepots+1:end,2:end)             
        
catch
    
    msgbox('Problem in the configuration file', 'error' )
    return;
end
          

        

end