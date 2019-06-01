function save_config_motsp(Targets, Robots, weight, fname)

param=[size(Targets,1) size(Robots,1) weight] %we can add other param

scriptName = mfilename('fullpath')
[currentpath, filename, fileextension]= fileparts(scriptName)
%currentpath=fullfile(currentpath, 'TSPLIB', 'burma14.tsp');

file_name=sprintf('config-%iT-%iR-%s.motsp',param(1:2), fname);
filename=fullfile(currentpath, 'scenario', file_name);

%[filename,PathName,FilterIndex] = uiputfile(file_name)
%fopen(filename, 'w')
%line 1 contains parameters
dlmwrite(filename,param,'-append');
dlmwrite(filename,Targets,'-append');
%dlmwrite(filename,size(Robots,2),'-append');
dlmwrite(filename,Robots,'-append');


end