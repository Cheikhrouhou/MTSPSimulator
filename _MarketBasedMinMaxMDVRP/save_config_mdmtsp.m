function save_config_mdmtsp(Robots, Targets,  filename)


param=[size(Robots,1) size(Targets,1) ]
%line 1 contains parameters
dlmwrite(filename,param,'-append', 'delimiter', '\t');
dlmwrite(filename,Robots,'-append', 'delimiter', '\t');
dlmwrite(filename,Targets,'-append', 'delimiter', '\t');
%dlmwrite(filename,size(Robots,2),'-append');



end