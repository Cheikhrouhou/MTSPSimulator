function save_result_motsp(Targets, Robots, weight, rtes, tourcost, globalCost, filename)

param=[size(Targets,1) size(Robots,1) weight]
if isempty(filename)
     %we can add other param
    filename=sprintf('Result-%iT-%iR-%iTTD-%iMT-%iBT--%s.motsp',param, date);
    [filename,PathName,FilterIndex] = uiputfile(filename)
    
end


fid = fopen(filename,'a');
%line=strcat('Results of the ', ' ', phase);
%fprintf(fid,'Results of the Multi-Objective TSP with the following configuration\n')
fprintf(fid,'%iT-%iR-%iTTD-%iMT-%iBT\n', param);

for ri=1:size(Robots,1)

    fprintf(fid,'R%d(TD %.f): %s\n',ri, tourcost(ri), num2str(rtes{ri}))

    
end
%globalCost varies from how OF we have
%globalCost=weight(1)*sum(tourcost)+weight(2)*max(tourcost)
fprintf(fid,'TTD:% .f, MaxTour:%.f, BT:% .f, GlobalCost:%.f\n',...
    sum(tourcost), max(tourcost), std(tourcost,1), globalCost );

fclose(fid)
end