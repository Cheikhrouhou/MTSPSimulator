function [TourIndex, TourDist]=tsp_solver(Targets, options)
%reading parameters
%global   Targets Robots;
%global tsp_approach;

nbrTargets=size(Targets, 1);
%nbrRobots=size(Robots,2)
switch nbrTargets 
    case 0
        msgbox('Nbre of Targets must not be zero','Change configuration','Warning');
        return;
    case 1
        TourIndex=1;
        TourDist=0;
        return;
    case 2
        TourIndex=[1 2]
        %tour is round trip so dist* 2
        TourDist=norm(Targets(1,:)-Targets(2,:))*2
        return;
    case 3
        TourIndex=[1 2 3]
        TourDist=norm(Targets(1,:)-Targets(2,:))+norm(Targets(2,:)-Targets(3,:))+norm(Targets(3,:)-Targets(1,:))
        return;
end

if nargin>1
    if isfield(options, 'tsp_approach') && ~isempty(options.tsp_approach)
        tsp_approach = options.tsp_approach;
    else
        tsp_approach = 'lkh';
    end
    if isfield(options, 'ShowProg')
        ShowProg = options.ShowProg;
    else
        ShowProg = 0;
    end
    
    if isfield(options, 'popSize')
        popSize = options.popSize;
    else
        popSize = 100;
    end
    if isfield(options, 'numIter')
        popSize = options.numIter;
    else
        numIter = 1000/2;
    end    
    
    
else
    tsp_approach = 'lkh';
    ShowProg = 0;
    
end

switch tsp_approach
    case 'ga'
        %[popSize, numIter, ShowProg, showResult]=read_option_tsp_ga
        [TourIndex, TourDist]=tsp_ga(Targets,popSize, numIter, ShowProg, 0)
        
    otherwise %'lkh'
        
        %creation of a .tsp file containing problem prameter
        %options='';
        [TourIndex, TourDist] = tsp_lkh(Targets);
        %we can here recompute the exact value of LKH
        TourDist=tour_length(Targets, TourIndex)
            
        if ShowProg
            tour=Targets(TourIndex,:)
            figure('Name','TSP_LKH');
            %plot(tour(:,1),tour(:,2),'ro')
            plot(tour(:,1),tour(:,2),'ro-')
            title(sprintf('Total Distance = %1.2f',TourDist));
        end
        
end

end