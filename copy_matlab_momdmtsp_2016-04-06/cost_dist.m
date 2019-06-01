function D = cost_dist(Robot, Targets)
% Compute euclidian distance array from coordinates

    nt= size(Targets,1);
    
    for ti = 1:nt
        D(ti,1) = norm(Targets(ti,:)-Robot);
        
    end
end%   }