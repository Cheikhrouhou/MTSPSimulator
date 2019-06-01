function C=cost_rtma(Robot, Targets)

    D=cost_dist(Robot, Targets);
    nt= size(Targets,1);
    
    if nt>1
        C=D-mean(D)
    else
        C=D
    end

end