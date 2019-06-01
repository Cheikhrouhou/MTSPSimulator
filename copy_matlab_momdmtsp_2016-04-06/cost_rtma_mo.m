function C=cost_rtma_mo(Robot, Targets, weight)

[wd wmt wt we wv]=deal(weight(1),weight(2), weight(3), weight(4), weight(5));
    D=cost_dist(Robot, Targets);
    nt= size(Targets,1);
    
    if nt>1
        C=D-[wd*mean(D)+wmt*max(D)]
        %C=wd*C+wmt*max(D)
    else
        C=wd*D+wmt*max(D)
    end

end