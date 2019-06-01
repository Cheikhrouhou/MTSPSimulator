function C=cost_rtma_mo2(Robot, Targets, weight)

[wd wmt wt we wv]=deal(weight(1),weight(2), weight(3), weight(4), weight(5));
    D=cost_dist(Robot, Targets);
    nt= size(Targets,1);
    
 options=struct('tsp_approach', 'lkh' );
    
    UTargets=Targets;
    ri=1;
    tour{ri}=Robot;
    costTable{ri}=[];
    rtma{ri}=cost_rtma(Robot, UTargets);
    while(UTargets)
         costTable{ri}=[];
        for ti=1:size(UTargets,1)
            %candidate tour
            candtour{ri}{ti}=vertcat(tour{ri}, UTargets(ti,:));
            [ind{ri}{ti},candtourcost(ti)]=tsp_solver(candtour{ri}{ti},options);
     
            %candrtmacost(ti)=wd*candtourcost(ti)+wmt*candtourcost(ti);
            candrtmacost(ti)=wd*rtma{ri}(ti,:)+wmt*candtourcost(ti);
            
            costTable{ri}=vertcat(costTable{ri}, [ti, candrtmacost(ti)]);
        end
        
        [targetcost, ti]=min(costTable{ri}(:,2))
         %newtourcost(ri)=candtourcost(ti);
         tour{ri}= candtour{ri}{ti};
         [y,loc]=ismember(UTargets(ti,:), Targets, 'rows')
         C(loc)=targetcost
         UTargets(ti,:)=[];
         
        
        
    end
    C=C';
   

end