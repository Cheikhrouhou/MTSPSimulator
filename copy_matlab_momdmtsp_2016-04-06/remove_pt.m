function result=remove_pt(pt, vector)

    [mb,loc]=ismember(pt,vector, 'rows');
    
    if find(loc)>0, vector(loc(find(loc)),:)=[];  end
    result=vector;
    
end