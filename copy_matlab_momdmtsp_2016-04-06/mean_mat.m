function R=mean_mat(mat,ni,nf,stp)
R=[];
for n=ni:stp:nf
    grp=mat(find(mat(:,2,:)==n),:,:);
    grp=mean(grp,1);
    R=vertcat(R,grp);
end