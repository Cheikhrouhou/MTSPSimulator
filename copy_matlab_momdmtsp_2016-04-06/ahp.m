function weight=ahp(compMat)

[V,D] = eig(compMat);
%d=diag(D);
[m,i]=max(diag(D))
vect=V(:,i);

weight=vect/sum(vect)
end