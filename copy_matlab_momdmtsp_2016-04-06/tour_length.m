function Length=tour_length(Targets, TourIndex)

if nargin>1
    %make the tour
    Targets=Targets(TourIndex,:);
end
T=circshift(Targets, 1);
diff=Targets-T;
n=arrayfun(@(i) norm(diff(i,:)), 1:size(T,1));
%n is also equal to 
%N = sqrt(sum(abs(diff).^2,2))
Length=sum(n);
end