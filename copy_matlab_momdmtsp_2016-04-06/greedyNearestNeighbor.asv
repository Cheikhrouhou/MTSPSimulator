function [costGreedyTour]=greedyNearestNeighbor(robot) 
global nbTargets
%global nbRobots
global Dist_target_target
global Dist_robots_targets
sortElement = cell(2,nbTargets);
costGreedyTour = 0;
% disp('------------greedyNearestNeighbor--------------');
     ri = Dist_robots_targets(robot, :);
     k = 0;%used to save element in sortElement
     index = 1;%index of the first target in the matrix of distances
     for j=1:nbTargets
         %select the nearest element
%          disp(j);
         [valueNearestElement,indexNearestElement]=selectNearestElement(sortElement, ri, index);%return the index of the nearest element
%        disp(valueNearestElement);
         k = k + 1;%increment the counter
         sortElement{1,k} = indexNearestElement;
         sortElement{2,k} = valueNearestElement;
%          disp(indexNearestElement);
%          disp(valueNearestElement);
         ri=Dist_target_target(indexNearestElement, :);
     end
     
     for h=1:nbTargets
         costGreedyTour = costGreedyTour + sum(sortElement{2,h});
     end
     costGreedyTour = costGreedyTour +  Dist_robots_targets(robot, sortElement{1,nbTargets});
     %disp(costGreedyTour);
end
function [nearest,I]=selectNearestElement(tab, l, i)%i: index of the first target in the matrix
%select the nearest element
%I = 0;%
global nbTargets
nbTargets=size(tab,2);

exist = true;
vect=l(1:nbTargets);
% disp(length(vect));
while (exist == true)
    %find a non zeros min element
    [nearest, I]=findMin(vect);
%     disp(I);
    %check if the element already exist 
    for j=1:length(tab)
       
        if(tab{1,j}==I)
%              disp(tab{1,j});vect(I)
            vect(I)=0;
            I=0;
        end
    end
    if((I~=0) )
        exist = false;
    end
end
end
function [ele,i]=findMin(vect)
i=0;
ele=0;
i = find(vect~=0, 1, 'first');
ele=vect(i);

for j=i+1:length(vect)
    if((vect(j)~=0)&&(vect(j) < ele))
        i = j;
        ele = vect(j);
    end
end
%disp(['i=' num2str(i)]);
end
