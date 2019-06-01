function [costGreedyTour]=greedyFarthestNeighbor(robot) 
global nbTargets
%global nbRobots
global Dist_target_target
global Dist_robots_targets
sortElement = cell(2,nbTargets);
costGreedyTour = 0;
% disp('------------greedyFarthestNeighbor--------------');
     ri = Dist_robots_targets(robot, :);
     k = 0;%used to save element in sortElement
     index = 1;%index of the first target in the matrix of distances
     for j=1:nbTargets
         %select the nearest element
         [valueNearestElement,indexNearestElement]=selectFarthestElement(sortElement, ri, index);%return the index of the nearest element
%        
         k = k + 1;%increment the counter
         sortElement{1,k} = indexNearestElement;
         sortElement{2,k} = valueNearestElement; 
         ri=Dist_target_target(indexNearestElement, :);
     end
      
     for h=1:nbTargets
         costGreedyTour = costGreedyTour + sum(sortElement{2,h});
     end
     costGreedyTour = costGreedyTour +  Dist_robots_targets(robot, sortElement{1,nbTargets});
end
function [farthest,I]=selectFarthestElement(tab, l, i)
%select the nearest element
%I = 0;%
exist = true;
vect=l(i:end);
while (exist == true)
    %find a non zeros min element
    [farthest, I]=findMax(vect);
    %check if the element already exist 
    for j=1:length(tab)
        if(tab{1,j}==I)
            vect(I)=0;
            I=0;
        end
    end
    if(I~=0)
        exist = false;
    end
end
end
function [ele,i]=findMax(vect)
i=0;
ele=0;
i = find(vect~=0, 1, 'first');
ele=vect(i);

for j=i+1:length(vect)
    if((vect(j)~=0)&&(vect(j) > ele))
        i = j;
        ele = vect(j);
    end
end
end
