function [output] = fuzzyLogicSystem_mamdani(maxMin, minMax, meanMin, meanMax, sumMaxMinMinMax, sumMeanMinMeanMax,val1, val2)
%Part_I :Member-ship Functions 
% [maxMin, minMax, meanMin, meanMax] = buildMFValues();
% sumMaxMinMinMax = maxMin+minMax;
% sumMeanMinMeanMax = meanMin+meanMax;
% Create new Fuzzy Inference System
mtsp=newfis('FMOMTSP.fis');
%  Add variable to Fuzzy Inference System
mtsp=addvar(mtsp,'input','totalTour',[0 sumMaxMinMinMax]); 
mtsp=addmf(mtsp,'input',1,'short','trapmf',[0 0 maxMin minMax]); 
mtsp=addmf(mtsp,'input',1,'long','trapmf',[maxMin minMax sumMaxMinMinMax sumMaxMinMinMax]); 
% mtsp=addmf(mtsp,'input',1,'','trapmf',[0 0 maxMin minMax]); 

mtsp = addvar(mtsp, 'input', 'maxTour', [0 sumMeanMinMeanMax]);%add and input variable 'maxTour' into the FIS
mtsp = addmf(mtsp, 'input', 2, 'short', 'trapmf', [0 0 meanMin meanMax]);%add 2 MFs into the variable y
mtsp = addmf(mtsp, 'input', 2, 'long', 'trapmf', [meanMin meanMax sumMeanMinMeanMax sumMeanMinMeanMax]);
% mtsp = addmf(mtsp, 'input', 2, '', 'trapmf', [0 0 meanMin meanMax]);%add 2 MFs into the variable y

mtsp = addvar(mtsp, 'output', 'assignment', [0 1]);%add and output variable 'assignment' into the FIS
mtsp = addmf(mtsp, 'output', 1, 'good', 'trapmf', [0 0 0.5 0.75]);
mtsp = addmf(mtsp, 'output', 1, 'bad', 'trapmf', [0.5 0.75 1 1]);

% figure  
% subplot(2,2,2); plotmf(mtsp,'input',2); title('maxTour')%input2
% subplot(2,1,2); plotmf(mtsp,'output',1); title('Assignment')%output

% r1 = 'If totalTour is good and maxTour is good then assignment is good';
% r2 = 'If totalTour is bad and maxTour is bad then assignment is bad';
% r = char(r1,r2);
% 
% mtsp = parsrule(mtsp,r);
% showrule(mtsp)

% 
%    output = evalfis([-0.5 0.2],mtsp);
rulelist = [1 1 1 1 1;
            2 2 2 1 1;
            1 2 2 1 1;
            2 1 2 1 1;]; %2 rule is defined and added into FIS
mtsp = addrule(mtsp, rulelist);
% showrule(mtsp)%show rules'if...and...then...'
writefis(mtsp, 'FMOMTSP.fis');

% ruleview FMOMTSP   %show membership figure
b=readfis('FMOMTSP');
% b = setfis(b,'aggMethod', 'min');
% sug_fismat = mam2sug(b);%Transform Mamdani FIS to Sugeno FIS
% sug_fismat.impMethod = 'prod';
% sug_fismat.aggMethod = 'sum';
% % outputs

 FL_input = [val1 val2];
%   output = evalfis(FL_input,sug_fismat);%Sugeno
 output = evalfis(FL_input,b);%Mamdani
%mtsp.type%display FIS type 'mamdani'
end

