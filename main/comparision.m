clc
clear
load ./Compare_data/geneticBestfitArr.mat
genetic_bestfitArr = bestfitArr;

load ./Compare_data/LECVESbestfitArr.mat
LECVESbestfitArr = bestfitArr;

load ./Compare_data/ACOBestfitArr.mat
ACOBestfitArr = bestfitArr;

load ./Compare_data/bestfitArrGaIII.mat
GaIIIBestfitArr = bestfitArr;

range = 1:5000;
matrix   = ones(length(range),1);
figure('Color', 'white'); % 设置背景颜色为纯白色

hold on;

plot(range, LECVESbestfitArr,'r-', 'DisplayName', 'LECVES');
plot(range, GaIIIBestfitArr,'y-', 'DisplayName', 'NSGA-III');
plot(range, genetic_bestfitArr,'g-', 'DisplayName', 'GA');
plot(range, ACOBestfitArr,'b-', 'DisplayName', 'ACO');




text(30,ACOBestfitArr(end)+2,num2str(ACOBestfitArr(end)))
text(30,GaIIIBestfitArr(end)+1.3,num2str(GaIIIBestfitArr(end)))
text(-620,genetic_bestfitArr(end),num2str(genetic_bestfitArr(end)))
text(-700,LECVESbestfitArr(end),num2str(LECVESbestfitArr(end)))



legend('show','Location', 'southeast');
legend('AutoUpdate', 'off');

plot(range, matrix.*genetic_bestfitArr(end),'k:');
plot(range, matrix.*LECVESbestfitArr(end),'k:');
plot(range, matrix.*ACOBestfitArr(end),'k:');
plot(range, matrix.*GaIIIBestfitArr(end),'k:');

hold off;

xlabel('Generation');
ylabel('Fitness');

box on;

title(' ');


