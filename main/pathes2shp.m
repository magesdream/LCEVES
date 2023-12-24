clc
clear
load ./bestExp5000/s_route.mat
load Optimal_pathes.mat


road = s_route{1};
lineObj = geoshape(road(1:end,2), road(1:end,1));
lineArr = lineObj;

for i =2:length(s_route)
    road = s_route{i};
    lineObj = geoshape(road(1:end,2), road(1:end,1));
    lineArr = [lineArr;lineObj];
end

% 保存为SHP文件
filename = 'lines_show.shp';
shapewrite(lineArr,filename);
