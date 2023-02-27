clc
clear
tic

% 读入距离网络矩阵
load G_Network
load RoadNodes
roads_shape = [];
% 道路形状与属性数据读入
shapes = shaperead('./file/roads');


% 输入起始点
orgin = [114.461777 , 30.428581];
destination = [114.503893 , 30.484736];


% 寻找离起点终点最近的重节点
startIndex = 0;
DesIndex = 0;
minOrgin_Dis = 99;
minDes_Dis = 99;
for i = 1:length(RoadNodes)
       X = RoadNodes(i,1);
       Y = RoadNodes(i,2);
       Orgin_dis = (X - orgin(1))^2 + (Y-orgin(2))^2;
       Des_dis = (X - destination(1))^2 + (Y-destination(2))^2;
       if minOrgin_Dis > Orgin_dis
           startIndex = i;
           minOrgin_Dis = Orgin_dis;
       end
       if minDes_Dis > Des_dis
           DesIndex = i;
           minDes_Dis = Des_dis;
       end
end
[dp,dpp] = djstl_figure(startIndex,G_Network);
% 重要节点的路径
mroute = dpp(DesIndex);
index = 1;
froute = orgin;
froute(2,1) = RoadNodes(mroute{1}(1),1);
froute(2,2) = RoadNodes(mroute{1}(1),2);
for i = 1:length(mroute{1})-1
   x1 = RoadNodes(mroute{1}(i),1);
   y1 = RoadNodes(mroute{1}(i),2);
   x2 = RoadNodes(mroute{1}(i+1),1);
   y2 = RoadNodes(mroute{1}(i+1),2);
   for j = 1:length(shapes) 
       arrx1 = shapes(j).X == x1;
       arry1 = shapes(j).Y == y1;
       arr1 = arrx1 + arry1;
       arrx2 = shapes(j).X == x2;
       arry2 = shapes(j).Y == y2;
       arr2 = arrx2 + arry2;
       if   sum(arr1)==2 && sum(arr2)==2 
           p1 = find(arr1==2);
           p2 = find(arr2==2);
           for t = p1+1:p2
               froute(end+1,1) = shapes(j).X(t);
               froute(end,2) = shapes(j).Y(t);
           end
           break;
       end 
   end
end
froute(end+1,1) = destination(1);
froute(end,2) = destination(2);


 toc
 
%  djstl规划算法
function [dp,dpp] = djstl_figure(start,mgraph)
n = length(mgraph);
dp = ones(1,n);
dp(:,:) = 9;
dpp = {};
seen = zeros(1,n);

passed = start;
nopass = [];
for x = 1:n
   if x~=start
       nopass(end+1) = x;
   end
end

% 初始化起始点到各点的最短距离
for i = 1:n
   dp(i) = mgraph(start,i);
   dpp{i}= [i];
end

% 迭代查找
for i = 1:length(nopass)
    k = nopass(1);
    min = inf;
%     找寻当前最近的节点
    for j = 1:length(nopass)
        t = nopass(j);
        if dp(t) < min && seen(t) == 0
            min = dp(t);
            k = t;
        end
    end
    seen(k) = 1;
%     计算经过点k是否还有更近的路径
    for j = 1:length(nopass)
        t = nopass(j);
        if dp(t)>dp(k) + mgraph(k,t) && seen(t)==0
            dp(t) = dp(k)+mgraph(k,t);
            dpp{t} = [];
            for x = 1:length(dpp{k})
                dpp{t}(end+1) = dpp{k}(x);
            end
            dpp{t}(end+1) = t;
        end
    end
    tag = nopass == k;
    nopass(tag) = [];
    passed(end+1) = k;
end
end