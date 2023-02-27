clear;
clc;
array = [1,2;3,4];
X = 3;
Y = 4;

T = InArray(array,X,Y);



mgraph = [[0, 1, 12, inf, inf, inf],
              [inf, 0, 9, 3, inf, inf],
              [inf, inf, 0, inf, 5, inf],
              [inf, inf, 4, 0, 13, 15],
              [inf, inf, inf, inf, 0, 4],
              [inf, inf, inf, inf, inf, 0]]
[d,dpp] = djstl_figure(1,mgraph)
function [Is] = InArray(all_points,X,Y)
    for n = 1:length(all_points)
       if X == all_points(n,1) &&  Y == all_points(n,2)
           Is = 1;
           return 
       end
    end
    Is = 0;
end

function rad = radians(degree)
rad = degree .* pi / 180;
end

function [distance]=geodesic(lat1,lon1,lat2,lon2)
dlat = radians(lat2-lat1);
dlon = radians(lon2-lon1);
lat1 = radians(lat1);
lat2 = radians(lat2);
a = (sin(dlat./2)).^2 + cos(lat1) .* cos(lat2) .* (sin(dlon./2)).^2;
c = 2 .* asin(sqrt(a));
distance = 6372.8 *c;
end

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


