clear;
clc;
tic;
roads_shape = [];
% ��·��״���������ݶ���
shapes = shaperead('../file/roads');
% ���ص�λ
pixel = 0.01;

% ������ˮ����
filepath='../file/waterlogging.tif';
waterData = imread(filepath);
waterInfo=geotiffinfo(filepath);
% waterLtPoint = waterInfo.BoundingBox(1,1);

% ���뽵ˮ����
filepath='../file/rain_data.tif';
RainData = imread(filepath);
RainInfo=geotiffinfo(filepath); 

% ���·���ؽڵ�RoadNodes
all_points = zeros();
RoadNodes = zeros();
pn = 1;
for i = 1:length(shapes)
%     ��·����㶼������Ҫ�ڵ�
    X = shapes(i).X(1);
    Y = shapes(i).Y(1);
    if IndexArray(RoadNodes,X,Y)==0
        RoadNodes(end+1,1) = X;
        RoadNodes(end,2) = Y;
    end
%     ��·�ߵ��յ�Ҳ��������Ҫ�ڵ�
    fd = length(shapes(i).X)-1;
    X = shapes(i).X(fd);
    Y = shapes(i).Y(fd);
    if IndexArray(RoadNodes,X,Y)==0
        RoadNodes(end+1,1) = X;
        RoadNodes(end,2) = Y;
    end
%     ��·���м�Ѱ�ң����·���ڵ��Ƕ���·�Ľ��㣬������Ϊ��Ҫ�ڵ�
    for j = 2:length(shapes(i).X)-2
       X = shapes(i).X(j);
       Y = shapes(i).Y(j);
       if IndexArray(all_points,X,Y) ~= 0 
           if IndexArray(RoadNodes,X,Y)==0
                RoadNodes(end+1,1) = X;
                RoadNodes(end,2) = Y;
           end
       else
           all_points(end+1,1) = X;
           all_points(end,2) = Y;
       end
   end
end
RoadNodes(1,:) = [];
% scatter(RoadNodes(:,1),RoadNodes(:,2),2)
NodeNum = length(RoadNodes);
% ����·�����۾������ڵϽ�˹�������
% ������
G_Network = zeros(NodeNum);
% ������������
Dis_Network = zeros(NodeNum);
Water_Network = zeros(NodeNum);
Rain_Network = zeros(NodeNum);
G_Network(:,:)=inf;
Dis_Network(:,:)=inf;
Water_Network(:,:)=inf;
Rain_Network(:,:)=inf;

for i=1:NodeNum
    for j=1:NodeNum
        if i==j
            G_Network(i,j)=0;
            Dis_Network(i,j)=0;
            Water_Network(i,j)=0;
            Rain_Network(i,j)=0;
        end
    end
end

toc

% ���ֵ��������ˮֵ
maxWater = 125;
maxRain = 12.9904;
maxDis = 11.9695 ;

% ��ʼ������磬����ĵĲ���
for i=1:length(shapes)
%     �����Ҫ�ڵ�
    nPoints = [shapes(i).X(1),shapes(i).Y(1)];
    tags=(1);
    for j = 2:length(shapes(i).X)-2
        X = shapes(i).X(j);
        Y = shapes(i).Y(j);
%         �ж�xy��������Ҫ�ڵ�������ĸ�λ��index
        if IndexArray(RoadNodes,X,Y)~=0
            tags(end+1)=j;
            nPoints(end+1,1) = X;
            nPoints(end,2) = Y;
        end
    end
    i
    toc
    tags(end+1)=length(shapes(i).X)-1;
%     iÿ��ѭ������shapes��Ȼ���ҵ�������RoadNodes�еĹؼ��ڵ㣬tags��¼���ڵ�ǰshapes�����е�����λ�á�
    nPoints(end+1,1) = shapes(i).X(end-1);
    nPoints(end,2) = shapes(i).Y(end-1);
%     �������
    for t = 1:length(tags)-1
%         �ж���RoadNodes��index��Ҳ��G_network��index��p1��p2���ڱ��G(p1,p2)
        p1 = IndexArray(RoadNodes,nPoints(t,1),nPoints(t,2));
        p2 = IndexArray(RoadNodes,nPoints(t+1,1),nPoints(t+1,2));
        dis = 0;
%         ����shapes��һ����·�У��ؼ��ڵ�֮��Ŀռ����
        for n = tags(t):tags(t+1)-1
            lat1 = shapes(i).Y(n);
            lon1 = shapes(i).X(n);
            lat2 = shapes(i).Y(n+1);
            lon2 = shapes(i).X(n+1);
            dis=dis+geodesic(lat1,lon1,lat2,lon2);
        end
        
%        ��ˮϵ��
        water_value = [];
%        ����ϵ��
        Rain_value = [];

        cut_value = 0;
        rdis = 0;
        for n = tags(t):tags(t+1)-1
            lat1 = shapes(i).Y(n);
            lon1 = shapes(i).X(n);
            lat2 = shapes(i).Y(n+1);
            lon2 = shapes(i).X(n+1);
            rdis=geodesic(lat1,lon1,lat2,lon2);
            if (rdis+cut_value) > pixel
                s = rdis + cut_value;
                for turn = 1:floor(s/pixel)
                    N_lon1 = lon1 + (lon2-lon1)*((pixel-cut_value)/rdis);
                    N_lat1 = lat1 + (lat2-lat1)*((pixel-cut_value)/rdis);
                    rdis = rdis + cut_value - pixel;
                    
                    %��ˮ����
                    xpixel = round((N_lon1 - waterInfo.BoundingBox(1,1))/waterInfo.RefMatrix(2,1));
                    ypixel = round((N_lat1 - waterInfo.BoundingBox(2,2))/waterInfo.RefMatrix(1,2));
                    if xpixel <= 0 || xpixel >=waterInfo.Height || ypixel <= 0 || ypixel >= waterInfo.Width
                        break;
                    end
                    value = waterData(xpixel,ypixel);
                    water_value(end+1) = value;
                    
                    %�������
                    xpixel = round((N_lon1 - RainInfo.BoundingBox(1,1))/RainInfo.RefMatrix(2,1));
                    ypixel = round((N_lat1 - RainInfo.BoundingBox(2,2))/RainInfo.RefMatrix(1,2));
                    if xpixel <= 0 || xpixel >=RainInfo.Height || ypixel <= 0 || ypixel >= RainInfo.Width
                        break;
                    end
                    value = RainData(xpixel,ypixel);
                    Rain_value(end+1) = value;
                    
                    lon1 = N_lon1;
                    lat1 = N_lat1;
                    cut_value = 0;
                end
%                 cut_value = rdis;
            else
%                 cut_value = cut_value+rdis;
            end
        end

%       ��ˮ����
        Water = sum(water_value)/((length(water_value)+0.000001)*maxWater);

%       ��������
        Rain = sum(Rain_value)/((length(Rain_value)+0.00001)*maxRain);

%       �ռ��������
        dis = dis/maxDis;
%       �������߶�����˹�һ��

% ��¼���ֵ
%         if max(water_value)>maxWater
%             maxWater = max(water_value);
%         end
% 
%         if max(Rain_value)>maxRain
%             maxRain = max(Rain_value);
%         end
%   
%         if dis>maxDis
%             maxDis = dis;
%         end
        
        k1 = 0.5;
        k2 = 0.1;
        if shapes(i).oneway == 'F'
            G_Network(p1,p2) = dis+k1*Water+k2*Rain;
            Dis_Network(p1,p2) = dis;
            Water_Network(p1,p2) = Water;
            Rain_Network(p1,p2) = Rain;

        else 
            G_Network(p1,p2) = dis+k1*Water+k2*Rain;
            G_Network(p2,p1) = dis+k1*Water+k2*Rain;

            Dis_Network(p1,p2) = dis;
            Dis_Network(p2,p1) = dis;
            Water_Network(p1,p2) = Water;
            Water_Network(p2,p1) = Water;
            Rain_Network(p1,p2) = Rain;
            Rain_Network(p2,p1) = Rain;

        end
    end
end

save ../Rec/G_Network G_Network;
save ../Rec/Dis_Network Dis_Network;
save ../Rec/Water_Network Water_Network;
save ../Rec/Rain_Network Rain_Network;
save ../Rec/RoadNodes RoadNodes;
save ../Rec/shapes shapes;
toc

function [Index] = IndexArray(all_points,X,Y)
    for n = 1:length(all_points)
       if X == all_points(n,1) &&  Y == all_points(n,2)
           Index = n;
           return 
       end
    end
    Index = 0;
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