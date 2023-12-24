clc
clear

global stAllpath;
global Dis_Network;
global Water_Network;
global Rain_Network;
global expect_pop;

load ../Rec/stAllpath100.mat
load ../Rec/Dis_Network
load ../Rec/Water_Network
load ../Rec/Rain_Network
load ../Rec/expect_pop_100.mat
expect_pop = expect_pop_100;

shortest_path_chromosome = [];
for i = 1:length(stAllpath)
    minDisfitness = 99;
    minNum = 0;

    for j = 1:length(stAllpath{i})
        passby_nodes = stAllpath{i}{j}(1:end-1);
        Disfitness = 0;
        for t = 1:2:length(passby_nodes)-1
            Disfitness = Disfitness + Dis_Network(passby_nodes(t), passby_nodes(t+1));
        end
        if Disfitness <minDisfitness
            minDisfitness = Disfitness;
            minNum = j;
        end
    end

    shortest_path_chromosome = [shortest_path_chromosome,minNum];
end

w1 = 1;w2=0.1;w3=0.05;w4=2;w5=2;w6=1;
afLength = length(stAllpath);

shortest_path_chromosome(1:15) = randi(30,15,1)';

[Disfitness,Waterfitness,Rainfitness,Rcfitness,Cafitness,Dpfitness,chromosome] = fitness_evaluate(shortest_path_chromosome);

Disfitness_nor = Disfitness*w1;
Waterfitness_nor = Waterfitness*w2;
Rainfitness_nor = Rainfitness*w3;
Rcfitness_nor = Rcfitness*w4;
Cafitness_nor = Cafitness*w5;
Dpfitness_nor = Dpfitness*w6;

fitness = Disfitness_nor+Waterfitness_nor+Rainfitness_nor+Rcfitness_nor+Cafitness_nor+Dpfitness_nor;
fitvalue = afLength*150./fitness;

shorestpath_fitvalue = [fitvalue,Disfitness_nor,Waterfitness_nor,Rainfitness_nor,Rcfitness_nor,Cafitness_nor,Dpfitness_nor];
% save ./Compare_exepect_and_short/shorestpath_fitvalue  shorestpath_fitvalue

final = 0;

% 适应值评价
function [Disfitness,Waterfitness,Rainfitness,Rcfitness,Cafitness,Dpfitness,chromosome] = fitness_evaluate(chromosome)

    global stAllpath;
    global Dis_Network;
    global Water_Network;
    global Rain_Network;
    global expect_pop;
    afLength = 100;
    shLength = 5;
    shelterIndex = [3402,	1495,	416,	245,	2206];
    %构造避难所容量因素
    shCapacity = zeros(shLength,1);
    shCapacity(:) = ceil(afLength/shLength*1.1);


    
    %各项适应值初始化
    Disfitness = 0;
    Waterfitness = 0;
    Rainfitness = 0;
    Rcfitness = 0;%拥挤度
    Cafitness = 0;%容量
    Dpfitness = 0;% 意愿
    
    passby_nodes_all = [];
    dpArr = [];
    
    for n = 1:afLength

        num = chromosome(n);
        
        %记录经过当前路径经过的节点
        passby_nodes = stAllpath{n}{num}(1:end-1);
    
        %计算经过节点的空间距离适应值
        %计算经过节点的渍水适应值
        %计算经过节点的降雨适应值
        for t = 1:2:length(passby_nodes)-1
            Disfitness = Disfitness + Dis_Network(passby_nodes(t), passby_nodes(t+1));
            Waterfitness = Waterfitness + Water_Network(passby_nodes(t), passby_nodes(t+1));
            Rainfitness = Rainfitness + Rain_Network(passby_nodes(t), passby_nodes(t+1));
        end
        
        %记录车辆一共经过了哪些节点     
        passby_nodes_all = [passby_nodes_all;passby_nodes];
        
        %记录所有的受灾车辆都去了哪个避难所
        dpReal = stAllpath{n}{num}(end-1);
        dpArr = [dpArr,dpReal];
    end
    
    %道路拥挤量Rc评价
    freq = histcounts(passby_nodes_all,max(passby_nodes_all)); 
    roadFreq = freq(freq>10);
    roadFreq_sum = sum(roadFreq);
    len_roadFreq = length(roadFreq);
    %计算拥挤适应值
    Rcfitness = roadFreq_sum/50;
    
    %避难所容量评价       
    ca = histcounts(dpArr,max(dpArr));
    ca = ca(ca>0);
    if length(ca)<shLength
        ca = ones(shLength,1)*999;
    end
    ca_diff = ca-shCapacity';
    caExp = ca_diff(ca_diff>0);
%     Cafitness = sum(1.2.^caExp);
    Cafitness = min(exp(sum(caExp))-1,100);
    
    % 车辆意愿点评价    
    dp_sta = (dpArr ~= shelterIndex(expect_pop));
    Dpfitness = sum(dp_sta);
end
