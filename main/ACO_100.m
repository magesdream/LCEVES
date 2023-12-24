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


% 参数设置
num_ants = 50; % 蚂蚁数量
num_iterations = 5000; % 迭代次数
evaporation_rate = 0.5; % 信息素挥发率
alpha = 1; % 信息素重要程度
beta = 2; % 启发因子重要程度
pheromone_initial = 0.1; % 初始信息素浓度
num_cars = 100; % 车辆数量
num_paths = 5*6; % 可选路径数量
target_limit = 25; % 目标限制
afLength = 100;

% 生成随机路径长度
path_lengths = rand(num_cars, num_paths);

path_lengths = zeros(100,30);
for i = 1:100
    for j = 1:30
        path_lengths(i,j) = stAllpath{i}{j}(end);
    end
end

% 初始化信息素矩阵
pheromone = pheromone_initial * ones(num_cars, num_paths);

% 初始化最优路径和最优路径长度
best_path = zeros(1, num_cars);
best_length = inf;
bestindividualArr = [];
w1 = 1;w2=0.1;w3=0.05;w4=2;w5=2;w6=1;
bestfitLast = 0;
bestFitvalue = 0;
bestfitArr = [];
% 迭代搜索
for iteration = 1:num_iterations
    iteration
    % 初始化每只蚂蚁的路径和前往点统计
    ant_paths = zeros(num_ants, num_cars);
    ant_targets = zeros(num_ants, num_paths);
    ant_lengths = zeros(num_ants, 1);
    
    % 每只蚂蚁选择路径和前往点
    for ant = 1:num_ants
        for car = 1:num_cars
            % 计算路径选择概率
            probabilities = (pheromone(car, :) .^ alpha) .* (1 ./ path_lengths(car, :) .^ beta);
            probabilities = probabilities / sum(probabilities);
            
            % 轮盘赌选择路径
            selected_path = roulette_wheel_selection(probabilities);
            
            % 更新蚂蚁的路径、前往点和长度
            ant_paths(ant, car) = selected_path;
        end
            chromosome = ant_paths(ant,:);

            [Disfitness,Waterfitness,Rainfitness,Rcfitness,Cafitness,Dpfitness,chromosome] = fitness_evaluate(chromosome);
            Disfitness_nor = Disfitness*w1;
            Waterfitness_nor = Waterfitness*w2;
            Rainfitness_nor = Rainfitness*w3;
            Rcfitness_nor = Rcfitness*w4;
            Cafitness_nor = Cafitness*w5;
            Dpfitness_nor = Dpfitness*w6;
            fitness = Disfitness_nor+Waterfitness_nor+Rainfitness_nor+Rcfitness_nor+Cafitness_nor+Dpfitness_nor;
            ant_lengths(ant) = fitness;
    end
    
    % 更新最优路径
    [min_length, min_index] = min(ant_lengths);
    bestindividual = ant_paths(min_index,:);
    bestfit = afLength*150/min_length;
    fprintf('The Y.0 is --->>%5.2f\n',bestfit); 

    if bestfitLast > bestfit
        ant_paths(randi([1,num_ants]),:) = bestindividualLast;
        bestfit = bestfitLast;
        bestindividual = bestindividualLast;
    end
    
    bestindividualLast = bestindividual;
    bestfitLast = bestfit;
    
    if bestfit > bestFitvalue
        bestFitvalue = bestfit;
        bestCode = bestindividual;
    end
    
    bestfitArr(end+1) = bestfit;

    fprintf('The Y is --->>%5.2f\n',bestfit); 

    if mod(iteration,50) == 0
        pheromone = pheromone_initial * ones(num_cars, num_paths);
    end
    
    % 更新信息素
    pheromone = (1 - evaporation_rate) * pheromone;
    for ant = 1:num_ants
        for car = 1:num_cars
            path = ant_paths(ant, car);
            pheromone(car, path) = pheromone(car, path) + 1 / ant_lengths(ant);
        end
    end
end





% 轮盘赌选择
function selected_index = roulette_wheel_selection(probabilities)
    cum_probabilities = cumsum(probabilities);
    random_value = rand();
    selected_index = find(cum_probabilities >= random_value, 1);
end

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
