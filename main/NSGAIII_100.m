clear;
clc;
tic;

global BnumArr;
global stAllpath;
global Dis_Network;
global Water_Network;
global Rain_Network;
global shLength;
global shCapacity;
global expect_pop;
global shelterIndex;
global fornum;
load ../Rec/affected_100.mat;
affected = affected_100;
% 构建预设想去点
load ../Rec/expect_pop_100.mat;
expect_pop = expect_pop_100;

afLength = length(affected);
load ../Rec/shelter.mat;
shLength = length(shelter);

stAllpathFile = '../Rec/stAllpath100.mat';
load ../Rec/stAllpath100

%种群大小 自行决定
popsize=20;

%交叉概率
pc = 0.7;
%变异概率
pm = 0.01;
% 迭代次数
fornum = 50000;

% 读入地图
load ../Rec/shapes.mat
load ../Rec/G_Network
load ../Rec/RoadNodes

load ../Rec/Dis_Network
load ../Rec/Water_Network
load ../Rec/Rain_Network

% 绘制地图
% for x = 1:length(shapes)
% plot(shapes(x).X ,shapes(x).Y,'b')
% hold on;
% end

toc;
% 先计算所有最短路径值

% % 计算风险点和避难点的index
affectedIndex = [];
shelterIndex = [];
minafDis = zeros(afLength,1);
minshDis = zeros(shLength,1);
minafDis(:)=99;
minshDis(:)=99;
for i = 1:length(RoadNodes)
       X = RoadNodes(i,1);
       Y = RoadNodes(i,2);

       afDis = (X - affected(:,1)).^2 + (Y-affected(:,2)).^2;
       shDis = (X - shelter(:,1)).^2 + (Y-shelter(:,2)).^2;
       for t=1:afLength
           if afDis(t)<minafDis(t)
               minafDis(t) = afDis(t);
               affectedIndex(t) = i;
           end
       end
       for t=1:shLength
           if shDis(t)<minshDis(t)
               minshDis(t) = shDis(t);
               shelterIndex(t) = i;
           end
       end
end

%初始种群
% 根据避难所数量而定
BnumArr = zeros(afLength,1);
len_exp_pop = length(expect_pop);
for i = 1:afLength
%     计算每个受灾车辆的可选方案数量，并转化为对应的二进制长度
   BnumArr(i) =  Bnumfigure(length(stAllpath{i}));
end
% 将二进制长度求和，生成种群
pop = round(rand(popsize,sum(BnumArr)));

%构造避难所容量因素
shCapacity = zeros(shLength,1);
shCapacity(:) = ceil(afLength/shLength*1.1);

bestFitvalue = 0;
bestCode = [];
bestindividualMid = [];
bestfitMid = 0;
disp("画图时间:")
toc;

bestfitArr  = [];

tic
disp("进化算法开始")
w1 = 1;w2=0.1;w3=0.05;w4=2;w5=2;w6=1;

DisfitnessArr = zeros(fornum,1);
WaterfitnessArr = zeros(fornum,1);
RainfitnessArr = zeros(fornum,1);
RcfitnessArr = zeros(fornum,1);
CafitnessArr = zeros(fornum,1);
DpfitnessArr = zeros(fornum,1);

% exp_pop = produce_expected_pop(100);

% 定义问题参数和函数
numVariables = 10; % 路径选择的决策变量数量
numObjectives = 2; % 目标函数数量
lowerBounds = zeros(1, numVariables); % 决策变量下界
upperBounds = ones(1, numVariables); % 决策变量上界

% 定义算法参数
populationSize = 100; % 种群大小
maxGenerations = 100; % 最大迭代次数

% 初始化种群
% population = pop;

% 迭代优化过程
for generation = 1:fornum
    disp(generation)
    objvalue = zeros(popsize,1);

    %各项适应值初始化
    Disfitness = 0;
    Waterfitness = 0;
    Rainfitness = 0;
    Rcfitness = 0;%拥挤度
    Cafitness = 0;%容量
    Dpfitness = 0;% 意愿
    for m = 1:popsize
        chromosome = pop(m,:);
        % 计算适应度值（函数值）
        [Disfitness,Waterfitness,Rainfitness,Rcfitness,Cafitness,Dpfitness,chromosome] = fitness_evaluate(chromosome);
        pop(m,:) = chromosome;
        Disfitness_nor = Disfitness*w1;
        Waterfitness_nor = Waterfitness*w2;
        Rainfitness_nor = Rainfitness*w3;
        Rcfitness_nor = Rcfitness*w4;
        Cafitness_nor = Cafitness*w5;
        Dpfitness_nor = Dpfitness*w6;
%         fitness = [Disfitness,Waterfitness,Rainfitness,Rcfitness,Cafitness,Dpfitness]*[w1,w2,w3,w4,w5,w6]';
        fitness = Disfitness_nor+Waterfitness_nor+Rainfitness_nor+Rcfitness_nor+Cafitness_nor+Dpfitness_nor;
        objvalue(m) = objvalue(m)+ fitness;
    end

    fitvalue = afLength*150./objvalue;

        %寻找最优解
    [bestindividual,bestfit] = best(pop,fitvalue);
    
    if bestfitMid > bestfit
        pop(ceil(rand*popsize),:) = bestindividualMid;
        bestfit = bestfitMid;
        bestindividual = bestindividualMid;
    end
    bestindividualMid = bestindividual;
    bestfitMid = bestfit;

    % 非支配排序和拥挤度计算
    [fronts, crowdingDistances] = nonDominatedSorting(fitvalue);
    
    % 环境选择
    newpop = environmentalSelection(pop, fronts, crowdingDistances, popsize);
    
    %交叉操作
%     newpop = crossover(newpop,pc,BnumArr);
    %变异操作
    newpop = mutation(newpop,pm);
    
    % 更新种群
    pop = newpop;
    
    %更新种群
    pop = newpop;
    
    if bestfit > bestFitvalue
        bestFitvalue = bestfit;
        bestCode = bestindividual;
    end

    [Disfitness,Waterfitness,Rainfitness,Rcfitness,Cafitness,Dpfitness,~] = fitness_evaluate(bestCode);

    DisfitnessArr(i) = Disfitness*w1;
    WaterfitnessArr(i) = Waterfitness*w2;
    RainfitnessArr(i) = Rainfitness*w3;
    RcfitnessArr(i) = Rcfitness*w4;
    CafitnessArr(i) = Cafitness*w5;
    DpfitnessArr(i) = Dpfitness*w6;

    fprintf('The Y is --->>%5.2f\n',bestfit);
    bestfitArr(end+1) = bestfit;

    if bestFitvalue >101
        break;
    end
end

fprintf('The best Y is --->>%5.2f\n',bestFitvalue);
figure('Color', 'white'); % 设置背景颜色为纯白色
plot(bestfitArr)
title("Fitness");
xlabel("Evolutionary algebra");
ylabel("适应值");

% 初始化种群
function population = initializePopulation(populationSize, numVariables, lowerBounds, upperBounds)
    population = rand(populationSize, numVariables) .* (upperBounds - lowerBounds) + lowerBounds;
end


% 非支配排序和拥挤度计算
function [fronts, ranks] = nonDominatedSorting(objectives)

    numIndividuals = size(objectives, 1);
    dominates = false(numIndividuals, numIndividuals);
    dominatedByCount = zeros(numIndividuals, 1);
    fronts = cell(1);
    ranks = zeros(numIndividuals, 1);
    
    % 计算支配关系
    for i = 1:numIndividuals
        for j = i+1:numIndividuals
            if dominatesIndividual(objectives(i,:), objectives(j,:))
                dominates(i,j) = true;
                dominatedByCount(j) = dominatedByCount(j) + 1;
            elseif dominatesIndividual(objectives(j,:), objectives(i,:))
                dominates(j,i) = true;
                dominatedByCount(i) = dominatedByCount(i) + 1;
            end
        end
    end
    
    % 确定第一个前沿
    fronts{1} = find(dominatedByCount == 0);
    currentFront = 1;
    
    % 逐步确定后续前沿
    while ~isempty(fronts{currentFront})
        nextFront = [];
        for i = 1:numel(fronts{currentFront})
            p = fronts{currentFront}(i);
            for j = 1:numIndividuals
                if dominates(p,j)
                    dominatedByCount(j) = dominatedByCount(j) - 1;
                    if dominatedByCount(j) == 0
                        nextFront = [nextFront j];
                        ranks(j) = currentFront + 1;
                    end
                end
            end
        end
        currentFront = currentFront + 1;
        fronts{currentFront} = nextFront;
    end
end

% 判断个体A是否支配个体B
function dominates = dominatesIndividual(A, B)
    dominates = all(A <= B) && any(A < B);
end

% 环境选择
function selectedPopulation = environmentalSelection(population, fronts, crowdingDistances, populationSize)
    numFronts = numel(fronts);
    selectedPopulation = zeros(populationSize, size(population, 2));
    selectedCount = 0;
    currentFront = 1;
    
    % 选择前沿中的个体直到种群大小达到限制
    while selectedCount < populationSize
        if selectedCount + numel(fronts{currentFront}) <= populationSize
            % 当前前沿的个体数不超过种群大小限制
            selectedPopulation(selectedCount+1:selectedCount+numel(fronts{currentFront}), :) = population(fronts{currentFront}, :);
            selectedCount = selectedCount + numel(fronts{currentFront});
        else
            % 当前前沿的个体数超过种群大小限制，使用拥挤度选择
            remainingCount = populationSize - selectedCount;
            frontIndices = fronts{currentFront};
            [~, sortedIndices] = sort(crowdingDistances(frontIndices), 'descend');
            selectedPopulation(selectedCount+1:selectedCount+remainingCount, :) = population(frontIndices(sortedIndices(1:remainingCount)), :);
            selectedCount = populationSize;
        end
        
        currentFront = currentFront + 1;
    end
end

function [Bnum] = Bnumfigure(num)
    t = dec2bin(num);
    Bnum = length(t);
    
end

% 适应值评价
function [Disfitness,Waterfitness,Rainfitness,Rcfitness,Cafitness,Dpfitness,chromosome] = fitness_evaluate(chromosome)
    global BnumArr;
    global stAllpath;
    global Dis_Network;
    global Water_Network;
    global Rain_Network;
    global shLength;
    global shCapacity;
    global expect_pop;
    global shelterIndex;
    
    %各项适应值初始化
    Disfitness = 0;
    Waterfitness = 0;
    Rainfitness = 0;
    Rcfitness = 0;%拥挤度
    Cafitness = 0;%容量
    Dpfitness = 0;% 意愿
    
    passby_nodes_all = [];
    dpArr = [];
    afLength = length(BnumArr);
    
    for n = 1:afLength
        %获取二进制,BnumArr保存了每一个二进制编码的位数
        len = BnumArr(n);
        if (n==1)
            range = 0;
        else
            range = range+ BnumArr(n-1);
        end
        bArr = chromosome(1+range:len+range);
        %二进制转十进制，num为当前选择的路径方案数
        str = num2str(bArr);
        num = bin2dec(str);
    
        %把编码中超出范围的编码随机到符合范围
        if num > length(stAllpath{n}) || num == 0
            num = ceil(rand * length(stAllpath{n}));
            chromosome(1 + range : len + range) = [zeros(1, len - length(dec2bin(num))), dec2bin(num) == '1'];
        end
        
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

function [bestindividual bestfit] = best(pop,fitvalue)
[px,py] = size(pop);
bestindividual = pop(1,:);
bestfit = fitvalue(1);
for i = 2:px
    if fitvalue(i)>bestfit
        bestindividual = pop(i,:);
        bestfit = fitvalue(i);
    end
end
end

function [newpop] = crossover(pop,pc,BnumArr)
[px,py] = size(pop);
newpop = ones(size(pop));
randTag = rand(px,1)<pc; 
crossNum = sum(randTag);
crossNum = floor(crossNum/2)*2;
part1 = pop(find(randTag==0),:);
part2 = pop(find(randTag==1),:);
%变化的交叉值
for i = 1:2:crossNum-1
    rd = randi(length(pop(1,:)));
    g1 = [part2(i,1:rd),part2(i+1,rd+1:end)];
    g2 = [part2(i+1,1:rd),part2(i,rd+1:end)];

    part2(i,:) = g1;
    part2(i+1,:) = g2;
end
newpop = [part1;part2];
end


%关于编译
%函数说明
%输入变量：pop：二进制种群，pm：变异概率
%输出变量：newpop变异以后的种群
function [newpop] = mutation(pop,pm)
[px,py] = size(pop);
newpop = pop;
randTag = rand(px*py,1)<pm;
rd = find(randTag==1);
newpop(rd) = (newpop(rd)==0);
end
