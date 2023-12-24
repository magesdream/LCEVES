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

%种群大小 自行决定
popsize=20;

%交叉概率
pc = 0.7;
%变异概率
pm = 0.01;
% 迭代次数
fornum = 5000;

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

toc;
% % 根据index计算风险点到避难点的可能路径,重点之一！
disp("开始初步最短路径规划")
G = digraph(G_Network);
Gf = sparse(G_Network);
% plot(G, 'Layout', 'force');
stdist = zeros(afLength,shLength);
stpath = cell(afLength,shLength);
stAllpath = cell(1,afLength);

step1 = 0.0100;%控制最短路径距离
step2 = 0.0100;%控制全路径距离
bufferDis = 0.002;%缓存距离0.001

problem_id = [];

stAllpathFile = '../Rec/stAllpath100.mat';
if exist(stAllpathFile,'file')==0
    for i = 1:afLength
        for j=1:shLength
    %         问题：1.还可以进行优化，加快速率。2.如果没有可达路径怎么办？
            start = affectedIndex(i);
            des = shelterIndex(j);
            
            [path,dist] = shortestpath(G,start,des);
            path =path';
            
    %         获得最短路径经过点与最短路径值
            stdist(i,j) = dist;
            stpath{i,j} = path;
    
    
    %         记录无法到达refuge的affected points
            if (dist==inf)
                problem_id = [problem_id,i];
                continue
            end
            
            %开始寻找所有路径
            %首先是缓冲区分析
            shortCor = RoadNodes(path,:);
            max1 = max(shortCor(:,1));
            max2 = max(shortCor(:,2));
            min1 = min(shortCor(:,1));
            min2 = min(shortCor(:,2));
            corner1 = [max1+step2,max2+step2];
            corner2 = [min1-step2,min2-step2];
            NodesTag1 = (RoadNodes(:,1)<corner1(1))+(RoadNodes(:,1)>corner2(1));
            NodesTag2 = (RoadNodes(:,2)<corner1(2))+(RoadNodes(:,2)>corner2(2));
            NodesTagSta = NodesTag1+NodesTag2;
            NodesTag = find(NodesTagSta==4);
            for t = 1:length(path)
                tag = find(path(t)==NodesTag);
                NodesTag(tag) = [];
            end
            ANodesTag = path;
            for m = 1:length(path)
               for n = 1:length(NodesTag)
                   x1 = shortCor(m,1);
                   x2 = RoadNodes(NodesTag(n),1);
                   y1 = shortCor(m,2);
                   y2 = RoadNodes(NodesTag(n),2);
                   if (((x2-x1)^2+(y2-y1)^2)<bufferDis^2)
                       ANodesTag(end+1) = NodesTag(n);
                       NodesTag(n) = [];
                   end
                   if(n>=length(NodesTag))
                       break
                   end
                   
               end
            end
    
            FG = Gf(ANodesTag,ANodesTag); 
            %核心点，保证每项可选不超过5个.
            edge = 6;
            AllPath = findPath(FG,1,length(path),0,edge);
            PLength = length(AllPath(:,1));
            LLength = length(stAllpath{i});
% sortrows
            sortedPath = sortrows(AllPath,length(AllPath(1,:)));

            if PLength<edge
                for p = 1: edge-PLength
                    path_arr = sortedPath(mod(p-1,PLength)+1,:);
                    sortedPath = [sortedPath;path_arr];
                end
            end
            AllPath = sortedPath;
            
            for t = 1:edge
                tag = find(AllPath(t,:)>0);
                arr = AllPath(t,1:tag(end-1));
                Path = ANodesTag(arr);
                Path(end+1) = AllPath(t,end);
                stAllpath{i}{LLength + t} = Path;
            end
            %--------------------------------------------------------
    
        end
        disp(i+"/"+afLength)
    end
    toc;
    
    save ../Rec/stAllpath100 stAllpath
else
    load ../Rec/stAllpath100
end


% 有空路径的时候的处理方法
% cha = find(stdist(:,1)==inf);
% stdist(cha,:)=[];
% stpath(cha,:)=[];
% affected(cha,:)=[];
% afLength = afLength - length(cha);
% chromlength=afLength;


%-----------------------------------

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

exp_pop = produce_expected_pop(100);
% pop(1,:) = exp_pop;

%--------------------------------------
for i = 1:fornum
    disp(i)
    tic
    %适应值数组初始化
    objvalue = zeros(popsize,1);

    %考虑expected期望染色体
%     if i<fornum*0.8
%         number = min(i*len_exp_pop/(1.2*fornum),len_exp_pop);
%         exp_pop = produce_expected_pop(ceil(number));
%         pop(1,:) = exp_pop;
%     end

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

%     disp("计算适应度的时间")
%     toc
    fitvalue = afLength*150./objvalue;
%    适应度函数是重点 
    
    %寻找最优解
    [bestindividual,bestfit] = best(pop,fitvalue);
    
    if bestfitMid > bestfit
        pop(ceil(rand*popsize),:) = bestindividualMid;
        bestfit = bestfitMid;
        bestindividual = bestindividualMid;
    end
    bestindividualMid = bestindividual;
    bestfitMid = bestfit;
    
    %选择操作
    newpop = selection(pop,fitvalue);

    %交叉操作
    newpop = crossover(newpop,pc,BnumArr,i);
    %变异操作
    newpop = mutation(newpop,pm);

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
end
toc

fprintf('The best Y is --->>%5.2f\n',bestFitvalue);
figure('Color', 'white'); % 设置背景颜色为纯白色
plot(bestfitArr)
title("Fitness");
xlabel("Evolutionary algebra");
ylabel("适应值");

% 创建 x 轴范围 1 到 500
range = 1:fornum;

% 绘制每个数组的数据
figure('Color', 'white'); % 设置背景颜色为纯白色
hold on;
% xcor = -10;
% text(xcor,DisfitnessArr(end),num2str(DisfitnessArr(end)))
% text(xcor,WaterfitnessArr(end),num2str(WaterfitnessArr(end)))
% text(xcor,RainfitnessArr(end),num2str(RainfitnessArr(end)))
% text(xcor,RcfitnessArr(end),num2str(RcfitnessArr(end)))
% text(xcor,CafitnessArr(end),num2str(CafitnessArr(end)))
% text(xcor,DpfitnessArr(end),num2str(DpfitnessArr(end)))

colororder({'#0072BD','k'})
yyaxis right; % 右侧Y轴
plot(range, bestfitArr, 'Color', [0, 0, 0], 'DisplayName', 'Fitness');
ylabel('Total Fitness');

yyaxis left; % 左侧Y轴
plot(range, DisfitnessArr,'-', 'Color', [1, 1, 0], 'DisplayName', 'Disfitness');
plot(range, WaterfitnessArr,'-', 'Color', [0, 1, 1], 'DisplayName', 'Waterfitness');
plot(range, RainfitnessArr, '-','Color', [0, 0, 1], 'DisplayName', 'Rainfitness');
plot(range, RcfitnessArr,'-', 'Color', [1, 0, 0], 'DisplayName', 'Rcfitness');
plot(range, CafitnessArr,'-', 'Color', [0, 1, 0], 'DisplayName', 'Cafitness');
plot(range, DpfitnessArr,'-', 'Color', [1, 0, 1], 'DisplayName', 'Dpfitness');
ylabel('Six subfitness');


hold off;

% 添加标签和图例
xlabel('generation');

title(' ');
legend('show','Location', 'northeast');

save DisfitnessArr DisfitnessArr
save WaterfitnessArr WaterfitnessArr
save RainfitnessArr RainfitnessArr
save RcfitnessArr RcfitnessArr
save CafitnessArr CafitnessArr
save DpfitnessArr DpfitnessArr
save bestfitArr bestfitArr


% -----------------------------------------------
% 开始结果评价
% -----------------------------------------------

% 最优染色体各项评价
Disfitness = 0;
Waterfitness = 0;
Rainfitness = 0;
Rcfitness = 0;%拥挤度
Cafitness = 0;%容量
Dpfitness = 0;% 意愿
[Disfitness,Waterfitness,Rainfitness,Rcfitness,Cafitness,Dpfitness,chromosome] = fitness_evaluate(chromosome);

Disfitness_nor = Disfitness*w1;
Waterfitness_nor = Waterfitness*w2;
Rainfitness_nor = Rainfitness*w3;
Rcfitness_nor = Rcfitness*w4;
Cafitness_nor = Cafitness*w5;
Dpfitness_nor = Dpfitness*w6;

s_route = [];
% 获取简化版最终路径
% road_note1 = {0};
% for n = 1:afLength
%         %获取二进制
%         len = BnumArr(n);
%         if (n==1)
%             range = 0;
%         else
%             range = range+ BnumArr(n-1);
%         end
%         bArr = bestindividual(1+range:len+range);
%         %二进制转十进制
%         str = num2str(bArr);
%         num = bin2dec(str);
%         %经过点评价        
%         PointArr = stAllpath{n}{num}(1:end-1);
%         NodesX = RoadNodes(PointArr,1);
%         NodesY = RoadNodes(PointArr,2);
%         NodesXY = [NodesX,NodesY];
%         NodesXY = [affected(n,:);NodesXY];
%         road_note1{n} = NodesXY;
%         
% end

% 获取完整版最终路径
s_route = cell(afLength,1);
for m = 1:afLength
    m
    %获取二进制
    len = BnumArr(m);
    range = sum(BnumArr(1:m-1));
    bArr = bestindividual(range+1:range+len);

    %二进制转十进制
    str = num2str(bArr);
    num = bin2dec(str);
    %经过点评价        
    PointArr = stAllpath{m}{num}(1:end-1);
    
    mroute{1} = PointArr;
    %补全路径
   	froute = [];
    %初始点 
    froute(1,1) = RoadNodes(PointArr(1),1);
    froute(1,2) = RoadNodes(PointArr(1),2);

    for i = 1:length(PointArr)-1
    x1 = RoadNodes(PointArr(i),1);
    y1 = RoadNodes(PointArr(i),2);
    x2 = RoadNodes(PointArr(i+1),1);
    y2 = RoadNodes(PointArr(i+1),2);
       for j = 1:length(shapes) 
           arrx1 = shapes(j).X == x1;
           arry1 = shapes(j).Y == y1;
           arr1 = arrx1 + arry1;
           arrx2 = shapes(j).X == x2;
           arry2 = shapes(j).Y == y2;
           arr2 = arrx2 + arry2;
           %道路存在双向性
           if   sum(arr1)==2 && sum(arr2)==2 
               p1 = find(arr1==2);
               p2 = find(arr2==2);
               if p1<p2
                   for t = p1+1:p2
                       froute(end+1,1) = shapes(j).X(t);
                       froute(end,2) = shapes(j).Y(t);
                   end
               else
                   for t = p1-1:-1:p2
                       froute(end+1,1) = shapes(j).X(t);
                       froute(end,2) = shapes(j).Y(t);
                   end
               end
               break;
           end 
           if sum(arr1) == 2 && sum(arr2) == 2
               p1 = find(arr1 == 2);
               p2 = find(arr2 == 2);
               p_min = min(p1, p2);
               p_max = max(p1, p2);

               for t = p_min+1:p_max
                   froute(end+1, 1) = shapes(j).X(t);
                   froute(end, 2) = shapes(j).Y(t);
               end

               break;
           end
       end
   end
   NodesX = froute(:,1);
   NodesY = froute(:,2);
    %    NodesX = RoadNodes(Nodes,1);
    %    NodesY = RoadNodes(Nodes,2);
   NodesX = [affected(m,1);NodesX];
   NodesY = [affected(m,2);NodesY];
   finalNode = stAllpath{m}{num}(end-1);
   sh_loc = find(shelterIndex==finalNode);
   NodesX(end+1) = shelter(sh_loc,1);
   NodesY(end+1) = shelter(sh_loc,2);
   route = [NodesX,NodesY];
   s_route{m} = route;
end

save s_route s_route

final = 0;


% 以下为此算法使用的函数-----------------------------
%用于计算实数所对应的二进制编码位数
function [Bnum] = Bnumfigure(num)
%     t = 0;
%     while(1)
%         if(2^t<num&&2^(t+1)>=num)
%             Bnum = t+1;
%             break
%         end
%         t = t+1;
%     end
    t = dec2bin(num);
    Bnum = length(t);
    
end

%求最优适应度函数
%输入变量：pop:种群，fitvalue:种群适应度
%输出变量：bestindividual:最佳个体，bestfit:最佳适应度值
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

%如何选择新的个体
%输入变量：pop二进制种群，fitvalue：适应度值
%输出变量：newpop选择以后的二进制种群
function [newpop] = selection(pop,fitvalue)
%构造轮盘
[px,py] = size(pop);
maxfit = max(fitvalue);
minfit = min(fitvalue);
newfitvalue = (fitvalue-minfit)/(maxfit-minfit);
totalfit = sum(newfitvalue);
p_fitvalue = newfitvalue/totalfit;
p_fitvalue = cumsum(p_fitvalue);%概率求和排序
ms = sort(rand(px,1));%从小到大排列
fitin = 1;
newin = 1;
while newin<=px
    if(ms(newin))<p_fitvalue(fitin)
        newpop(newin,:)=pop(fitin,:);
        newin = newin+1;
    else
        fitin=fitin+1;
    end
end
end

%交叉变换
%输入变量：pop：二进制的父代种群数，pc：交叉的概率
%输出变量：newpop：交叉后的种群数
function [newpop] = crossover(pop,pc,BnumArr,genum)
global fornum;

[px,py] = size(pop);
newpop = ones(size(pop));
randTag = rand(px,1)<pc;
crossNum = sum(randTag);
crossNum = floor(crossNum/2)*2;
part1 = pop(find(randTag==0),:);
part2 = pop(find(randTag==1),:);
%变化的交叉值
for i = 1:2:crossNum-1
    rd = ceil(rand(1,length(BnumArr)).*(BnumArr-1)');
    g1 = [];
    g2 = [];
    for j = 1:length(BnumArr)
        len = BnumArr(j);
        if (j==1)
            range = 0;
        else
            range = range+ BnumArr(j-1);
        end
        if rand < genum/fornum
            left = 1+range:rd(j)+range;
            right = rd(j)+range+1:len+range;
            g1 = [g1,left];
            g2 = [g2,right];
        end
    end
    part2(i,g1) = part2(i+1,g1);
    part2(i+1,g2) = part2(i,g2);
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

% 产生一个满足number个车辆意愿的染色体；
% 只包含number车辆的染色体，其他部分需要补充
function [expected_pop] = produce_expected_pop(number)
    pop_dp = [];
    global expect_pop;
    global shelterIndex;
    global stAllpath;
    global BnumArr;
    
    for i = 1:number
        dp = expect_pop(i);
        dpindex = shelterIndex(dp);
        len_path = length(stAllpath{i});
        %在全路径中寻找，如果找到相同的就退出并记录
        while true
            num = randperm(len_path,1);
            if dpindex == stAllpath{i}{num}(end-1)
                break
            end
        end
        binaryString = dec2bin(num);
        
        % 指定二进制编码长度
        desiredLength = BnumArr(i);
        
        % 计算需要补充的零的数量
        paddingZeros = max(0, desiredLength - length(binaryString));
        
        % 在前方补充零
        paddedBinaryString = [repmat('0', 1, paddingZeros), binaryString];
        pop_dp = [pop_dp,paddedBinaryString];
    end

    remain_len = length(expect_pop)-number;

    pop = round(rand(1,sum(BnumArr(end-remain_len+1:end))));
    
    % 第一行替换为全意愿
    doubleArray = zeros(size(pop_dp));
    for i = 1:numel(pop_dp)
        doubleArray(i) = str2double(pop_dp(i));
    end

    expected_pop = [doubleArray,pop];

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