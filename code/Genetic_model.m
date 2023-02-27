clear;
clc;
tic;
% 114.442255  30.406214
% 114.644212  30.553146

affected = [114.500324 , 30.489971;
            114.485809 , 30.460704;
            114.576229 , 30.479501;
            114.538395 , 30.457610;
            114.559818 , 30.485360;
            114.613059 , 30.460078;
            114.559295  30.502880;
            114.530146  30.488008;
            114.530146  30.473434;
            114.480773  30.423763 ];
% afnum = 100;
% affectedX = 114.442255+rand(afnum,1)*(114.644212-114.442255);
% affectedY = 30.406214+rand(afnum,1)*(30.553146-30.406214);
% affected = [affectedX,affectedY];

afLength = length(affected);
shelter = [114.538633,  30.513052;
            114.485571 , 30.440478;
            114.466536 , 30.502582;
            114.610025  30.444671;
            114.606289  30.512397];
shLength = length(shelter);

%种群大小 自行决定
popsize=100;
%实体编码长度 根据避难所决定
chromlength=afLength;
%交叉概率
pc = 0.7;
%变异概率
pm = 0.01;
% 迭代次数
fornum = 500;

% 读入地图
load shapes.mat
load G_Network
load RoadNodes

% 绘制地图
for x = 1:length(shapes)
plot(shapes(x).X ,shapes(x).Y,'b')
hold on;
end

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
% % 根据index计算风险点到避难点的最短路径
disp("开始初步最短路径规划")
G = sparse(G_Network);
stdist = zeros(afLength,shLength);
stpath = cell(afLength,shLength);


for i = 1:afLength
    for j=1:shLength
        start = affectedIndex(i);
        des = shelterIndex(j);
        [dist, path, pred] = graphshortestpath(G,start,des);
        stdist(i,j) = dist;
        stpath{i,j} = path;
    end
    disp(i+"/"+afLength)
end
% 有空路径的时候的处理方法
cha = find(stdist(:,1)==inf);
stdist(cha,:)=[];
stpath(cha,:)=[];
affected(cha,:)=[];
afLength = afLength - length(cha);
chromlength=afLength;

toc;

%初始种群
% 根据避难所数量而定
pop = round(rand(popsize,chromlength)*(shLength-1))+1;
initp = [];
for i = 1:afLength
    f = stdist(i,:);
    [m,n] = min(f);
    initp(i) = n;
end
pop(1,:) = initp;
% 构建预设想去点
expect_pop = [1,2,3,4,3,2,1,2,3,4];
% expect_pop = round(rand(1,chromlength)*(shLength-1))+1;
pop(2,:) = expect_pop;
% 愿望系数
expK = 10;

%构造避难所容量因素
shCapacity = zeros(shLength,1);
shCapacity(:) = ceil(afLength/shLength);

bestFitvalue = 0;
bestCode = [];
bestindividualMid = [];
bestfitMid = 0;
disp("画图时间:")
toc;

bestfitArr  = [];
passbys = [];
disp("遗传算法开始")
%--------------------------------------
objvalue = zeros(popsize,1);
for i = 1:fornum
    %计算适应度值（函数值）
%     可优化
    for m = 1:popsize
        for n = 1:afLength
           ds = stdist(n,pop(m,n)); 
           passbys = [passbys ,stpath{n,pop(m,n)}]; 
           exp = (expect_pop(n)~=pop(m,n))*expK;
           objvalue(m) = objvalue(m) + ds+exp;
        end

%        道路拥挤量评价
%         squeezeSta  = tabulate(passbys);
        freq = hist(passbys,1:max(passbys));
        
        roadFreq = freq(freq>1);
        squeeze = sum(roadFreq.^2)/length(roadFreq);
        passbys = [];

        

%        避难所容量评价       
        ca = hist(pop(m,:),shLength);
        if(length(ca)<length(shCapacity))
            zeroArr = zeros(length(shCapacity)-length(ca),1);
            ca = [ca;zeroArr];
        end
        ca = ca-shCapacity;
        caExp = ca(ca>0);
        capSta = sum(1.2.^caExp);

             
        objvalue(m) = objvalue(m)/n + squeeze+capSta;
        
    end
    
    fitvalue = afLength*100./objvalue;
%    适应度函数是重点 

    %寻找最优解
    [bestindividual,bestfit] = best(pop,fitvalue);
    
    if bestfitMid > bestfit
       if rand<1
       pop(ceil(rand*100),:) = bestindividualMid;
       bestfit = bestfitMid;
       bestindividual = bestindividualMid;
       end
    end
    bestindividualMid = bestindividual;
    bestfitMid = bestfit;
    
    %选择操作
    newpop = selection(pop,fitvalue);
    %交叉操作
    newpop = crossover(newpop,pc,i,fornum);
    %变异操作
    newpop = mutation(newpop,pm,shLength);
    %更新种群
    pop = newpop;
    
    [bestindividual2,bestfit2] = best(pop,fitvalue);
       
    if bestfit > bestFitvalue
        bestFitvalue = bestfit;
        bestCode = bestindividual;
    end
    
    
    fprintf('The Y is --->>%5.2f\n',bestfit);
    bestfitArr(end+1) = bestfit;
end
toc;
fprintf('The best Y is --->>%5.2f\n',bestFitvalue);


% 绘制散点，受灾点和避难所点
DrawX1 = affected(:,1);
DrawY1 = affected(:,2);

DrawX2 = shelter(:,1);
DrawY2 = shelter(:,2);
hold on;
pointsize = 10;
scatter(DrawX1,DrawY1,pointsize, "r");
hold on;
scatter(DrawX2,DrawY2,pointsize*5, "g");

% 绘制受灾点到避难点的路径
bestFitvalue
bestindividual
s_route = cell(afLength,1);
for m = 1:afLength
   NodesCell = stpath(m,bestindividual(m));
   Nodes = NodesCell{1};
   mroute = NodesCell;
%    补全路径
   froute = [];
   froute(1,1) = RoadNodes(mroute{1}(1),1);
   froute(1,2) = RoadNodes(mroute{1}(1),2);
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
   NodesX = froute(:,1);
   NodesY = froute(:,2);
%    NodesX = RoadNodes(Nodes,1);
%    NodesY = RoadNodes(Nodes,2);
   NodesX = [affected(m,1);NodesX];
   NodesY = [affected(m,2);NodesY];
   NodesX(end+1) = shelter(bestindividual(m),1);
   NodesY(end+1) = shelter(bestindividual(m),2);
   
   plot(NodesX ,NodesY,'k-.','LineWidth',2)
   
   route = [NodesX,NodesY];
   s_route{m} = route;
   hold on;
end
toc;
figure;
plot(bestfitArr)


%如何选择新的个体
%输入变量：pop二进制种群，fitvalue：适应度值
%输出变量：newpop选择以后的二进制种群
function [newpop] = selection(pop,fitvalue)
%构造轮盘
[px,py] = size(pop);
totalfit = sum(fitvalue);
p_fitvalue = fitvalue/totalfit;
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
function [newpop] = crossover(pop,pc,turn,fornum)
[px,py] = size(pop);
newpop = ones(size(pop));
randTag = rand(100,1)<pc; 
crossNum = sum(randTag);
crossNum = floor(crossNum/2)*2;
part1 = pop(find(randTag==0),:);
part2 = pop(find(randTag==1),:);
%变化的交叉值
Num = turn*(2-py)/(fornum-1)+(fornum*(py-1)-1)/(fornum-1);
for i = 1:2:crossNum-1
    rd = randperm(py-1);
    if rand<0.6
        crossRange = round(Num);    
    else
        crossRange = rd(1);
    end
    cross = rd(1:crossRange);
    g1 = part2(i,:);
    g2 = part2(i+1,:);
    
    gt = g2(cross);
    g2(cross) = g1(cross);
    g1(cross) =  gt;
    
    part2(i,:) = g1;
    part2(i+1,:) = g2;
end
newpop = [part1;part2];
end


%关于编译
%函数说明
%输入变量：pop：二进制种群，pm：变异概率
%输出变量：newpop变异以后的种群
function [newpop] = mutation(pop,pm,shLength)
[px,py] = size(pop);
newpop = pop;
randTag = rand(px*py,1)<pm;
rd = find(randTag==1);
rdArr = round(rand(length(rd),1)*(shLength-1))+1;
newpop(rd) = rdArr;
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

function fullRoute = fullRouteLook(mroute,shapes)

end