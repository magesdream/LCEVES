clear;
clc;
tic;

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
load affected_50.mat;
% affected = affected(1:20,:);
% load affected_1000.mat;
% load affected_800.mat;

% 构建预设想去点
expect_pop = [1,2,3,4,3,2,1,2,3,4];
load expect_pop_50.mat;
% expect_pop = expect_pop(1:20);
% load expect_pop_800.mat;

afLength = length(affected);
shelter = [114.538633,  30.513052;
            114.485571 , 30.440478;
            114.466536 , 30.502582;
            114.610025  30.444671;
            114.606289  30.512397];
shLength = length(shelter);

%种群大小 自行决定
popsize=50;
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
% % 根据index计算风险点到避难点的可能路径,重点之一！
disp("开始初步最短路径规划")
G = sparse(G_Network);
stdist = zeros(afLength,shLength);
stpath = cell(afLength,shLength);
stAllpath = cell(1,afLength);

step1 = 0.0100;%控制最短路径距离
step2 = 0.0050;%控制全路径距离
bufferDis = 0.0002;%缓存距离0.001

problem_id = [];
for i = 1:afLength
    for j=1:shLength
%         问题：1.还可以进行优化，加快速率。2.如果没有可达路径怎么办？
        start = affectedIndex(i);
        des = shelterIndex(j);
        if RoadNodes(start,1)>=RoadNodes(des,1)
            max1 = RoadNodes(start,1);
            min1 = RoadNodes(des,1);
        else
            max1 = RoadNodes(des,1);
            min1 = RoadNodes(start,1);
        end
        
        if RoadNodes(start,2)>=RoadNodes(des,2)
            max2 = RoadNodes(start,2);
            min2 = RoadNodes(des,2);
        else
            max2 = RoadNodes(des,2);
            min2 =  RoadNodes(start,2);
        end
        corner1 = [max1+step1,max2+step1];
        corner2 = [min1-step1,min2-step1];
        NodesTag1 = (RoadNodes(:,1)<corner1(1))+(RoadNodes(:,1)>corner2(1));
        NodesTag2 = (RoadNodes(:,2)<corner1(2))+(RoadNodes(:,2)>corner2(2));
        NodesTagSta = NodesTag1+NodesTag2;
        NodesTag = find(NodesTagSta==4);

        if (length(NodesTag)<1000)
            startT = find(NodesTag==start);
            desT = find(NodesTag==des);
            TG = G(NodesTag,NodesTag);
            [dist, pathT] = graphshortestpath(TG,startT,desT);
            path = NodesTag(pathT);
            if (dist==inf)
                [dist, path] = graphshortestpath(G,start,des);
                path = path';
            end
        else
            [dist, path] = graphshortestpath(G,start,des);
            path =path';
        end
        
        [dist, path] = graphshortestpath(G,start,des);
        path =path';
        
        stdist(i,j) = dist;
        stpath{i,j} = path;
        if (dist==inf)
            problem_id = [problem_id,i]
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
        FG = G(ANodesTag,ANodesTag); 
        %核心点
        edge = 5;
        AllPath = findPath(FG,1,length(path),0,edge);
        PLength = length(AllPath(:,1));
        LLength = length(stAllpath{i});
        
        for t = 1:PLength
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

% 有空路径的时候的处理方法
% cha = find(stdist(:,1)==inf);
% stdist(cha,:)=[];
% stpath(cha,:)=[];
% affected(cha,:)=[];
% afLength = afLength - length(cha);
% chromlength=afLength;
toc;

%-----------------------------------二进制编码，可能出现不可行解。建议检查并重新生成
%初始种群
% 根据避难所数量而定
BnumArr = zeros(afLength,1);
for i = 1:afLength
   BnumArr(i) =  Bnumfigure(length(stAllpath{i}));
end
pop = round(rand(popsize,sum(BnumArr)));



% 愿望系数
expK = 20;

%构造避难所容量因素
shCapacity = zeros(shLength,1);
shCapacity(:) = ceil(afLength/shLength*1.5);

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
trafficJamArr = [];
capArr = [];
for i = 1:fornum
    disp(i)
    tic
    
%     先以效率为主
%     校检函数，保证种群里面没有不可行解
    
    for m = 1:afLength
        if (m==1)
            range = 0;
        else
            range = range+ BnumArr(m-1);
        end
        
        %二进制转十进制
        len = BnumArr(m);
        binary_arr = pop(:,1+range:range+len);
        binary_str_arr = num2str(binary_arr);
        value = bin2dec(binary_str_arr);
        p_max = find(value>length(stAllpath{m}));
        p_zeros = find(value==0);
        p_correction = [p_max;p_zeros];
       
        for n=1:length(p_correction)
            num = ceil(rand*length(stAllpath{m}));
            sArr = dec2bin(num);
            Arr = (sArr == '1');
            num = len-length(Arr);
            array = zeros(1,num);
            Arr = [array,Arr];
            pop(p_correction(n),1+range:range+len) = Arr;
        end
    end
%     disp("补全解的时间")
%     toc
    %计算适应度值（函数值）
%     可优化
    for m = 1:popsize
        for n = 1:afLength
            %获取二进制
            len = BnumArr(n);
            if (n==1)
                range = 0;
            else
                range = range+ BnumArr(n-1);
            end
            bArr = pop(m,1+range:len+range);
            %二进制转十进制
            str = num2str(bArr);
            num = bin2dec(str);
            %距离评价
            ds = stAllpath{n}{num}(end);
%             -----------------------------------------
            %经过点评价        
            trafficJamArr = [trafficJamArr;stAllpath{n}{num}(1:end-1)];
            
            %满足用户期望评价
            p1 = shelterIndex(expect_pop(n));
            p2 = stAllpath{n}{num}(end-1);
            capArr = [capArr,p2];
            exp = (p1~=p2)*expK;
            %总体计算
            objvalue(m) = objvalue(m) + ds+exp;
        end
%        道路拥挤量评价

        freq = hist(trafficJamArr,1:max(trafficJamArr)); 
        roadFreq = freq(freq>1);
        squeeze = sum(roadFreq.^1.2)/length(roadFreq);
        trafficJamArr = [];

        
%        避难所容量评价       
        ca = hist(capArr,shLength);
        if(length(ca)<length(shCapacity))
            zeroArr = zeros(length(shCapacity)-length(ca),1);
            ca = [ca;zeroArr];
        end
        ca = ca-shCapacity';
        caExp = ca(ca>0);
        capSta = sum(1.2.^caExp)/afLength;
        capArr = [];
        objvalue(m) = objvalue(m)/n + squeeze+capSta;
    end
%     disp("计算适应度的时间")
%     toc
    fitvalue = afLength*100./objvalue;
%    适应度函数是重点 
    
    %寻找最优解
    [bestindividual,bestfit] = best(pop,fitvalue);
    
%     if bestfitMid > bestfit
%        if rand<1
%        pop(ceil(rand*popsize),:) = bestindividualMid;
%        bestfit = bestfitMid;
%        bestindividual = bestindividualMid;
%        end
%     end
    bestindividualMid = bestindividual;
    bestfitMid = bestfit;
    
    %选择操作
    newpop = selection(pop,fitvalue);
    %交叉操作
    newpop = crossover(newpop,pc,BnumArr);
    %变异操作
    newpop = mutation(newpop,pm);
    %更新种群
    pop = newpop;
    
    
       
    if bestfit > bestFitvalue
        bestFitvalue = bestfit;
        bestCode = bestindividual;
    end
    
    fprintf('The Y is --->>%5.2f\n',bestfit);
    bestfitArr(end+1) = bestfit;
end
fprintf('The best Y is --->>%5.2f\n',bestFitvalue);
figure;
plot(bestfitArr)
title("适应值随进化代数的变化");
xlabel("进化代数");
ylabel("适应值");

trafficJamArr = [];
capArr = [];
% 结果评价
ds_eval = [];
exp_eval = [];
jam_eval = [];
cap_eval = [];
for n = 1:afLength
    %获取二进制
    len = BnumArr(n);
    if (n==1)
        range = 0;
    else
        range = range+ BnumArr(n-1);
    end
    bArr = bestCode(1+range:len+range);
    %二进制转十进制
    str = num2str(bArr);
    num = bin2dec(str);
    %距离评价
    ds = stAllpath{n}{num}(end);
    
    %经过点评价        
    trafficJamArr = [trafficJamArr;stAllpath{n}{num}(1:end-1)];
    
    %满足用户期望评价
    p1 = shelterIndex(expect_pop(n));
    p2 = stAllpath{n}{num}(end-1);
    capArr = [capArr,p2];
    exp = (p1~=p2);
    %总体计算
    ds_eval = [ds_eval,ds];
    exp_eval = [exp_eval,exp];
end
%        道路拥挤量评价
freq = hist(trafficJamArr,1:max(trafficJamArr)); 
roadFreq = freq(freq>1);
squeeze = sum(roadFreq.^1.2)/length(roadFreq);
jam_eval = roadFreq;


%        避难所容量评价       
ca = hist(capArr,shLength);
if(length(ca)<length(shCapacity))
    zeroArr = zeros(length(shCapacity)-length(ca),1);
    ca = [ca;zeroArr];
end
ca_ex = ca-shCapacity';
caExp = ca_ex(ca_ex>0);
cap_eval = caExp;


ds_eval_avg = sum(ds_eval)/length(ds_eval);
exp_eval_f = find(exp_eval==0);

%全按照意愿评价
ds_eval2 = [];
for n = 1:afLength
    num =expect_pop(n);
    %距离评价
    ds = stAllpath{n}{num}(end);
    %总体计算
    ds_eval2 = [ds_eval2,ds];
end
ds_eval_avg2 = sum(ds_eval2)/length(ds_eval2);

%最短路径评价
for n = 1:afLength
   minvalue = min(stdist(n,:));
   num = find(stdist(n,:)==minvalue);
    
    %经过点评价        
    trafficJamArr = [trafficJamArr;stpath{n,num}];
end
%        道路拥挤量评价
freq2 = hist(trafficJamArr,1:max(trafficJamArr)); 
roadFreq2 = freq2(freq2>1);
squeeze2 = sum(roadFreq2.^1.2)/length(roadFreq2);


s_route = [];
road_note2 = {0};
for n = 1:afLength
        %获取二进制
        len = BnumArr(n);
        if (n==1)
            range = 0;
        else
            range = range+ BnumArr(n-1);
        end
        bArr = bestindividual(1+range:len+range);
        %二进制转十进制
        str = num2str(bArr);
        num = bin2dec(str);
        %经过点评价        
        PointArr = stAllpath{n}{num}(1:end-1);
        NodesX = RoadNodes(PointArr,1);
        NodesY = RoadNodes(PointArr,2);
        NodesXY = [NodesX,NodesY];
        NodesXY = [affected(n,:);NodesXY];
        road_note2{n} = NodesXY;
        
end
save road_note2 road_note2

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
        left = 1+range:rd(j)+range;
        right = rd(j)+range+1:len+range;
        g1 = [g1,left];
        g2 = [g2,right];
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

% 去往某点的路径多，那么它被选中的概率大。