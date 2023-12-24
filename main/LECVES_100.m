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
% ����Ԥ����ȥ��
load ../Rec/expect_pop_100.mat;
expect_pop = expect_pop_100;

afLength = length(affected);
load ../Rec/shelter.mat;
shLength = length(shelter);

%��Ⱥ��С ���о���
popsize=20;

%�������
pc = 0.7;
%�������
pm = 0.01;
% ��������
fornum = 5000;

% �����ͼ
load ../Rec/shapes.mat
load ../Rec/G_Network
load ../Rec/RoadNodes

load ../Rec/Dis_Network
load ../Rec/Water_Network
load ../Rec/Rain_Network

% ���Ƶ�ͼ
% for x = 1:length(shapes)
% plot(shapes(x).X ,shapes(x).Y,'b')
% hold on;
% end

toc;
% �ȼ����������·��ֵ

% % ������յ�ͱ��ѵ��index
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
% % ����index������յ㵽���ѵ�Ŀ���·��,�ص�֮һ��
disp("��ʼ�������·���滮")
G = digraph(G_Network);
Gf = sparse(G_Network);
% plot(G, 'Layout', 'force');
stdist = zeros(afLength,shLength);
stpath = cell(afLength,shLength);
stAllpath = cell(1,afLength);

step1 = 0.0100;%�������·������
step2 = 0.0100;%����ȫ·������
bufferDis = 0.002;%�������0.001

problem_id = [];

stAllpathFile = '../Rec/stAllpath100.mat';
if exist(stAllpathFile,'file')==0
    for i = 1:afLength
        for j=1:shLength
    %         ���⣺1.�����Խ����Ż����ӿ����ʡ�2.���û�пɴ�·����ô�죿
            start = affectedIndex(i);
            des = shelterIndex(j);
            
            [path,dist] = shortestpath(G,start,des);
            path =path';
            
    %         ������·�������������·��ֵ
            stdist(i,j) = dist;
            stpath{i,j} = path;
    
    
    %         ��¼�޷�����refuge��affected points
            if (dist==inf)
                problem_id = [problem_id,i];
                continue
            end
            
            %��ʼѰ������·��
            %�����ǻ���������
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
            %���ĵ㣬��֤ÿ���ѡ������5��.
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


% �п�·����ʱ��Ĵ�������
% cha = find(stdist(:,1)==inf);
% stdist(cha,:)=[];
% stpath(cha,:)=[];
% affected(cha,:)=[];
% afLength = afLength - length(cha);
% chromlength=afLength;


%-----------------------------------

%��ʼ��Ⱥ
% ���ݱ�������������
BnumArr = zeros(afLength,1);
len_exp_pop = length(expect_pop);
for i = 1:afLength
%     ����ÿ�����ֳ����Ŀ�ѡ������������ת��Ϊ��Ӧ�Ķ����Ƴ���
   BnumArr(i) =  Bnumfigure(length(stAllpath{i}));
end
% �������Ƴ�����ͣ�������Ⱥ
pop = round(rand(popsize,sum(BnumArr)));

%�����������������
shCapacity = zeros(shLength,1);
shCapacity(:) = ceil(afLength/shLength*1.1);

bestFitvalue = 0;
bestCode = [];
bestindividualMid = [];
bestfitMid = 0;
disp("��ͼʱ��:")
toc;

bestfitArr  = [];

tic
disp("�����㷨��ʼ")
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
    %��Ӧֵ�����ʼ��
    objvalue = zeros(popsize,1);

    %����expected����Ⱦɫ��
%     if i<fornum*0.8
%         number = min(i*len_exp_pop/(1.2*fornum),len_exp_pop);
%         exp_pop = produce_expected_pop(ceil(number));
%         pop(1,:) = exp_pop;
%     end

    %������Ӧֵ��ʼ��
    Disfitness = 0;
    Waterfitness = 0;
    Rainfitness = 0;
    Rcfitness = 0;%ӵ����
    Cafitness = 0;%����
    Dpfitness = 0;% ��Ը
    for m = 1:popsize
        chromosome = pop(m,:);
        % ������Ӧ��ֵ������ֵ��
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

%     disp("������Ӧ�ȵ�ʱ��")
%     toc
    fitvalue = afLength*150./objvalue;
%    ��Ӧ�Ⱥ������ص� 
    
    %Ѱ�����Ž�
    [bestindividual,bestfit] = best(pop,fitvalue);
    
    if bestfitMid > bestfit
        pop(ceil(rand*popsize),:) = bestindividualMid;
        bestfit = bestfitMid;
        bestindividual = bestindividualMid;
    end
    bestindividualMid = bestindividual;
    bestfitMid = bestfit;
    
    %ѡ�����
    newpop = selection(pop,fitvalue);

    %�������
    newpop = crossover(newpop,pc,BnumArr,i);
    %�������
    newpop = mutation(newpop,pm);

    %������Ⱥ
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
figure('Color', 'white'); % ���ñ�����ɫΪ����ɫ
plot(bestfitArr)
title("Fitness");
xlabel("Evolutionary algebra");
ylabel("��Ӧֵ");

% ���� x �᷶Χ 1 �� 500
range = 1:fornum;

% ����ÿ�����������
figure('Color', 'white'); % ���ñ�����ɫΪ����ɫ
hold on;
% xcor = -10;
% text(xcor,DisfitnessArr(end),num2str(DisfitnessArr(end)))
% text(xcor,WaterfitnessArr(end),num2str(WaterfitnessArr(end)))
% text(xcor,RainfitnessArr(end),num2str(RainfitnessArr(end)))
% text(xcor,RcfitnessArr(end),num2str(RcfitnessArr(end)))
% text(xcor,CafitnessArr(end),num2str(CafitnessArr(end)))
% text(xcor,DpfitnessArr(end),num2str(DpfitnessArr(end)))

colororder({'#0072BD','k'})
yyaxis right; % �Ҳ�Y��
plot(range, bestfitArr, 'Color', [0, 0, 0], 'DisplayName', 'Fitness');
ylabel('Total Fitness');

yyaxis left; % ���Y��
plot(range, DisfitnessArr,'-', 'Color', [1, 1, 0], 'DisplayName', 'Disfitness');
plot(range, WaterfitnessArr,'-', 'Color', [0, 1, 1], 'DisplayName', 'Waterfitness');
plot(range, RainfitnessArr, '-','Color', [0, 0, 1], 'DisplayName', 'Rainfitness');
plot(range, RcfitnessArr,'-', 'Color', [1, 0, 0], 'DisplayName', 'Rcfitness');
plot(range, CafitnessArr,'-', 'Color', [0, 1, 0], 'DisplayName', 'Cafitness');
plot(range, DpfitnessArr,'-', 'Color', [1, 0, 1], 'DisplayName', 'Dpfitness');
ylabel('Six subfitness');


hold off;

% ���ӱ�ǩ��ͼ��
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
% ��ʼ�������
% -----------------------------------------------

% ����Ⱦɫ���������
Disfitness = 0;
Waterfitness = 0;
Rainfitness = 0;
Rcfitness = 0;%ӵ����
Cafitness = 0;%����
Dpfitness = 0;% ��Ը
[Disfitness,Waterfitness,Rainfitness,Rcfitness,Cafitness,Dpfitness,chromosome] = fitness_evaluate(chromosome);

Disfitness_nor = Disfitness*w1;
Waterfitness_nor = Waterfitness*w2;
Rainfitness_nor = Rainfitness*w3;
Rcfitness_nor = Rcfitness*w4;
Cafitness_nor = Cafitness*w5;
Dpfitness_nor = Dpfitness*w6;

s_route = [];
% ��ȡ�򻯰�����·��
% road_note1 = {0};
% for n = 1:afLength
%         %��ȡ������
%         len = BnumArr(n);
%         if (n==1)
%             range = 0;
%         else
%             range = range+ BnumArr(n-1);
%         end
%         bArr = bestindividual(1+range:len+range);
%         %������תʮ����
%         str = num2str(bArr);
%         num = bin2dec(str);
%         %����������        
%         PointArr = stAllpath{n}{num}(1:end-1);
%         NodesX = RoadNodes(PointArr,1);
%         NodesY = RoadNodes(PointArr,2);
%         NodesXY = [NodesX,NodesY];
%         NodesXY = [affected(n,:);NodesXY];
%         road_note1{n} = NodesXY;
%         
% end

% ��ȡ����������·��
s_route = cell(afLength,1);
for m = 1:afLength
    m
    %��ȡ������
    len = BnumArr(m);
    range = sum(BnumArr(1:m-1));
    bArr = bestindividual(range+1:range+len);

    %������תʮ����
    str = num2str(bArr);
    num = bin2dec(str);
    %����������        
    PointArr = stAllpath{m}{num}(1:end-1);
    
    mroute{1} = PointArr;
    %��ȫ·��
   	froute = [];
    %��ʼ�� 
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
           %��·����˫����
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


% ����Ϊ���㷨ʹ�õĺ���-----------------------------
%���ڼ���ʵ������Ӧ�Ķ����Ʊ���λ��
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

%��������Ӧ�Ⱥ���
%���������pop:��Ⱥ��fitvalue:��Ⱥ��Ӧ��
%���������bestindividual:��Ѹ��壬bestfit:�����Ӧ��ֵ
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

%���ѡ���µĸ���
%���������pop��������Ⱥ��fitvalue����Ӧ��ֵ
%���������newpopѡ���Ժ�Ķ�������Ⱥ
function [newpop] = selection(pop,fitvalue)
%��������
[px,py] = size(pop);
maxfit = max(fitvalue);
minfit = min(fitvalue);
newfitvalue = (fitvalue-minfit)/(maxfit-minfit);
totalfit = sum(newfitvalue);
p_fitvalue = newfitvalue/totalfit;
p_fitvalue = cumsum(p_fitvalue);%�����������
ms = sort(rand(px,1));%��С��������
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

%����任
%���������pop�������Ƶĸ�����Ⱥ����pc������ĸ���
%���������newpop����������Ⱥ��
function [newpop] = crossover(pop,pc,BnumArr,genum)
global fornum;

[px,py] = size(pop);
newpop = ones(size(pop));
randTag = rand(px,1)<pc;
crossNum = sum(randTag);
crossNum = floor(crossNum/2)*2;
part1 = pop(find(randTag==0),:);
part2 = pop(find(randTag==1),:);
%�仯�Ľ���ֵ
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


%���ڱ���
%����˵��
%���������pop����������Ⱥ��pm���������
%���������newpop�����Ժ����Ⱥ
function [newpop] = mutation(pop,pm)
[px,py] = size(pop);
newpop = pop;
randTag = rand(px*py,1)<pm;
rd = find(randTag==1);
newpop(rd) = (newpop(rd)==0);
end

% ����һ������number��������Ը��Ⱦɫ�壻
% ֻ����number������Ⱦɫ�壬����������Ҫ����
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
        %��ȫ·����Ѱ�ң�����ҵ���ͬ�ľ��˳�����¼
        while true
            num = randperm(len_path,1);
            if dpindex == stAllpath{i}{num}(end-1)
                break
            end
        end
        binaryString = dec2bin(num);
        
        % ָ�������Ʊ��볤��
        desiredLength = BnumArr(i);
        
        % ������Ҫ������������
        paddingZeros = max(0, desiredLength - length(binaryString));
        
        % ��ǰ��������
        paddedBinaryString = [repmat('0', 1, paddingZeros), binaryString];
        pop_dp = [pop_dp,paddedBinaryString];
    end

    remain_len = length(expect_pop)-number;

    pop = round(rand(1,sum(BnumArr(end-remain_len+1:end))));
    
    % ��һ���滻Ϊȫ��Ը
    doubleArray = zeros(size(pop_dp));
    for i = 1:numel(pop_dp)
        doubleArray(i) = str2double(pop_dp(i));
    end

    expected_pop = [doubleArray,pop];

end

% ��Ӧֵ����
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
    
    %������Ӧֵ��ʼ��
    Disfitness = 0;
    Waterfitness = 0;
    Rainfitness = 0;
    Rcfitness = 0;%ӵ����
    Cafitness = 0;%����
    Dpfitness = 0;% ��Ը
    
    passby_nodes_all = [];
    dpArr = [];
    afLength = length(BnumArr);
    
    for n = 1:afLength
        %��ȡ������,BnumArr������ÿһ�������Ʊ����λ��
        len = BnumArr(n);
        if (n==1)
            range = 0;
        else
            range = range+ BnumArr(n-1);
        end
        bArr = chromosome(1+range:len+range);
        %������תʮ���ƣ�numΪ��ǰѡ���·��������
        str = num2str(bArr);
        num = bin2dec(str);
    
        %�ѱ����г�����Χ�ı�����������Ϸ�Χ
        if num > length(stAllpath{n}) || num == 0
            num = ceil(rand * length(stAllpath{n}));
            chromosome(1 + range : len + range) = [zeros(1, len - length(dec2bin(num))), dec2bin(num) == '1'];
        end
        
        %��¼������ǰ·�������Ľڵ�
        passby_nodes = stAllpath{n}{num}(1:end-1);
    
        %���㾭���ڵ�Ŀռ������Ӧֵ
        %���㾭���ڵ����ˮ��Ӧֵ
        %���㾭���ڵ�Ľ�����Ӧֵ
        for t = 1:2:length(passby_nodes)-1
            Disfitness = Disfitness + Dis_Network(passby_nodes(t), passby_nodes(t+1));
            Waterfitness = Waterfitness + Water_Network(passby_nodes(t), passby_nodes(t+1));
            Rainfitness = Rainfitness + Rain_Network(passby_nodes(t), passby_nodes(t+1));
        end
        
        %��¼����һ����������Щ�ڵ�     
        passby_nodes_all = [passby_nodes_all;passby_nodes];
        
        %��¼���е����ֳ�����ȥ���ĸ�������
        dpReal = stAllpath{n}{num}(end-1);
        dpArr = [dpArr,dpReal];
    end
    
    %��·ӵ����Rc����
    freq = histcounts(passby_nodes_all,max(passby_nodes_all)); 
    roadFreq = freq(freq>10);
    roadFreq_sum = sum(roadFreq);
    len_roadFreq = length(roadFreq);
    %����ӵ����Ӧֵ
    Rcfitness = roadFreq_sum/50;
    
    %��������������       
    ca = histcounts(dpArr,max(dpArr));
    ca = ca(ca>0);
    if length(ca)<shLength
        ca = ones(shLength,1)*999;
    end
    ca_diff = ca-shCapacity';
    caExp = ca_diff(ca_diff>0);
%     Cafitness = sum(1.2.^caExp);
    Cafitness = min(exp(sum(caExp))-1,100);
    
    % ������Ը������    
    dp_sta = (dpArr ~= shelterIndex(expect_pop));
    Dpfitness = sum(dp_sta);
end