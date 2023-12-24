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
load ../Rec/expect_pop_100-0.5.mat;
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
fornum = 50000;

% �����ͼ
load ../Rec/shapes.mat
load ../Rec/G_Network
load ../Rec/RoadNodes

load ../Rec/Dis_Network
load ../Rec/Water_Network
load ../Rec/Rain_Network

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

w1 = 1;w2=0.1;w3=0.05;w4=2;w5=2;w6=1;

DisfitnessArr = zeros(fornum,1);
WaterfitnessArr = zeros(fornum,1);
RainfitnessArr = zeros(fornum,1);
RcfitnessArr = zeros(fornum,1);
CafitnessArr = zeros(fornum,1);
DpfitnessArr = zeros(fornum,1);

while 1
    chromosome = produce_expected_pop(100);
    load bestExp5000\bestindividual.mat
    chromosome = bestindividual;
    
    [Disfitness,Waterfitness,Rainfitness,Rcfitness,Cafitness,Dpfitness,chromosome] = fitness_evaluate(chromosome);
    Disfitness_nor = Disfitness*w1;
    Waterfitness_nor = Waterfitness*w2;
    Rainfitness_nor = Rainfitness*w3;
    Rcfitness_nor = Rcfitness*w4;
    Cafitness_nor = Cafitness*w5;
    Dpfitness_nor = Dpfitness*w6;

    fitness = Disfitness_nor+Waterfitness_nor+Rainfitness_nor+Rcfitness_nor+Cafitness_nor+Dpfitness_nor;
    fitvalue = afLength*150./fitness;

    expect_pop_fitvalue = [fitvalue,Disfitness_nor,Waterfitness_nor,Rainfitness_nor,Rcfitness_nor,Cafitness_nor,Dpfitness_nor];
    % save ./Compare_exepect_and_short/expect_pop_fitvalue  expect_pop_fitvalue
end

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
    Cafitness

    
    % ������Ը������    
    dp_sta = (dpArr ~= shelterIndex(expect_pop));
    Dpfitness = sum(dp_sta);
end