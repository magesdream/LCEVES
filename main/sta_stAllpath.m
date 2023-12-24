clc
clear
load ../Rec/stAllpath100.mat

num = 69;
arr = [];
len1 = length(stAllpath{num});
for i = 1:len1
    arr = [arr;stAllpath{num}{i}(end-1);]
end

% 创建一个新的图形窗口
figure;


% 循环绘制柱状图

bar(arr);

% 设置轴标签
xlabel('Index');
ylabel('Fitness');
% 设置标题
title('Fitness Comparison');
% 关闭 hold
hold off;



