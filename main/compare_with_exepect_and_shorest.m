clc
clear
load ./Compare_exepect_and_short/expect_pop_fitvalue.mat
load ./Compare_exepect_and_short/best_fitvalue.mat
load ./Compare_exepect_and_short/shorestpath_fitvalue.mat

% 创建示例数据
x = [1 2 3];        % x 坐标
y1 = best_fitvalue;    % 左边 y 轴数据
y2 = expect_pop_fitvalue;       % 右边 y 轴数据1
y3 = shorestpath_fitvalue;      % 右边 y 轴数据2

% 创建图像
figure('Color', 'white'); % 设置背景颜色为纯白色


% 绘制柱状图
b = bar(x, [y1; y2; y3]);
b(1).FaceColor = [0 0 0];
b(2).FaceColor = [1 1 0];
b(3).FaceColor = [0 1 1];
b(4).FaceColor = [0 0 1];
b(5).FaceColor = [1 0 0];
b(6).FaceColor = [0 1 0];
b(7).FaceColor = [1 0 1];
hold on;

line([0.75 3.5], [best_fitvalue(2) best_fitvalue(2)], 'Color', 'k', 'LineStyle', ':');
line([0.85 3.5], [best_fitvalue(3) best_fitvalue(3)], 'Color', 'k', 'LineStyle', ':');
line([0.95 3.5], [best_fitvalue(4) best_fitvalue(4)], 'Color', 'k', 'LineStyle', ':');
line([1.1 3.5], [best_fitvalue(5) best_fitvalue(5)], 'Color', 'k', 'LineStyle', ':');
line([1.2 3.5], [best_fitvalue(6) best_fitvalue(6)], 'Color', 'k', 'LineStyle', ':');
line([1.35 3.5], [best_fitvalue(7) best_fitvalue(7)], 'Color', 'k', 'LineStyle', ':');


hold off

% 设置左边 y 轴属性
ylabel('Fitness');
yyaxis left;
ylim([0 max(y1)]);

% 设置右边 y 轴属性
yyaxis right;
ylabel('Six subfitness');
ylim([0 max([y2 y3])]);

% 创建右边 y 轴的刻度标签
yticks_right = linspace(0, max([y2 y3]), 5);
yticklabels_right = string(yticks_right);

% 创建左边 y 轴的刻度标签
yticks_left = linspace(0, max(y1), 5);
yticklabels_left = string(yticks_left);

% 设置左边 y 轴和右边 y 轴的刻度标签
ax = gca;
ax.YAxis(1).TickValues = yticks_left;
ax.YAxis(1).TickLabels = yticklabels_left;
ax.YAxis(2).TickValues = yticks_right;
ax.YAxis(2).TickLabels = yticklabels_right;

% 设置 x 轴标签和标题
xlabel('');
xticklabels({'LECVES', 'DpPriority', 'DisPriority'});

title('Evaluation of different scheduling schemes');

% 添加图例
legend('fitness', 'Disfitness','Waterfitness','Rainfitness','Rcfitness','Cafitness','Dpfitness');

% 绘制容量图==============================
best_refuge = [15,22,22,19,22];
shortest_refuge = [13,20,34,18,15];
expect_refuge = [12,22,21,25,20];

% 创建图像
figure('Color', 'white'); % 设置背景颜色为纯白色

% 创建示例数据
x = [1 2 3];        % x 坐标
y1 = best_refuge;    % 左边 y 轴数据
y2 = expect_refuge;       % 右边 y 轴数据1
y3 = shortest_refuge;      % 右边 y 轴数据2

% 绘制柱状图
b = bar(x, [y1; y2; y3]);
hold on;
line([0.5 3.5], [22 22], 'Color', 'k', 'LineStyle', ':');
text(0.04,22,'Ceiling: 22')
% 添加图例
legend('Refuge1', 'Refuge2','Refuge3','Refuge4','Refuge5',Location='northwest');
% 设置 x 轴标签和标题
xlabel('');
xticklabels({'LECVES', 'DpPriority', 'DisPriority'});

title('Number of affected vehicles at different refuges');
% ylabel('Numbers');



