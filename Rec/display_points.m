clc
clear

load shapes.mat
load affected_1000.mat

% shelter = [ 114.538633  ,30.513052;
%             114.485571 ,30.440478;
%             114.466536 ,30.502582;
%             114.610025 ,30.444671;
%             114.606289 ,30.512397;
%             114.474226 ,30.468581;
%             114.537797 ,30.449100;
%             114.572658 ,30.462429;
%             114.661520 ,30.494214;
%             114.506398 ,30.553503];
load shelter

% 绘制地图
for x = 1:length(shapes)
    plot(shapes(x).X ,shapes(x).Y,'b')
    hold on;
end

affected = affected_1000;

% 将图保持在同一图中
hold on;

% 标记为圆形红色
plot(affected(:,1), affected(:,2), 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'red');

plot(shelter(:,1), shelter(:,2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'green', 'MarkerEdgeColor', 'green');

% 添加标题和标签
title('坐标点图');
xlabel('X轴');
ylabel('Y轴');

% 显示图例
legend('坐标点', '圆形红色');

% 关闭保持图的状态
hold off;

