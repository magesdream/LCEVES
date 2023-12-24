clc
clear
% 指定要保存的 Excel 文件的文件名
% % load affected_1000.mat
% % affected = affected;
% % filename = 'affected_1000.csv';
% 使用 writematrix 函数将数组写入 Excel 文件
% writematrix(affected, filename);

% 使用 writematrix 函数将数组写入文本文件
% writematrix(affected, filename, 'Delimiter', '\t');

shelter = [ 114.538633  ,30.513052;
            114.485571 ,30.440478;
            114.466536 ,30.502582;
            114.610025 ,30.444671;
            114.606289 ,30.512397;
            114.474226 ,30.468581;
            114.537797 ,30.449100;
            114.572658 ,30.462429;
            114.661520 ,30.494214;
            114.506398 ,30.553503];

save shelter shelter
filename = 'shelter.csv';
csvwrite(filename, shelter);
