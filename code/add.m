clc;
clear;
a = [1;2;3];
a(end+1) = 4;
a = [5;a];
i = 5;
t = 6;
rand

tic
A = [1,2,5,6,6,8,9,20];
y = hist(A,1:20)
for x=1:500
    for y = 1:100 
%         t = tabulate(A);
    end
%     toc
%     disp(x)
end
toc
    