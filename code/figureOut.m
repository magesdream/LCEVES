clc;
clear;
load G_Network
time = [];
mgraph = [[0, 1, 12, inf, inf, inf]
              [inf, 0, 9, 3, inf, inf]
              [inf, inf, 0, inf, 5, inf]
              [inf, inf, 4, 0, 13, 15]
              [inf, inf, inf, inf, 0, 4]
              [inf, inf, inf, inf, inf, 0]];
% mgraph = ones(10);
% road = findPath(mgraph,1,10,0);
mgraph(find(mgraph>0));
test = [5,6,8,4,6,8,9,5,3,1,4,8];

test = []
length(test(:,1))







