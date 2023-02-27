clear;
clc;
tic

% route = C_RoutePlan(114.461777,30.428581,114.503893,30.484736); route =
C_RoutePlan(114.461777,30.428581,114.503893,30.484736);
load G_Network.mat
%输入桐乡到富阳的高速公路网络图的边权矩阵
a=[0,62,66,inf,inf,inf,inf;
      62,0,inf,25,11,inf,inf;
      66,inf,0,9,inf,inf,49;
      inf,25,9,0,11,14,inf;
      inf,11,inf,11,0,13,inf;
      inf,inf,inf,14,13,0,35;
      inf,inf,49,inf,inf,35,0;];
%   a = zeros(500);
%   a(:) = inf;
%   a(1,7) = 5;
  
%调用搜索图中任意两点间所有路径的M文件
% findPath(a, 1, 7, 0)

toc