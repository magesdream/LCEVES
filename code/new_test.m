% graphshortestpath 
% graphallshortestpaths
% graphshortestpath 
clc;
clear;
ssss = cell(1,5);
ssss{1}=[1,2];
load G_Network.mat
a=[0,62,66,inf,inf,inf,inf;
      62,0,inf,25,11,inf,inf;
      66,inf,0,9,inf,inf,49;
      inf,25,9,0,11,14,inf;
      inf,11,inf,11,0,13,inf;
      inf,inf,inf,14,13,0,35;
      inf,inf,49,inf,inf,35,0;];

G = sparse(G_Network);
tic;
[dist, path, pred] = graphshortestpath(G,1,7);
toc;

