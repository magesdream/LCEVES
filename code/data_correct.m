clc;
clear;
load affected_1000
load problem_id
t=unique(problem_id);
affected(t,:) = [];

affected(1:19,:)=[];
save affected_800 affected;

