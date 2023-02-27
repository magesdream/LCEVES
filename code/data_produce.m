clc;
clear;
afnum = 800;
% affectedX = 114.442255+randn(afnum,1)*(114.644212-114.442255);
% affectedY = 30.406214+randn(afnum,1)*(30.553146-30.406214);
% affected = [affectedX,affectedY];
% save affected_1000 affected

shLength = 5;
chromlength = afnum;

expect_pop = ceil(rand(1,chromlength)*(shLength));
save expect_pop_800 expect_pop;