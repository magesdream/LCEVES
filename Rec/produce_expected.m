clc
clear
% 3402	1495	416	245	2206	1066	3461	7	1551	1337
expect_pop_100 = zeros(100,1);
load affected_100.mat
load shelter

for i = 1:100
    x1 = affected_100(i,1);
    y1 = affected_100(i,2);
    mindis = 99;
    mindis_index = 1;
    for j = 1:5
        x2 = shelter(j,1);
        y2 = shelter(j,2);
        dis = (x1-x2)^2+(y1-y2)^2;
        if mindis > dis
            mindis = dis;
            mindis_index = j;
        end
    end
    if rand < 1
        expect_pop_100(i) = mindis_index;
    else
        expect_pop_100(i) = randperm(5,1);
    end
end

save expect_pop_100 expect_pop_100

1