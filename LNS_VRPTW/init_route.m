%% 初始化各个车辆配送路线，每个安装场地由一辆车配送
function [ vehicles_customer ] = init_route( vehicles_customer )
vecnum=size(vehicles_customer,1);
for i=1:vecnum
    vehicles_customer{i}=i;                                     %初始一辆车只去一个顾客那里
end
end

