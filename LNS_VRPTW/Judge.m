clear
clc
load op_c101.mat
%% 用importdata这个函数来读取文件
c101=importdata('c101.txt');

E=c101(1,5);                                                %配送中心时间窗开始时间
L=c101(1,6);                                                %配送中心时间窗结束时间
cap=200;                                                    %车辆负荷
vertexs=c101(:,2:3);                                        %所有点的坐标x和y
customer=vertexs(2:end,:);                                  %顾客坐标
cusnum=size(customer,1);                                    %顾客数
vecnum=cusnum;                                              %车辆数
demands=c101(2:end,4);                                      %需求量
a=c101(2:end,5);                                            %顾客时间窗开始时间[a[i],b[i]]
b=c101(2:end,6);                                            %顾客时间窗结束时间[a[i],b[i]]
s=c101(2:end,7);                                            %客户点的服务时间
h=pdist(vertexs);
dist=squareform(h);                                         %距离矩阵，满足三角关系，暂用距离表示花费c[i][j]=dist[i][j]
% op_fvc=final_vehicles_customer;

%intersect(a,b)，可以得到a,b两个矩阵中相同的元素


NV=size(op_fvc,1);
route=[];
for i=1:NV
    route=[route op_fvc{i}];
end
sr=sort(route);
LEN=length(sr);
%% 寻找丢失的元素
INIT=1:100;
%setxor(a,b)可以得到a,b两个矩阵不相同的元素，也叫不在交集中的元素
DEL=setxor(sr,INIT);

