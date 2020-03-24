clear
clc
%% 用importdata这个函数来读取文件 
r101=importdata('vrp20.txt');
r101 = reshape(r101,1280,9,21);
r101 = r101(1,:,:);
r101=reshape(r101,9,21);
r101 =r101'
cap=30;
D=3;                                                       %Remove过程中的随机元素
toRemove=3;                                                %将要移出顾客的数量
%% CW法构造VRPTW初始解
[init_vc,init_TD,init_vl,violate_INTW] = init_TW(r101,cap);
init_vc
%% LNS优化初始解
[op_fvc,best_TD,op_vl,violate_OPTW,DEL] = LNS(init_vc,r101,cap,toRemove,D);
bestNV=size(op_fvc,1);