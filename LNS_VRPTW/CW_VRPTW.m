clear
clc
%% 用importdata这个函数来读取文件 
rc208=importdata('rc208.txt');
cap=1000;
%% CW法构造VRPTW初始解
[init_vc,init_TD,init_vl,violate_INTW] = init_TW(rc208,cap);
initNV=size(init_vc,1);