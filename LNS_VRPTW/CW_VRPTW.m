clear
clc
%% ��importdata�����������ȡ�ļ� 
rc208=importdata('rc208.txt');
cap=1000;
%% CW������VRPTW��ʼ��
[init_vc,init_TD,init_vl,violate_INTW] = init_TW(rc208,cap);
initNV=size(init_vc,1);