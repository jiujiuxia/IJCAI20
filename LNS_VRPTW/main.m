clear
clc
%% ��importdata�����������ȡ�ļ� 
r101=importdata('vrp20.txt');
r101 = reshape(r101,1280,9,21);
r101 = r101(1,:,:);
r101=reshape(r101,9,21);
r101 =r101'
cap=30;
D=3;                                                       %Remove�����е����Ԫ��
toRemove=3;                                                %��Ҫ�Ƴ��˿͵�����
%% CW������VRPTW��ʼ��
[init_vc,init_TD,init_vl,violate_INTW] = init_TW(r101,cap);
init_vc
%% LNS�Ż���ʼ��
[op_fvc,best_TD,op_vl,violate_OPTW,DEL] = LNS(init_vc,r101,cap,toRemove,D);
bestNV=size(op_fvc,1);