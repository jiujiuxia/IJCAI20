%% ��ʼ��������������·�ߣ�ÿ����װ������һ��������
function [ vehicles_customer ] = init_route( vehicles_customer )
vecnum=size(vehicles_customer,1);
for i=1:vecnum
    vehicles_customer{i}=i;                                     %��ʼһ����ֻȥһ���˿�����
end
end

