clear
clc
load op_c101.mat
%% ��importdata�����������ȡ�ļ�
c101=importdata('c101.txt');

E=c101(1,5);                                                %��������ʱ�䴰��ʼʱ��
L=c101(1,6);                                                %��������ʱ�䴰����ʱ��
cap=200;                                                    %��������
vertexs=c101(:,2:3);                                        %���е������x��y
customer=vertexs(2:end,:);                                  %�˿�����
cusnum=size(customer,1);                                    %�˿���
vecnum=cusnum;                                              %������
demands=c101(2:end,4);                                      %������
a=c101(2:end,5);                                            %�˿�ʱ�䴰��ʼʱ��[a[i],b[i]]
b=c101(2:end,6);                                            %�˿�ʱ�䴰����ʱ��[a[i],b[i]]
s=c101(2:end,7);                                            %�ͻ���ķ���ʱ��
h=pdist(vertexs);
dist=squareform(h);                                         %��������������ǹ�ϵ�����þ����ʾ����c[i][j]=dist[i][j]
% op_fvc=final_vehicles_customer;

%intersect(a,b)�����Եõ�a,b������������ͬ��Ԫ��


NV=size(op_fvc,1);
route=[];
for i=1:NV
    route=[route op_fvc{i}];
end
sr=sort(route);
LEN=length(sr);
%% Ѱ�Ҷ�ʧ��Ԫ��
INIT=1:100;
%setxor(a,b)���Եõ�a,b����������ͬ��Ԫ�أ�Ҳ�в��ڽ����е�Ԫ��
DEL=setxor(sr,INIT);

