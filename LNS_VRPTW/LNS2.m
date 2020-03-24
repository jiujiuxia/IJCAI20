%% LNS�Ż���ʼ��
%����init_vc      ÿ�����������Ĺ˿�
%����dataset      ���ݼ�
%����cap          ÿ��������Լ��
%toRemove         ��Ҫ�Ƴ��˿͵�����
%D                Remove�����е����Ԫ��
%���op_fvc       ÿ�����������Ĺ˿�
%���op_TD        ���г���ʻ���ܾ���
%���op_vl        ÿ������װ����
%���violate_OPTW �ж��Ƿ�Υ��ʱ�䴰Լ����0����Υ����1����Υ��
%���DEL          ������Ž����Ƿ����Ԫ�ض�ʧ�����
function [op_fvc,op_TD,op_vl,violate_OPTW,DEL] = LNS2(init_vc,dataset,cap,toRemove,D)
E=dataset(1,3);                                                %��������ʱ�䴰��ʼʱ��
L=dataset(1,4);                                                %��������ʱ�䴰����ʱ��
vertexs=dataset(:,1:2);                                        %���е������x��y
customer=vertexs(2:end,:);                                  %�˿�����
cusnum=size(customer,1);                                    %�˿���
vecnum=cusnum;                                              %������
demands=dataset(2:end,6);                                      %������
a=dataset(2:end,3);                                            %�˿�ʱ�䴰��ʼʱ��[a[i],b[i]]
b=dataset(2:end,4);                                            %�˿�ʱ�䴰����ʱ��[a[i],b[i]]
s=dataset(2:end,5);                                            %�ͻ���ķ���ʱ��
h=pdist(vertexs);
dist=squareform(h);                                         %��������������ǹ�ϵ�����þ����ʾ����c[i][j]=dist[i][j]

%% ����ÿ��������ʻ�ľ��룬�Լ����г���ʻ���ܾ���
% final_vehicles_customer=op_fvc;
[ initTD,everyTD ] = travel_distance( init_vc,dist );
%% ѭ��Remove��Re_inserting���̣�ֱ���ҵ����ȳ�ʼ��õķ���
flag=1;
count=1;            %������
ITER=200;            %����������
op_TD=initTD;        %��ǰ�����ܾ���
op_fvc=init_vc;
while count<ITER
% while flag
    %% Remove
    count
    [removed,rfvc] = Remove(cusnum,toRemove,D,dist,op_fvc);
    %% Re-inserting
    [ ReIfvc,RTD ] = Re_inserting(removed,rfvc,L,a,b,s,dist,demands,cap);
    if RTD<op_TD
        flag=0;
        op_fvc=ReIfvc;
        op_TD=RTD;
    end
    count=count+1;
end
%% ����ÿ������װ����
op_vl= vehicle_load( op_fvc,demands );
%% ����ÿ��������·�����ڸ����㿪ʼ�����ʱ�䣬�����㷵�ؼ�������ʱ��
bsv= begin_s_v( op_fvc,a,s,dist );
%% �ж��Ƿ�Υ��ʱ�䴰Լ����0����Υ����1����Υ��
[ violate_OPTW ] = Judge_TW( op_fvc,bsv,b,L );
%% ������Ž����Ƿ����Ԫ�ض�ʧ�����
 DEL=Judge_Del(op_fvc);


end

