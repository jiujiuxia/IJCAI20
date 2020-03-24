clear
clc

% load CustomerPosition.mat c101
%% ��importdata�����������ȡ�ļ� 
rc201=importdata('rc201.txt');

E=rc201(1,5);                                                %��������ʱ�䴰��ʼʱ��
L=rc201(1,6);                                                %��������ʱ�䴰����ʱ��

cap=1000;                                                    %��������

vertexs=rc201(:,2:3);                                        %���е������x��y
customer=vertexs(2:end,:);                                  %�˿�����
cusnum=size(customer,1);                                    %�˿���
vecnum=cusnum;                                              %������
demands=rc201(2:end,4);                                      %������
a=rc201(2:end,5);                                            %�˿�ʱ�䴰��ʼʱ��[a[i],b[i]]
b=rc201(2:end,6);                                            %�˿�ʱ�䴰����ʱ��[a[i],b[i]]
s=rc201(2:end,7);                                            %�ͻ���ķ���ʱ��
h=pdist(vertexs);
dist=squareform(h);                                         %��������������ǹ�ϵ�����þ����ʾ����c[i][j]=dist[i][j]
vehicles_customer=cell(vecnum,1);                           %ÿ�����������Ĺ˿�

%% ��ʼ��������������·�ߣ�ÿ����װ������һ�������ͣ���d2����İ�װ������ǰ����ϼӹ�������0��ʾ
[ vehicles_customer ] = init_route( vehicles_customer );
%% �����ں���������·���Ľ�Լֵ
[sav,sav_sort ] = sav_cal(vehicles_customer,L,a,b,s,dist );
%% ����VRPTW��ʼ��
%����ÿ�����뿪��ǰ·���ϼ�������ʱ���ػ�����ÿ�����뿪��ǰ·����ÿһ��ʱ���ػ���
[vl]= vehicle_load( vehicles_customer,demands);
count=0;                                                                %������
pos=1;                                                                  %��ǵڼ�������Լֵ
while sav_sort(1,1)>0
    %% ���²���֮���sav����
    [sav,sav_sort ] = sav_cal(vehicles_customer,L,a,b,s,dist );
    
    if ~isempty(sav_sort)
        in_index=sav_sort(:,4);                                                             %������·�����������
        put_index=sav_sort(:,3);                                                            %��Ҫ�����·�����
        in_pos=sav_sort(:,2);                                                               %����ԭʼ·��λ��
        route_in= merge( vehicles_customer,in_index(1),put_index(1),in_pos(1) );            %Ԥ�Ƚ�����·���ϲ�
        Ld=leave_load( route_in,demands);                                                   %Ԥ�ȼ���ϲ����·�����뿪��������ػ���
        if Ld<=cap
            %% �����ں�·�����vehicles_customer����
            [ vehicles_customer ] = update_vehicles_customer( vehicles_customer,in_index(1),put_index(1),in_pos(1) );
            pos=1;
        else
            pos=pos+1;
            if pos<size(in_index,1)
                route_in= merge( vehicles_customer,in_index(pos),put_index(pos),in_pos(pos) );      %Ԥ�Ƚ�����·���ϲ�
                Ld=leave_load( route_in,demands);                                                   %Ԥ�ȼ���ϲ����·�����뿪��������ػ���
                if Ld<=cap
                    %% �����ں�·�����vehicles_customer����
                    [ vehicles_customer ] = update_vehicles_customer( vehicles_customer,in_index(pos),put_index(pos),in_pos(pos) );
                    pos=1;
                end
            else
                break;
            end
        end
    else
        break;
    end
    %% ����ÿ������װ����
    %����ÿ�����뿪��ǰ·���ϼ�������ʱ���ػ�����ÿ�����뿪��ǰ·����ÿһ��ʱ���ػ���
    [vl]= vehicle_load( vehicles_customer,demands);
    count=count+1;
    count
end




%% ����vehicles_customer�����final_vehicles_customer����vehicles_customer�пյ������Ƴ�
[ final_vehicles_customer,vehicles_used ] = deal_vehicles_customer( vehicles_customer );
%% ����ÿ��������ʻ�ľ��룬�Լ����г���ʻ���ܾ���
[ sumTD,everyTD ] = travel_distance( final_vehicles_customer,dist );
%% ��ͼ
plot(customer(:,1),customer(:,2),'ro');hold on;
plot(vertexs(1,1),vertexs(1,2),'pm');hold on;


for i=1:vehicles_used
    part_seq=final_vehicles_customer{i};            %ÿ�����������Ĺ˿�
    len=length(part_seq);                           %ÿ�����������Ĺ˿�����
    for j=0:len
        %��j=0ʱ���������������ĳ��������·���ϵĵ�һ���˿�
        if j==0                                     
            c1=customer(part_seq(1),:);
            plot([vertexs(1,1),c1(1)],[vertexs(1,2),c1(2)],'-');
        %��j=lenʱ�������Ӹ�·���ϵ����һ���˿ͳ���������������
        elseif j==len
            c_len=customer(part_seq(len),:);
            plot([c_len(1),vertexs(1,1)],[c_len(2),vertexs(1,2)],'-');
        %���򣬳�����·���ϵ�ǰһ���˿͵����·���Ͻ��ڵ���һ���˿�
        else
            c_pre=customer(part_seq(j),:);
            c_lastone=customer(part_seq(j+1),:);
            plot([c_pre(1),c_lastone(1)],[c_pre(2),c_lastone(2)],'-');
        end
    end
end

%0������������
for k=1:vehicles_used
    part_seq=final_vehicles_customer{k};            %ÿ�����������Ĺ˿�
    sqe1=[0 part_seq 0];
    disp(['����',num2str(k),'��·�����£�']);
    disp(sqe1)
end

disp(['���г�����ʻ���ܾ���Ϊ��',num2str(sumTD)]);

%% ����ÿ������װ����
final_vl= vehicle_load( final_vehicles_customer,demands );
%% ����ÿ��������·�����ڸ����㿪ʼ�����ʱ�䣬�����㷵�ؼ�������ʱ��
bsv= begin_s_v( final_vehicles_customer,a,s,dist );
%% �ж��Ƿ�Υ��ʱ�䴰Լ����0����Υ����1����Υ��
[ violate_TW ] = Judge_TW( final_vehicles_customer,bsv,b,L );

save('init_rc201.mat')