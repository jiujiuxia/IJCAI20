%% CW������VRPTW��ʼ��
%����dataset      ���ݼ�
%����cap          ÿ��������Լ��
%���init_vc      ÿ�����������Ĺ˿�
%���init_TD      ���г���ʻ���ܾ���
%���init_vl      ÿ������װ����
%���violate_INTW �ж��Ƿ�Υ��ʱ�䴰Լ����0����Υ����1����Υ��
function [init_vc,init_TD,init_vl,violate_INTW] = init_TW(dataset,cap)
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
    count;
end




%% ����vehicles_customer�����final_vehicles_customer����vehicles_customer�пյ������Ƴ�
[ final_vehicles_customer,vehicles_used ] = deal_vehicles_customer( vehicles_customer );
%% ����ÿ��������ʻ�ľ��룬�Լ����г���ʻ���ܾ���
[ sumTD,everyTD ] = travel_distance( final_vehicles_customer,dist );
init_vc=final_vehicles_customer;
init_TD=sumTD;

%% ����ÿ������װ����
init_vl= vehicle_load( init_vc,demands );
%% ����ÿ��������·�����ڸ����㿪ʼ�����ʱ�䣬�����㷵�ؼ�������ʱ��
bsv= begin_s_v( init_vc,a,s,dist );
%% �ж��Ƿ�Υ��ʱ�䴰Լ����0����Υ����1����Υ��
[ violate_INTW ] = Judge_TW( init_vc,bsv,b,L );

end

