%% ���������ں���������·���Ľ�Լֵ
%���룺vehicles_customer              ÿ�����������Ĺ˿�
%����L                                ��������ʱ�䴰
%����a                                �˿�ʱ�䴰
%����b                                �˿�ʱ�䴰
%����s                                ����ÿ���˿͵�ʱ��
%���룺dist                           �������
%�����sav                            ��ʼ�����Լ����[�����Լֵ ����ԭʼ·��λ�� ��Ҫ�����·����� ������·�����������]
%�����sav_sort                       sav_sort��sav��һ�о����Լֵ��С��������


function [sav,sav_sort ] = sav_cal(vehicles_customer,L,a,b,s,dist )
sav=[];                                              %���ڼ�������֮��ľ����Լ��
vecnum=size(vehicles_customer,1);                    %������
%% ����һ���µ�����n��4�У�[�����Լֵ ����ԭʼ·��λ�� ��Ҫ�����·����� ������·�����������]
w_e=zeros(vecnum,1);

%% ������ʱ�䴰Լ����sav����
for i=1:vecnum
    in=vehicles_customer{i};            %������·��
    len_in=size(in,2);                  %������·������������װ���غͼӹ����������
    for j=1:vecnum
        put=vehicles_customer{j};       %Ҫ����·��
        len_put=size(put,2);            %Ҫ����·�����������˿͵�����
        
        %������·������û�мӹ����䶼����,����·����û�мӹ�����
        if (i~=j)&&(w_e(j)==0)&&(len_put==1)
            %�ӵ�1����϶���룬����ԭʼ·��Ϊ0 1 2����Ҫ��3�����һ����4�����ܲ����3012��0312��0132��0123
            for k=1:len_in+1
                if k==1
                    put_in=[put(1) in];
                elseif k==len_in+1
                    put_in=[in put(1)];
                else
                    put_in=[in(1:k-1) put(1) in(k:end)];
                end
                dealt=part_length(in,dist)+part_length(put,dist)-part_length(put_in,dist);
                sav=[sav;dealt,k,j,i];
            end
        end
    end
end

%% ������������ʱ�䴰Լ����sav����Ļ����ϣ���һ������ʱ�䴰Լ������������ʱ�䴰Լ������ɾ��
sav_row=size(sav,1);                        %sav��������
del=zeros(sav_row,1);                       %��Ǹ����Ƿ�ɾ��,1��ʾ��ɾ����0��ʾ����ɾ��
for i=1:sav_row
    pos=sav(i,2);
    put=vehicles_customer{sav(i,3)};
    in=vehicles_customer{sav(i,4)};
    len_i=length(in);
    if pos==1
        put_in=[put(end) in];
    elseif pos==len_i+1
        put_in=[in put(end)];
    else
        put_in=[in(1:pos-1) put(end) in(pos:end)];
    end
    [bs,back]= begin_s( put_in,a,s,dist );
    for j=1:len_i+1
        if put_in(j)~=0
            if (bs(j)>b(put_in(j)))||(back>L)
                del(i)=1;
            end
        else
            if (bs(j)>l)||(back>L)
                del(i)=1;
            end
        end
    end
end
del_index= del==1;                             %�ҳ���ɾ���е����
sav(del_index,:)=[];                                %�����ɾ�����д�sav������ɾ��

%% ��sav�����յ�1�У��Ӵ�С��˳������
sav_sort=sav;
if ~isempty(sav)
    [~,sort_index]=sort(sav(:,1),'descend');
    sav_sort=sav(sort_index,:);
end
end

