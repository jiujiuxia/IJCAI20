%% �����ں�·�����vehicles_customer����
%���룺vehicles_customer              ÿ�����������Ĺ˿�
%�����ins_pos                        ����ԭʼ·��λ��
%�����put_index                      ��Ҫ�����·�����
%�����in_index                       ������·�����������
%�����vehicles_customer              �����ں�·�����vehicles_customer����
function route_in= merge( vehicles_customer,in_index,put_index,in_pos )
put=vehicles_customer{put_index};       %��Ҫ�����·��
in=vehicles_customer{in_index};         %�������·��
len_i=length(in);
%���in_pos==1�����ʾ����λ��Ϊԭʼ·���е�һ�����ǰһ��λ��
if in_pos==1
    put_in=[put(end) in];
%���in_pos==len_i+1�����ʾ����λ��Ϊԭʼ·�������һ����ĺ�һ��λ��
elseif in_pos==len_i+1
    put_in=[in put(end)];
else
    put_in=[in(1:in_pos-1) put(end) in(in_pos:end)];
end
route_in=put_in;
end

