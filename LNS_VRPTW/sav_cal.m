%% 计算任意融合任意两个路径的节约值
%输入：vehicles_customer              每辆车所经过的顾客
%输入L                                集配中心时间窗
%输入a                                顾客时间窗
%输入b                                顾客时间窗
%输入s                                服务每个顾客的时间
%输入：dist                           距离矩阵
%输出：sav                            初始距离节约矩阵[距离节约值 插入原始路径位置 将要插入的路径序号 被插入路径的坐标序号]
%输出：sav_sort                       sav_sort按sav第一列距离节约值大小降序排列


function [sav,sav_sort ] = sav_cal(vehicles_customer,L,a,b,s,dist )
sav=[];                                              %用于计算两点之间的距离节约量
vecnum=size(vehicles_customer,1);                    %车辆数
%% 创造一个新的数组n行4列，[距离节约值 插入原始路径位置 将要插入的路径序号 被插入路径的坐标序号]
w_e=zeros(vecnum,1);

%% 不考虑时间窗约束的sav矩阵
for i=1:vecnum
    in=vehicles_customer{i};            %被插入路径
    len_in=size(in,2);                  %被插入路径中所经过安装场地和加工车间的数量
    for j=1:vecnum
        put=vehicles_customer{j};       %要插入路径
        len_put=size(put,2);            %要插入路径中所经过顾客的数量
        
        %被插入路径上有没有加工车间都可以,插入路径上没有加工车间
        if (i~=j)&&(w_e(j)==0)&&(len_put==1)
            %从第1个空隙插入，比如原始路径为0 1 2，现要将3插进来一共有4个可能插入点3012、0312、0132、0123
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

%% 在上述不考虑时间窗约束的sav矩阵的基础上，进一步考虑时间窗约束，将不符合时间窗约束的行删掉
sav_row=size(sav,1);                        %sav矩阵行数
del=zeros(sav_row,1);                       %标记该行是否被删除,1表示被删除，0表示不被删除
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
del_index= del==1;                             %找出被删除行的序号
sav(del_index,:)=[];                                %将标记删除的行从sav矩阵中删除

%% 将sav矩阵按照第1列，从大到小的顺序排列
sav_sort=sav;
if ~isempty(sav)
    [~,sort_index]=sort(sav(:,1),'descend');
    sav_sort=sav(sort_index,:);
end
end

