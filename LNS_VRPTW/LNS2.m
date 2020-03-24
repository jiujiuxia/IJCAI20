%% LNS优化初始解
%输入init_vc      每辆车所经过的顾客
%输入dataset      数据集
%输入cap          每辆车容量约束
%toRemove         将要移出顾客的数量
%D                Remove过程中的随机元素
%输出op_fvc       每辆车所经过的顾客
%输出op_TD        所有车行驶的总距离
%输出op_vl        每辆车的装载量
%输出violate_OPTW 判断是否违背时间窗约束，0代表不违背，1代表违背
%输出DEL          检查最优解中是否存在元素丢失的情况
function [op_fvc,op_TD,op_vl,violate_OPTW,DEL] = LNS2(init_vc,dataset,cap,toRemove,D)
E=dataset(1,3);                                                %配送中心时间窗开始时间
L=dataset(1,4);                                                %配送中心时间窗结束时间
vertexs=dataset(:,1:2);                                        %所有点的坐标x和y
customer=vertexs(2:end,:);                                  %顾客坐标
cusnum=size(customer,1);                                    %顾客数
vecnum=cusnum;                                              %车辆数
demands=dataset(2:end,6);                                      %需求量
a=dataset(2:end,3);                                            %顾客时间窗开始时间[a[i],b[i]]
b=dataset(2:end,4);                                            %顾客时间窗结束时间[a[i],b[i]]
s=dataset(2:end,5);                                            %客户点的服务时间
h=pdist(vertexs);
dist=squareform(h);                                         %距离矩阵，满足三角关系，暂用距离表示花费c[i][j]=dist[i][j]

%% 计算每辆车所行驶的距离，以及所有车行驶的总距离
% final_vehicles_customer=op_fvc;
[ initTD,everyTD ] = travel_distance( init_vc,dist );
%% 循环Remove和Re_inserting过程，直到找到，比初始解好的方案
flag=1;
count=1;            %计数器
ITER=200;            %最大迭代次数
op_TD=initTD;        %当前最优总距离
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
%% 计算每辆车的装载量
op_vl= vehicle_load( op_fvc,demands );
%% 计算每辆车配送路线上在各个点开始服务的时间，还计算返回集配中心时间
bsv= begin_s_v( op_fvc,a,s,dist );
%% 判断是否违背时间窗约束，0代表不违背，1代表违背
[ violate_OPTW ] = Judge_TW( op_fvc,bsv,b,L );
%% 检查最优解中是否存在元素丢失的情况
 DEL=Judge_Del(op_fvc);


end

