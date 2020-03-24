%% CW法构造VRPTW初始解
%输入dataset      数据集
%输入cap          每辆车容量约束
%输出init_vc      每辆车所经过的顾客
%输出init_TD      所有车行驶的总距离
%输出init_vl      每辆车的装载量
%输出violate_INTW 判断是否违背时间窗约束，0代表不违背，1代表违背
function [init_vc,init_TD,init_vl,violate_INTW] = init_TW(dataset,cap)
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
vehicles_customer=cell(vecnum,1);                           %每辆车所经过的顾客

%% 初始化各个车辆配送路线，每个安装场地由一辆车配送，有d2需求的安装场地在前面加上加工车间用0表示
[ vehicles_customer ] = init_route( vehicles_customer );
%% 计算融合任意两个路径的节约值
[sav,sav_sort ] = sav_cal(vehicles_customer,L,a,b,s,dist );
%% 构造VRPTW初始解
%计算每辆车离开当前路径上集配中心时的载货量、每辆车离开当前路径上每一点时的载货量
[vl]= vehicle_load( vehicles_customer,demands);
count=0;                                                                %计数器
pos=1;                                                                  %标记第几大距离节约值
while sav_sort(1,1)>0
    %% 更新插入之后的sav矩阵
    [sav,sav_sort ] = sav_cal(vehicles_customer,L,a,b,s,dist );
    
    if ~isempty(sav_sort)
        in_index=sav_sort(:,4);                                                             %被插入路径的坐标序号
        put_index=sav_sort(:,3);                                                            %将要插入的路径序号
        in_pos=sav_sort(:,2);                                                               %插入原始路径位置
        route_in= merge( vehicles_customer,in_index(1),put_index(1),in_pos(1) );            %预先将两条路径合并
        Ld=leave_load( route_in,demands);                                                   %预先计算合并后的路径上离开各个点的载货量
        if Ld<=cap
            %% 更新融合路径后的vehicles_customer矩阵
            [ vehicles_customer ] = update_vehicles_customer( vehicles_customer,in_index(1),put_index(1),in_pos(1) );
            pos=1;
        else
            pos=pos+1;
            if pos<size(in_index,1)
                route_in= merge( vehicles_customer,in_index(pos),put_index(pos),in_pos(pos) );      %预先将两条路径合并
                Ld=leave_load( route_in,demands);                                                   %预先计算合并后的路径上离开各个点的载货量
                if Ld<=cap
                    %% 更新融合路径后的vehicles_customer矩阵
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
    %% 计算每辆车的装载量
    %计算每辆车离开当前路径上集配中心时的载货量、每辆车离开当前路径上每一点时的载货量
    [vl]= vehicle_load( vehicles_customer,demands);
    count=count+1;
    count;
end




%% 根据vehicles_customer整理出final_vehicles_customer，将vehicles_customer中空的数组移除
[ final_vehicles_customer,vehicles_used ] = deal_vehicles_customer( vehicles_customer );
%% 计算每辆车所行驶的距离，以及所有车行驶的总距离
[ sumTD,everyTD ] = travel_distance( final_vehicles_customer,dist );
init_vc=final_vehicles_customer;
init_TD=sumTD;

%% 计算每辆车的装载量
init_vl= vehicle_load( init_vc,demands );
%% 计算每辆车配送路线上在各个点开始服务的时间，还计算返回集配中心时间
bsv= begin_s_v( init_vc,a,s,dist );
%% 判断是否违背时间窗约束，0代表不违背，1代表违背
[ violate_INTW ] = Judge_TW( init_vc,bsv,b,L );

end

