clear
clc

% load CustomerPosition.mat c101
%% 用importdata这个函数来读取文件 
rc201=importdata('rc201.txt');

E=rc201(1,5);                                                %配送中心时间窗开始时间
L=rc201(1,6);                                                %配送中心时间窗结束时间

cap=1000;                                                    %车辆负荷

vertexs=rc201(:,2:3);                                        %所有点的坐标x和y
customer=vertexs(2:end,:);                                  %顾客坐标
cusnum=size(customer,1);                                    %顾客数
vecnum=cusnum;                                              %车辆数
demands=rc201(2:end,4);                                      %需求量
a=rc201(2:end,5);                                            %顾客时间窗开始时间[a[i],b[i]]
b=rc201(2:end,6);                                            %顾客时间窗结束时间[a[i],b[i]]
s=rc201(2:end,7);                                            %客户点的服务时间
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
    count
end




%% 根据vehicles_customer整理出final_vehicles_customer，将vehicles_customer中空的数组移除
[ final_vehicles_customer,vehicles_used ] = deal_vehicles_customer( vehicles_customer );
%% 计算每辆车所行驶的距离，以及所有车行驶的总距离
[ sumTD,everyTD ] = travel_distance( final_vehicles_customer,dist );
%% 画图
plot(customer(:,1),customer(:,2),'ro');hold on;
plot(vertexs(1,1),vertexs(1,2),'pm');hold on;


for i=1:vehicles_used
    part_seq=final_vehicles_customer{i};            %每辆车所经过的顾客
    len=length(part_seq);                           %每辆车所经过的顾客数量
    for j=0:len
        %当j=0时，车辆从配送中心出发到达该路径上的第一个顾客
        if j==0                                     
            c1=customer(part_seq(1),:);
            plot([vertexs(1,1),c1(1)],[vertexs(1,2),c1(2)],'-');
        %当j=len时，车辆从该路径上的最后一个顾客出发到达配送中心
        elseif j==len
            c_len=customer(part_seq(len),:);
            plot([c_len(1),vertexs(1,1)],[c_len(2),vertexs(1,2)],'-');
        %否则，车辆从路径上的前一个顾客到达该路径上紧邻的下一个顾客
        else
            c_pre=customer(part_seq(j),:);
            c_lastone=customer(part_seq(j+1),:);
            plot([c_pre(1),c_lastone(1)],[c_pre(2),c_lastone(2)],'-');
        end
    end
end

%0代表配送中心
for k=1:vehicles_used
    part_seq=final_vehicles_customer{k};            %每辆车所经过的顾客
    sqe1=[0 part_seq 0];
    disp(['车辆',num2str(k),'的路径如下：']);
    disp(sqe1)
end

disp(['所有车辆行驶的总距离为：',num2str(sumTD)]);

%% 计算每辆车的装载量
final_vl= vehicle_load( final_vehicles_customer,demands );
%% 计算每辆车配送路线上在各个点开始服务的时间，还计算返回集配中心时间
bsv= begin_s_v( final_vehicles_customer,a,s,dist );
%% 判断是否违背时间窗约束，0代表不违背，1代表违背
[ violate_TW ] = Judge_TW( final_vehicles_customer,bsv,b,L );

save('init_rc201.mat')