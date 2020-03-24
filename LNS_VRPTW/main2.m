clear
clc

final = [];
%% 用importdata这个函数来读取文�? 
origin_data=importdata('vrp50.txt');
cap=40;
D=4;                                                       %Remove过程中的随机元素
toRemove=4;                                                %将要移出顾客的数�?
%%改变数组形状
origin_data = reshape(origin_data,1280,9,51);

init_solution = importdata('50RL_10.txt');
t1=clock;
for instance=1:1280
    r101 = origin_data(instance,:,:);
    r101=reshape(r101,9,51);
    r101 =r101';

    %% CW法构造VRPTW初始�?
    %[init_vc,init_TD,init_vl,violate_INTW] = init_TW(r101,cap);;
    %solution = "[0, 7, 8, 12, 20, 6, 0, 9, 11, 16, 0, 4, 1, 2, 17, 0, 15, 19, 0, 3, 14, 0, 10, 13, 0, 5, 0, 18, 0]"
    solution = init_solution(instance,:);
    a=solution{1};

    a(findstr('[',a))=[];
    a(findstr(']',a))=[];
    a(findstr('"',a))=[];
    a(findstr('"',a))=[];
    %a=strrep(a,' ','');
    a(find(isspace(a))) = [] ;
    a=strrep(a,',',' ');
    a=str2num(a);
    init_vc={};
    temp=[];
    count = 1;
    for i=1:length(a)
        if a(i) ~= 0
            temp = [temp a(i)];
        else
            if ~isempty(temp)
                init_vc{count}=temp;
                count=count+1;
            end
            temp = [];
        end

    end
    init_vc = init_vc';
    %% LNS优化初始�?
    [op_fvc,best_TD,op_vl,violate_OPTW,DEL] = LNS2(init_vc,r101,cap,toRemove,D);
    bestNV=size(op_fvc,1);
    final = [final best_TD];
end    
t2=clock;
time = etime(t2,t1);
fid=fopen('50RL_10_final_4_200.csv','w');
for i = 1:size(final,2)
    fprintf(fid,'%s\n',final(i));
end
fprintf(fid,'%s\n',time);
fclose(fid);