function seq_c = adjustsat(vrp2e,seq_c,seq_s,cap_s)
    if rand < 1
        seq_c = adjustsat1(vrp2e,seq_c,seq_s,cap_s);
    else
        seq_c = adjustsat2(vrp2e,seq_c);
    end
end
%% --------------根据第一层信息调整卫星(第二层客户路径顺序不改变，整体)-----------------------
function seq_c = adjustsat1(vrp2e,seq_c,seq_s,cap_s)
    %global dis_ds; global fleet;
    dis_ds = vrp2e.dis_ds;
    fleet = vrp2e.fleet;
    %----查询第一层需要被替换的卫星信息----
    loc_d = find(seq_s == 0);                           %调度中心对应的序号
    num_f = ceil(sum(cap_s)/fleet(1,1));                %第一层路径的车辆数
    s_re  = zeros(1,4);                                 %存储被替换的卫星号、被替换的最小容量、最大容量、减少的代价 
    for i = 1:num_f

    end
end


%% --------------根据第二层信息更改卫星(第二层客户路径顺序不改变，所有卫星插入子路段)-----------------------
function seq_c = adjustsat2(vrp2e,seq_c)
    %global dis_sc; global num_sat; global num_cus;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;
    
    loc_s = [find(seq_c<=num_sat),length(seq_c)+1];     %第一层路径中卫星所在的位置
    len_s = length(loc_s)-1;                            %第一层总的路径数目
    for i = 1:len_s                   
        len_r = loc_s(i+1)-loc_s(i)-1;                  %子路段客户数目
        if len_r > 0
            route  = [seq_c(loc_s(i+1)-1),seq_c(loc_s(i)+1:loc_s(i+1)-1)];     %一条路径中的卫星序号
            %----检索最小的卫星插入位置----
            [cost_s,s_ins]  = min(bsxfun(@minus,dis_sc(1:num_sat,route(1:end-1))+dis_sc(1:num_sat,route(2:end)),dis_sc(route(1:end-1)+(route(2:end)-1)*(num_sat+num_cus))),[],1);
            [~,loc_ins]     = min(cost_s);
            s               = s_ins(loc_ins);
            seq_c(loc_s(i)) = s;
            seq_c(loc_s(i)+1:loc_s(i+1)-1) = [route(loc_ins+1:end),route(2:loc_ins)];
        end
    end
end