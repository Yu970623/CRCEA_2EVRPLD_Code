%% --------------------------初始化第一层--------------------------
%step1: 将卫星按照角度排序，并选择角度间隔最大的点作为起点，在满足容量的条件下，将卫星依次添加至车辆；
%step2: 将添加的卫星按照倒序的方式进行调整。
%step3: 将同一车辆的卫星按照角度进行排序
function [seq_sats,cap_sats,vrp1,vlt1] = initsat(vrp2e,seq_r) 
    %global fleet; global demand; global dis_ds;  global num_sat;
    fleet = vrp2e.fleet;
    dis_ds = vrp2e.dis_ds; 
    dis_sc = vrp2e.dis_sc;  
    num_dep = length(vrp2e.rad_ds(:,1));
    demand = vrp2e.demand; 
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    % ----统计各卫星的货物总量----
    num_pop = size(seq_r,1);
    %----初始化变量----
    len_route1 = 2*(fleet(1,2)+num_sat);
    seq_sats   = zeros(num_pop,len_route1);         %预分配存储卫星序号2*fleet(1,2)+2*ns，超出空间删除
    cap_sats   = zeros(num_pop,len_route1);         %预分配存储卫星容量2*fleet(1,2)+2*ns，超出空间删除
    vrp1     = zeros(1,num_pop);
    vlt1     = zeros(1,num_pop);


    %% -=--初始化第一层----
    for i = 1:num_pop
        seq_sat   = zeros(1,len_route1);         %预分配存储卫星序号2*fleet(1,2)+2*ns，超出空间删除
        cap_sat   = zeros(1,len_route1);         %预分配存储卫星容量2*fleet(1,2)+2*ns，超出空间删除
        cap_s     = zeros(1,num_sat);
        sat_cus = zeros(1,num_cus);
        for c = 1:num_cus
            sat_cus(c) = seq_r(i, find(seq_r(i, 1:find(seq_r(i,:) == c + num_sat, 1)) <= num_sat, 1, 'last'));
        end
        %----按照距离采用轮盘赌将卫星分配给仓库----
        dep_sat = zeros(1,num_sat);              %存储每个卫星归属的仓库
        for c = 1:num_sat
            if ismember(c, sat_cus)
                dep_sat(c) = roulette(1./dis_sc(1:num_dep,c+num_dep),1);
            else
                dep_sat(c) = 0;
            end
        end
        nrs   = 0;   %路径总长度
        for s = 1:num_sat
            cus_s = find(sat_cus==s)+num_sat;
            cap_s(s) = sum(demand(cus_s));
            while cap_s(s)>fleet(1,1)
                nrs          = nrs+2;
                seq_sat(nrs-1) = dep_sat(s);
                seq_sat(nrs) = s+num_dep;
                cap_sat(nrs) = fleet(1,1);
                cap_s(s)     = cap_s(s)-fleet(1,1);
            end
        end
        for d = 1:num_dep
            dep_s                 = find(dep_sat==d)+num_dep;  %当前仓库下的卫星
            route                 = algcw1(dis_ds(d,dep_s),dis_ds(dep_s,dep_s),cap_s,fleet(1,1));
            loc                   = find(route>0);  %路段中卫星位置
            route(loc)            = dep_s(route(loc));  %卫星序号换成卫星整体序号
            cap_r                 = route;
            if length(loc)==1 && cap_s(route(loc)-num_dep) == 0
                route = [];
                cap_r = [];
                nr=0;
            else
                cap_r(loc)            = cap_s(route(loc)-num_dep);
                route(route==0)       = d;  %卫星序号替换0
                nr                    = numel(route);
            end
            seq_sat(nrs+1:nrs+nr) = route;
            cap_sat(nrs+1:nrs+nr) = cap_r;
            nrs                   = nrs+nr;
            seq_sat(nrs+1:end)    = [];
            cap_sat(nrs+1:end)    = [];
        end
        [vrp1(i),vlt1(i)] = mfirst(vrp2e,seq_sat,cap_sat);
        seq_sat(find(seq_sat ~= 0, 1, 'last')+1:end) = [];
        cap_sat(find(cap_sat ~= 0, 1, 'last')+1:end) = [];
        if length(seq_sat)~=length(cap_sat)
            seq_sat
        end
        seq_sats(i,:)=[seq_sat,zeros(1,len_route1-length(seq_sat))];
        cap_sats(i,:)=[cap_sat,zeros(1,len_route1-length(seq_sat))];
    end
end


%% --------------将客户初排序(基于CW)--------------------
function rount = algcw1(dis_dtos,dis_stos,demand_sat,capacity)
    nc         = length(dis_dtos);     %客户总数
    link       = zeros(2,nc);         %客户连接的其它客户：存储与本客户连接的其它客户
    nlink      = zeros(1,nc);         %客户连接的次数：用于识别该客户是否还可连接
    seq_veh    = 1:nc;                %客户对应的车辆序号：用于计算车辆的载货量
    cw_cc      = reshape(triu(bsxfun(@plus,dis_dtos,dis_dtos')-dis_stos+0.01,1),1,[]);  %计算节省的费用
    [~,srt_cp] = sort(cw_cc,'descend');    %将节省量进行排序
    npair      = nc*(nc-1)/2;    %所有的配对
    srt_cp     = srt_cp(1:npair);          %提取上三角矩阵的序号
    cr         = mod(srt_cp-1,nc)+1;  %排序值对应的行(行与列对应的是连接关系)
    cl         = ceil(srt_cp/nc);     %排序值对应的列
    for i = 1:npair
        nv1 = seq_veh(cr(i));                %提取客户1对应的车辆序号
        nv2 = seq_veh(cl(i));                %提取客户2对应的车辆序号 nv1<nv2
        %----满足可连接、不在同一路径、容量不超----
        if nlink(cr(i))<2 && nlink(cl(i))<2 && nv1~= nv2 && demand_sat(nv1)+demand_sat(nv2) <= capacity 
            demand_sat(nv1) = demand_sat(nv1)+demand_sat(nv2);  %合并车辆nv2的货物至车辆nv1
            demand_sat(nv2) = 0;                                %清空车辆nv2的货物
            seq_veh(seq_veh==nv2) = nv1;                        %将车辆nv2中客户对应车辆号设为nv1
            nlink(cr(i)) = nlink(cr(i))+1;                      %更新客户1的连接数目
            nlink(cl(i)) = nlink(cl(i))+1;                      %更新客户2的连接数目
            if link(1,cr(i))== 0
                link(1,cr(i)) = cl(i);
            else
                link(2,cr(i)) = cl(i);
            end
            if link(1,cl(i))== 0
                link(1,cl(i)) = cr(i);
            else
                link(2,cl(i)) = cr(i);
            end
        end  
    end
    rount   = zeros(1,2*nc+1);          %最长可能返回排序序列
    cus_isolate = find(nlink==0);       %寻找孤立的客户
    count = 2*numel(cus_isolate);
    if count>0
        rount(2:2:count) = cus_isolate; %孤立点单独形成一个回路
    end
    cus_start = find(nlink==1);         %寻找回路的起点
    for nv = cus_start
        if sum(rount==nv)==0            %判断该回路起点客户是否在回路中
           count = count+2; 
           rount(count) = nv;
           cus_next = link(1,nv);
           count = count+1;
           rount(count) = cus_next;
           while nlink(cus_next)==2
               count = count+1;
               if link(1,cus_next)==rount(count-2) %判断与当前客户的连接第一个客户是否已在路径中
                   rount(count) = link(2,cus_next);%存储与当前客户连接的第二个客户
                   cus_next = link(2,cus_next);
               else
                   rount(count) = link(1,cus_next);%存储与当前客户连接第一个客户
                   cus_next = link(1,cus_next);
               end
           end
        end
    end
    rount(count+1:end) = []; %清除没有存储客户的空间
end