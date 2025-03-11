%% 第二层路径的局部搜索----
function seq_c = local2(vrp2e,seq_c)
    %----5种局部搜索----
%     seq_c = split2(vrp2e,seq_c);
    seq_c = move2(vrp2e,seq_c); %随机删除所有客户并重新插入完整路段对比结果
    seq_c = swap2(vrp2e,seq_c); %随机将所有客户与其单一+连续邻居交换后对比结果
    seq_c = opt2(vrp2e,seq_c);  %每个子路段内，遍历逆转部分子路段
    seq_c = opt2x(vrp2e,seq_c); %拼接同一卫星下不同子路段内的子路段
    %----end----
end

%% --------------split2策略:第二层路径优化--------------------
function seq_cs = split2(vrp2e,route)
    fleet = vrp2e.fleet;
    demand = vrp2e.demand;  
    dis_sc = vrp2e.dis_sc;  
    num_sat = vrp2e.num_sat;  

    len_route = numel(route);
    seq_cs    = zeros(1,len_route+fleet(2,2));
    rg        = 0;
    %----卫星s的所有路径----
    sat_fleet     = route(route<=num_sat);                      %车辆对应的卫星
    sign_fleet    = cumsum(route<=num_sat,2);                    %卫星和客户对应的车辆
    for s = 1:num_sat
        sign_fleet_s        = find(sat_fleet==s);                 %卫星s对应的车辆号码
        [logic_loc,~]       = ismember(sign_fleet,sign_fleet_s);  %卫星s及对应客户参数为1
        cus_sat             = route(logic_loc);                   %卫星s的所有路径（含s）
        route_s             = cus_sat(cus_sat>num_sat);                %提取路径中的客户点（不含s）
        len_s               = numel(sign_fleet_s);
        if len_s >1
            seq_c           = splitopt(dis_sc(s,route_s),dis_sc(route_s,route_s),demand(route_s),fleet(2,1));
            loc_c           = find(seq_c>0);
            seq_c(loc_c)    = route_s(seq_c(loc_c));
            seq_c(seq_c==0) = s;
            len_r           = numel(seq_c);
            seq_cs(rg+1:rg+len_r) = seq_c;
            rg              = rg+len_r;
        else
            len_r           = numel(route_s)+1;
            seq_cs(rg+1:rg+len_r) = [s,route_s];
            rg              = rg+len_r;
        end
    end
    seq_cs(rg+1:end) = [];
end
% ----按照split算法对路径进行重新分割----
 function seq_c = splitopt(dis_sc,dis_cc,require,capcity)
    nc        = length(dis_sc);                  %被选取的卫星数目
    point     = zeros(1,nc);                    %每个客户的前向指针
    cost_c    = inf*ones(1,nc);                 %到当前客户需要的路径长度
    for i = 1:nc
        load  = 0;                              %当前的车载容量
        cost  = 2*dis_sc(i);                    %当前的路径长度
        j     = i;
        while j<=nc && load<=capcity
            load  = load+require(j);
            if j>i
                cost = cost-dis_sc(j-1)+dis_cc(j-1,j)+dis_sc(j); %依次插入卫星代价
            end
            if load<=capcity  %容量是否满足判断
                if i-1 ==0
                    cost_c(j) = cost;
                    point(j)  = i-1;
                elseif cost_c(i-1)+cost<cost_c(j)  %车辆长度代价比较
                    cost_c(j) = cost_c(i-1)+cost;
                    point(j)  = i-1;
                end
                j = j+1;
            end
        end
    end
    %----根据指针在路径中插入卫星点----
    seq_c = [1:nc,zeros(1,nc)];
    r     = 1;                        %统计路径的数目
    i     = point(nc);                %插入卫星点的位置
    seq_c(i+2:end) = seq_c(i+1:end-1);
    seq_c(i+1) = 0;
    while i>0
        i = point(i);
        r = r+1;
        seq_c(i+2:end) = seq_c(i+1:end-1);
        seq_c(i+1) = 0;
    end
    seq_c(nc+r+1:end) = [];
 end
 
%% --------------------------2-opt策略:第二层路径优化(同一路径中的卫星优化)-------------------------
function seq_cs = opt2(vrp2e,seq_cs)
    num_sat = vrp2e.num_sat;
    loc_s = [find(seq_cs<=num_sat),numel(seq_cs)+1];         %卫星对应的序号
    nfs   = length(loc_s)-1;                            %车辆总数 
    for i = 1:nfs
        if loc_s(i+1)-loc_s(i)>3                        %需要优化的条件：客户数目>2
           seq_cs(loc_s(i):loc_s(i+1)-1) = opttwo(vrp2e,[seq_cs(loc_s(i):loc_s(i+1)-1),seq_cs(loc_s(i))]);  %逐一对每趟旅程中的客户进行交换
        end
    end
end
%------------采用2-opt策略将同一条路径重组------------------
function route = opttwo(vrp2e,route)  %逆转部分子路段
    Energy_distribution = funcopt2(vrp2e,route);
    len_route = numel(route);   %客户数目
    i         = 2;
    while i <= len_route-2
        j = i+1;
        while j <= len_route-1
            new_route = [route(1:i-1), route(j:-1:i), route(j+1:end)];
            newEnergy_distribution = funcopt2(vrp2e,new_route);
            diff_dis = sum(Energy_distribution)-sum(newEnergy_distribution);  %计算新路径的总能耗
            if diff_dis > 0
                Energy_distribution = newEnergy_distribution;
                route(i:j) = route(j:-1:i); 
                i = 2;
                j = i+1;
            else
                j = j+1;
            end
        end
        i = i+1;
    end
    route = route(1:end-1);              %生成新的路径
end
%% --------------------------2-opt*策略:第二层路径优化-------------------------
function seq_cs = opt2x(vrp2e,seq_cs)
    num_sat = vrp2e.num_sat;
    
    len_route   = numel(seq_cs);
    loc_s = [find(seq_cs<=num_sat),len_route+1];              %卫星对应的序号
    nfs     = length(loc_s)-1;                   %车辆总数
    route   = cell(nfs,1);                       %将每条路径独立的存储
    for i = 1:nfs
        route{i} = [seq_cs(loc_s(i):loc_s(i+1)-1),seq_cs(loc_s(i))];
    end
    %----2-opt*----
    for i = 1:nfs-1
        for j = i+1:nfs
            if route{i}(1)==route{j}(1) && numel(route{i})>2 && numel(route{j})>2
               [route{i},route{j}] = opttwox(vrp2e,route{i},route{j});  %注意交换同卫星下出发的路段中的客户
            end
        end
    end
    %----路径重组----
    r = 0;
    for i = 1:nfs
        len = numel(route{i})-1;
        if len>1
            seq_cs(r+1:r+len) = route{i}(1:len);  %更新客户与卫星序列
            r = r+len;
        end
    end
    seq_cs(r+1:end) = [];
end
%------------采用2-opt*策略将同一卫星的两条路径重组------------------
function [seq_c1,seq_c2] = opttwox(vrp2e,seq_c1,seq_c2)
    nc1 = numel(seq_c1);
    nc2 = numel(seq_c2);
    fr  = funcopt2(vrp2e,seq_c1)+funcopt2(vrp2e,seq_c2);  %计算原始两条路径的总距离
    i   = 1;
    while i <= nc1-1  %使用变量i遍历第一条路径seq_c1中的所有客户，但不包括最后一个客户
        j = 1;
        while j <= nc2-1    %使用变量j遍历第二条路径seq_c2中的所有客户，同样不包括最后一个客户
            %----第一种组合方式----
            ra1 = [seq_c1(1:i),seq_c2(j+1:end)];  %将seq_c1的前i个客户与seq_c2的从j+1到末尾的客户组合成新的路径ra1
            ra2 = [seq_c2(1:j),seq_c1(i+1:end)];  %将seq_c2的前j个客户与seq_c1的从i+1到末尾的客户组合成新的路径ra2
            fra    = funcopt2(vrp2e,ra1)+funcopt2(vrp2e,ra2);  %成本计算方法，包括约束违反情况
            %----第二种组合方式----
            rb1 = [seq_c1(1:i),seq_c2(j:-1:1)];  %将seq_c1的前i个客户与seq_c2的从j到第一个客户逆序组合成新的路径rb1
            rb2 = [seq_c1(end:-1:i+1),seq_c2(j+1:end)];  %将seq_c1的从最后一个客户逆序到i+1的客户与seq_c2的从j+1到末尾的客户组合成新的路径rb2
            frb    = funcopt2(vrp2e,rb1)+funcopt2(vrp2e,rb2);  %成本计算方法，包括约束违反情况
            %----判断优劣---- 每种组合方式计算新的总距离（fra和frb）
            if fra <= frb
                if fra < fr
                    seq_c1 = ra1;
                    seq_c2 = ra2;
                    fr     = fra;
                    i      = 1;
                    j      = 0;
                end
            else
                if frb < fr
                    seq_c1 = rb1;
                    seq_c2 = rb2;
                    fr     = frb;
                    i      = 1;
                    j      = 0;
                end
            end
            nc1    = numel(seq_c1);
            nc2    = numel(seq_c2);
            j = j+1;    
       end
       i = i+1;    
    end
end
 %% --------------------------move2策略:第二层路径优化-------------------------
function route = move2(vrp2e,route)
    neib_cc = vrp2e.neib_cc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    tao     = min(25,num_cus-1);
    %----待插入的客户随机排序----
    seq_c   = randperm(num_cus)+num_sat;   %将客户从一个位置移动到另一个位置来改善路径

    for c = seq_c
        loc_c      = find(route==c);                            %删除客户所在解中的位置
        %----计算原客户的代价----
        cost_init = funcopt4(vrp2e,route);
        seq_r         = [route(1:loc_c-1),route(loc_c+1:end)];
        neib_c        = neib_cc(c-num_sat,2:tao+1);               %邻居客户
        [~,pos_in]    = ismember(neib_c,seq_r);                   %邻居客户在路径中的位置（含自身）

        % 计算插入到 rfront 后的新路径
        c_front = arrayfun(@(k) [seq_r(1:k-1), c, seq_r(k:end)], pos_in, 'UniformOutput', false);
        cost_front = arrayfun(@(k) [funcopt4(vrp2e,c_front{k})], 1:length(c_front), 'UniformOutput', false);
        % 计算插入到 rback 后的新路径
        c_back = arrayfun(@(k) [seq_r(1:k), c, seq_r(k+1:end)], pos_in, 'UniformOutput', false);
        cost_back = arrayfun(@(k) [funcopt4(vrp2e,c_back{k})], 1:length(c_back), 'UniformOutput', false);
        [cost_c,loc]  = min([cell2mat(cost_front);cell2mat(cost_back)],[],1); %cost_c存储插入邻居X前或后最小成本，loc存储插入邻居X前还是后
        [cost_new,loc_in] = min(cost_c);  %最小邻居的成本和该邻居序号
        if cost_new < cost_init
            loc_in       = pos_in(loc_in)+loc(loc_in)-2;
            route        = [seq_r(1:loc_in),c,seq_r(loc_in+1:end)];
        end
    end
    %----消除空路径----
    len_route   = numel(route);                                 %已有的路径长度
    loc_sat     = [find(route<=num_sat),len_route+1];           %卫星在路径中的位置
    route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径    
end

%% --------------------------swap2策略:第二层路径优化-------------------------
function route = swap2(vrp2e,route)
    neib_cc = vrp2e.neib_cc; 
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;

    tao     = min(25,num_cus-1);  %设置邻居客户数量tao
    %----待插入的客户随机排序----
    seq_c      = randperm(num_cus)+num_sat;  %随机生成一个待插入的客户序列seq_c
    sign_fleet = cumsum(route<=num_sat,2);               %卫星和客户对应的车辆
    %----单个临近点交换----
    for c1 = seq_c
        loc_c1 = find(route==c1);  %找到客户c1在路径中的位置loc_c1
        i = 2;
        while i <= tao+1
            c2     = neib_cc(c1-num_sat,i);  %对于客户c1的每个邻居客户c2
            loc_c2 = find(route==c2);
            route2 = route;
            route2(loc_c1) = route(loc_c2);  %交换c1和c2在路径中的位置
            route2(loc_c2) = route(loc_c1);
            fun1 = funcopt1(vrp2e,route(sign_fleet==sign_fleet(loc_c1)))+funcopt1(vrp2e,route(sign_fleet==sign_fleet(loc_c2)));  %计算交换前后的路径成本
            fun2 = funcopt1(vrp2e,route2(sign_fleet==sign_fleet(loc_c1)))+funcopt1(vrp2e,route2(sign_fleet==sign_fleet(loc_c2)));
            if fun2 < fun1
                route  = route2;
                loc_c1 = find(route==c1);    %重置邻居客户位置，重新开始交换过程
                i     = 1;
            end
            i = i+1;
        end
    end
    %----两个临近点交换----
    for c1 = seq_c
        loc_c1     = find(route==c1);
        [~,pos_in] = ismember(neib_cc(c1-num_sat,2:tao+1),route);  %邻居客户在路径中的位置（不含自身）
        pos_in     = sort(pos_in);
        i          = 1;
        while i < tao
            if pos_in(i+1)-pos_in(i)==1
                fun1 = funcopt1(vrp2e,route(sign_fleet==sign_fleet(loc_c1)))+funcopt1(vrp2e,route(sign_fleet==sign_fleet(pos_in(i))));
                if loc_c1<pos_in(i)  %对于客户c1的每对连续邻居客户
                    route2 = [route(1:loc_c1-1),route(pos_in(i):pos_in(i+1)),route(loc_c1+1:pos_in(i)-1),route(loc_c1),route(pos_in(i+1)+1:end)]; %交换c1和这对连续邻居客户在路径中的位置
                else
                    route2 = [route(1:pos_in(i)-1),route(loc_c1),route(pos_in(i)+2:loc_c1-1),route(pos_in(i):pos_in(i+1)),route(loc_c1+1:end)];
                end
                sign_fleet2 = cumsum(route2<=num_sat,2);                 %卫星和客户对应的车辆
                fun2 = funcopt1(vrp2e,route2(sign_fleet2==sign_fleet2(pos_in(i))))+funcopt1(vrp2e,route2(sign_fleet2==sign_fleet2(loc_c1)));
                if fun2 <fun1
                    route      = route2;
                    sign_fleet = sign_fleet2;
                    loc_c1     = find(route==c1);
                    [~,pos_in] = ismember(neib_cc(c1-num_sat,2:tao+1),route);  %邻居客户在路径中的位置（不含自身）
                    pos_in     = sort(pos_in);
                    i          = 0;
                end
            end
            i = i+1;
        end
    end
end
 

%% ----单路径适应值度量（考虑超重）：首为卫星点,尾为客户点----
function fvrp = funcopt1(vrp2e,route)
    fleet = vrp2e.fleet;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    cap_sc = energy(vrp2e,[route,0]);
    loc_sat = find(route<=num_sat); 
    rback   = route; rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
    route2  = [rback(2:end),route(loc_sat(end))];
    Dis_route = dis_sc(route+(route2-1)*(num_sat+num_cus));
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
    Energy_distribution = Energy_route.*Dis_route./vrp2e.V; 
    Energy_sat = arrayfun(@(i) sum(Energy_distribution(loc_sat(i):loc_sat(i+1)-1)), 1:length(loc_sat)-1);
    Energy_sat = [Energy_sat, sum(Energy_distribution(loc_sat(end):end))];
    penalty  = 100000000;
    fvrp     = sum(Energy_distribution)+penalty*(max(max(cap_sc - fleet(2,1)),0) + max(max(Energy_sat-vrp2e.E),0));
end
%% ----单路径适应值度量（考虑超重）：首尾为卫星点----
function fvrp = funcopt2(vrp2e,route)
    fleet = vrp2e.fleet;
    dis_sc = vrp2e.dis_sc;
    cap_sc = energy(vrp2e,route);  %当前子路段的实时运载重量
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);%当前子路段的实时运载功率
    Dis_route = dis_sc(sub2ind(size(dis_sc), route(1:end-1), route(2:end)));
    Energy_distribution = Energy_route.*Dis_route./vrp2e.V;
    penalty  = 100000000;
    fvrp     = sum(Energy_distribution)+penalty*(max(max(cap_sc - fleet(2,1)),0) + max(sum(Energy_distribution)-vrp2e.E,0));
end

%% ----完整路径适应值度量（考虑超重）：首尾为客户----
function fvrp = funcopt4(vrp2e,route)
    fleet = vrp2e.fleet;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    cap_sc = energy(vrp2e,[route,0]);
    loc_sat = find(route<=num_sat); 
    rback   = route; rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
    route2  = [rback(2:end),route(loc_sat(end))];
    Dis_route = dis_sc(route+(route2-1)*(num_sat+num_cus));
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
    Energy_distribution = Energy_route.*Dis_route./vrp2e.V; 
    Energy_sat = arrayfun(@(i) sum(Energy_distribution(loc_sat(i):loc_sat(i+1)-1)), 1:length(loc_sat)-1);
    Energy_sat = [Energy_sat, sum(Energy_distribution(loc_sat(end):end))];
    penalty  = 100000000;
    fvrp     = sum(Energy_distribution)+penalty*(max(max(cap_sc - fleet(2,1)),0) + max(max(Energy_sat-vrp2e.E),0));
end