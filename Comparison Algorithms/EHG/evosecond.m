%% --------------------------第二层路径优化-------------------------
function seq_cs = evosecond1(vrp2e,seq_cs,sm,opt_cus_pre)
    %global dis_sc; global neib_cc; global num_sat; global num_cus;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;

    seq_cs_pre = seq_cs;
    ndl               = ceil(rand*min(60,0.4*num_cus));  %随机删除的客户数目
    if sm == 1
        %----随机删除临近的ndl个客户并贪婪插入----
        neib_cc = vrp2e.neib_cc;
        num_sat = vrp2e.num_sat;
        num_cus = vrp2e.num_cus;
        ndl           = ceil(rand*0.3*num_cus);       %随机删除的客户数目
        rc            = ceil(rand*num_cus);          %随机选取一个客户点
        seq_c         = neib_cc(rc,1:ndl);           %选取与客户邻近的ndl个客户 
        [~,loc_c]     = ismember(seq_c,seq_cs);      %多个客户在路径中的位置
        seq_cs(loc_c) = [];
        len_route     = numel(seq_cs);                               %已有的路径长度
        loc_sat       = [find(seq_cs<=num_sat),len_route+1];         %卫星在路径中的位置
        seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
    elseif sm == 2
        %----随机删除ndl个客户并贪婪插入----
        seq_c          = randperm(num_cus,ndl)+num_sat;
        [~,loc_c]      = ismember(seq_c,seq_cs);     %多个客户在路径中的位置
        seq_cs(loc_c)   = [];                         %从路径中删除客户
        len_route      = numel(seq_cs);                              %已有的路径长度
        loc_sat        = [find(seq_cs<=num_sat),len_route+1];             %卫星在路径中的位置
        seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
    elseif sm == 3
        seq_cs(find(seq_cs ~= 0, 1, 'last')+1:end) = [];
        num_sat = vrp2e.num_sat;
        loc_cus       = find(seq_cs>num_sat);           %客户在路径中的位置
        ndl           = ceil(rand*length(loc_cus));  %随机出即将要删掉的客户数
        cost_route = funcopt3(vrp2e,seq_cs);
        no_cus = arrayfun(@(i) [seq_cs(1:i-1), seq_cs(i+1:end)], loc_cus, 'UniformOutput', false);
        cost_cus = arrayfun(@(i) cost_route-funcopt3(vrp2e,no_cus{i}), 1:length(no_cus), 'UniformOutput', false);
        cost_cus = cell2mat(cost_cus);
        del_c         = zeros(1,ndl);
        for i = 1:ndl
            cl           = roulette(cost_cus,1);
            del_c(i)     = loc_cus(cl);
            cost_cus(cl) = 0;
        end
        seq_c     = seq_cs(del_c);
        seq_cs(del_c) = [];
    else
        num_sat = vrp2e.num_sat;
        if rand < 0.4
            len_route = length(seq_cs);
            loc_s     = fliplr([find(seq_cs<=num_sat),len_route+1]);    %卫星在解中对应的位置
            nr        = numel(loc_s)-1;
            seq_c     = zeros(1,len_route);
            for i = 1:nr
                if loc_s(i+1)-loc_s(i)==2
                    seq_c(loc_s(i)+1:loc_s(i+1)-1) = seq_cs(loc_s(i)+1:loc_s(i+1)-1);
                    seq_cs(loc_s(i):loc_s(i+1)-1)  = [];
                end
            end
            seq_c(seq_c==0)  = [];
        else
            seq_c = [];
        end
    end
    seq_cs = greedyin2(vrp2e,seq_cs,seq_c);
    [vrp_cs,~] = msecond(vrp2e,seq_cs);
    if ~isequal(seq_cs, opt_cus_pre)
        [seq_cs_temp, opt_cus_pre_temp, seq_cs_waite, opt_cus_pre_waite] = crossover(seq_cs, opt_cus_pre, num_sat);
%         seq_cs_temp = [seq_cs_temp, zeros(1, numel(seq_cs) - numel(seq_cs_temp))];
        if ~isempty(seq_cs_waite)
            seq_cs_temp = greedyin2(vrp2e,seq_cs_temp,seq_cs_waite);
        end
%         opt_cus_pre_temp = [opt_cus_pre_temp, zeros(1, numel(seq_cs) - numel(opt_cus_pre_temp))];
        if ~isempty(opt_cus_pre_waite)
            opt_cus_pre_temp = greedyin2(vrp2e,opt_cus_pre_temp,opt_cus_pre_waite);
        end
        [vrp_cs_temp,vlt_cs] = msecond(vrp2e,seq_cs_temp);
        [vrp_opt,vlt_opt] = msecond(vrp2e,opt_cus_pre_temp);
        if vlt_cs<=0 && vlt_opt<=0
            if vrp_cs_temp<vrp_opt && vrp_cs_temp<vrp_cs
                seq_cs = seq_cs_temp;
                vrp_cs = vrp_cs_temp;
            elseif vrp_opt<vrp_cs_temp && vrp_opt<vrp_cs
                seq_cs = opt_cus_pre_temp;
                vrp_cs = vrp_opt;
            end
        end
    end
    if ~isequal(sort(seq_cs(seq_cs>num_sat)), num_sat+1:num_sat+num_cus)
        setdiff(sort(seq_cs(seq_cs>num_sat)),num_sat+1:num_sat+num_cus)
    end
    if ~isequal(sort(seq_cs_new(seq_cs_new>num_sat)), num_sat+1:num_sat+num_cus)
        sort(seq_cs_new(seq_cs_new>num_sat))
    end
    [vrp_cs_new,vlt_cs_new] = msecond(vrp2e,seq_cs_new);
    if vlt_cs<=0 && vlt_cs_new<=0
        if vrp_cs<vrp_cs_new
            seq_cs = seq_cs_pre;
        else
            seq_cs = seq_cs_new;
        end
    else
        seq_cs = seq_cs_pre;
    end
end

function [seq_cs_temp, opt_cus_temp, seq_cs_waite, opt_cus_waite] = crossover(seq_cs, opt_cus, num_sat)
    cs_sat = unique(seq_cs(seq_cs<=num_sat&seq_cs>0));
    opt_sat = unique(opt_cus(opt_cus<=num_sat&opt_cus>0));
    selected_cs = cs_sat(randi(length(cs_sat)));    %seq_cs待替换卫星
    selected_opt = opt_sat(randi(length(opt_sat))); %opt_cus待替换卫星
    
    sat_cs = arrayfun(@(x) find(seq_cs == x), cs_sat, 'UniformOutput', false);
    sat_cs = [sat_cs{:}];
    sat_cs = sort(sat_cs);      %找出seq_cs所有卫星下标

    sat_opt = arrayfun(@(x) find(opt_cus == x), opt_sat, 'UniformOutput', false);
    sat_opt = [sat_opt{:}];
    sat_opt = sort(sat_opt);    %找出opt_cus所有卫星下标


    seq_cs_temp = seq_cs;
    idx1 = find(seq_cs(sat_cs)==selected_cs);  %找出所有待替换卫星的下标
    next_idx1 = idx1+1;
    for i = 1:length(idx1)
        if next_idx1(i) <= length(sat_cs)
            seq_cs_temp(sat_cs(idx1(i)):sat_cs(next_idx1(i))-1)=0;
        else
            seq_cs_temp(sat_cs(idx1(i)):end)=0;
        end
    end
    
    opt_cus_customers = [];
    idx2 = find(opt_cus(sat_opt)==selected_cs);
    next_idx2 = idx2+1;
    for i = 1:length(idx2)
        if next_idx2(i) <= length(sat_opt)
            opt_cus_customers = [opt_cus_customers, opt_cus(sat_opt(idx2(i)):sat_opt(next_idx2(i))-1)]; 
        else
            opt_cus_customers = [opt_cus_customers, opt_cus(sat_opt(idx2(i)):end)]; 
        end
    end

    opt_cus_temp = opt_cus;
    idx3 = find(opt_cus(sat_opt)==selected_opt);
    next_idx3 = idx3+1;
    for i = 1:length(idx3)
        if next_idx3(i) <= length(sat_opt)
            opt_cus_temp(sat_opt(idx3(i)):sat_opt(next_idx3(i))-1)=0;
        else
            opt_cus_temp(sat_opt(idx3(i)):end)=0;
        end
    end

    seq_cs_customers = [];
    idx4 = find(seq_cs(sat_cs)==selected_opt);
    next_idx4 = idx4+1;
    for i = 1:length(idx4)
        if next_idx4(i) <= length(sat_cs)
            seq_cs_customers = [seq_cs_customers, seq_cs(sat_cs(idx4(i)):sat_cs(next_idx4(i))-1)]; 
        else
            seq_cs_customers = [seq_cs_customers, seq_cs(sat_cs(idx4(i)):end)]; 
        end
    end

    seq_cs_temp = [seq_cs_temp,opt_cus_customers];
    opt_cus_temp = [opt_cus_temp,seq_cs_customers];

    cs_cus = seq_cs_temp(seq_cs_temp > num_sat);
    [unique_elements, ~, idx] = unique(cs_cus); % 获取唯一元素及其索引
    counts = accumarray(idx, 1); % 统计每个元素的出现次数
    seq_cs_waite = unique_elements(counts > 1); % 找出重复的元素
    seq_cs_temp(ismember(seq_cs_temp, seq_cs_waite)) = 0;

    op_cus = opt_cus_temp(opt_cus_temp > num_sat);
    [unique_elements, ~, idx] = unique(op_cus); % 获取唯一元素及其索引
    counts_opt = accumarray(idx, 1); % 统计每个元素的出现次数
    opt_cus_waite = unique_elements(counts_opt > 1); % 找出重复的元素
    opt_cus_temp(ismember(opt_cus_temp, opt_cus_waite)) = 0;

    seq_cs_temp(seq_cs_temp == 0) = [];
    opt_cus_temp(opt_cus_temp == 0) = [];

    % 将 seq_cs_temp_unique 与 seq_cs 对比，找到 seq_cs_temp_unique 中没有的且大于 num_sat 的元素
    missing_cs = setdiff(seq_cs, seq_cs_temp);  % 找到 seq_cs_temp_unique 中没有的元素
    missing_cs = missing_cs(missing_cs > num_sat);  % 筛选出大于 num_sat 的元素
    missing_opt = setdiff(opt_cus, opt_cus_temp);  % 找到 seq_cs_temp_unique 中没有的元素
    missing_opt = missing_opt(missing_opt > num_sat);  % 筛选出大于 num_sat 的元素

    seq_cs_waite = unique([seq_cs_waite, missing_cs]);  % 去重并合并到 A 中
    opt_cus_waite = unique([opt_cus_waite, missing_opt]);  % 去重并合并到 A 中
end



%% ----贪婪插入算法----
%将seq_c中的客户插入到已有的队列seq_cs中，s_off：关闭或禁止的卫星
function seq_r = greedyin2(vrp2e,route,seq_c)
    route(find(route ~= 0, 1, 'last')+1:end) = [];
    fleet = vrp2e.fleet;  %获取车队信息，如每辆车的最大容量等。
    demand = vrp2e.demand; %获取客户的需求量
    dis_sc = vrp2e.dis_sc;  %卫星与客户之间的距离 
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;
    cus_type = vrp2e.type;
    E = vrp2e.E;  %电池总功率
    V = vrp2e.V;    %无人机飞行速度
    penalty = 10000000;  %设置一个惩罚值，用于约束条件的违规情况
    %----计算当前路径中已分配的车辆数量和其累积装载量----
    num_fleet     = sum(route<=num_sat);                    %计算当前已分配的车辆数量
    cap_s         = zeros(1,2*num_cus);
    nums_c           = num_sat+num_cus;
    len_route     = numel(route);
    seq_r         = zeros(1,2*num_cus); %卫星-客户对应关系记录
    seq_r(1:len_route) = route;

    %----贪婪插入----
    for c = seq_c    %根据距离和约束条件，将每个客户插入到路径 route 中的合适位置
        cost_newr        = 2*dis_sc(c,1:num_sat);   %当前客户分别与所有卫星的距离
        [cost,s]         = min(cost_newr);          % 找出当前客户应该插入的最优位置
        if len_route > 0 
            route      = seq_r(1:len_route);        %已有路径
            sign_fleet = cumsum(route<=num_sat,2);  %卫星和客户对应的无人机（按行求累积和）
%             cap_sc     = cap_s(sign_fleet);         %每一个位置对应的容量(这个公式需要调整)
            loc_sat    = find(route<=num_sat);      %卫星在路径中的位置
             % 将卫星和车辆的相应路径记录
            rback      = route; 
            rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
            route2     = [rback(2:end),route(loc_sat(end))];    %返回卫星的路径
            cap_sc = cap_s(1:len_route);
            power = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);

%           %计算当前已有路径的总能耗
            Energy_route = power.* dis_sc(route+(route2-1)*nums_c)./V;
            Energy = [arrayfun(@(i)[sum(Energy_route(loc_sat(i):loc_sat(i+1)-1))], 1:length(loc_sat)-1, 'UniformOutput', false),sum(Energy_route(loc_sat(end):end))];  %计算原路径的总能耗
            Energy = arrayfun(@(x) Energy(x), sign_fleet);
            Energy = cell2mat(Energy);
            %根据取-送情况分别计算插入新增用户后可能的代价, 无人机载重+续航能力和数量是否超过约束
            if strcmp(cus_type(c-num_sat), 'pickup')
                W_end = arrayfun(@(n) [cap_sc(loc_sat(end):n-1), demand(c) + cap_sc(n-1), cap_sc(n:end)+demand(c)], loc_sat(end)+1:length(cap_sc)+1, 'UniformOutput', false);
                W_all = [];
                for i = 1:length(loc_sat)-1
                    cap_pre = cap_sc(1:loc_sat(i)-1);
                    cap_i = cap_sc(loc_sat(i):loc_sat(i+1)-1);
                    cap_after = cap_sc(loc_sat(i+1):end);
%                     diff_i = diff_all{i};
                    W_i = arrayfun(@(n) [cap_i(1:n-1), demand(c) + cap_i(n-1), cap_i(n:end)+demand(c)], 2:length(cap_i)+1, 'UniformOutput', false);
                    W1 = cellfun(@(x) [cap_pre, x, cap_after], W_i, 'UniformOutput', false);
                    W_all = [W_all,W1];
                end
                cap_p = cap_sc(1:loc_sat(end)-1);
                W2 = cellfun(@(x) [cap_p, x], W_end, 'UniformOutput', false);
                W_distribution = [W_all,W2];
                cost = (5).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4)*cost./V;
            else
                %插入到哪个位置，该位置之前的cap_sc+demand(c)，之后的不加
%                 arrayfun(@(n) [cap_sc(1:n-1)+demand(c), cap_sc(n-1), cap_sc(n:end)], 2:length(cap_sc)+1, 'UniformOutput', false)
                W_end = arrayfun(@(n) [cap_sc(loc_sat(end):n-1)+demand(c), cap_sc(n-1), cap_sc(n:end)], loc_sat(end)+1:length(cap_sc)+1, 'UniformOutput', false);
                W_all = [];
                for i = 1:length(loc_sat)-1
                    cap_pre = cap_sc(1:loc_sat(i)-1);
                    cap_i = cap_sc(loc_sat(i):loc_sat(i+1)-1);
                    cap_after = cap_sc(loc_sat(i+1):end);
                    W_i = arrayfun(@(n) [cap_i(1:n-1)+demand(c), cap_i(n-1), cap_i(n:end)], 2:length(cap_i)+1, 'UniformOutput', false);
                    W1 = cellfun(@(x) [cap_pre, x, cap_after], W_i, 'UniformOutput', false);
                    W_all = [W_all,W1];
                end
                cap_p = cap_sc(1:loc_sat(end)-1);
                W2 = cellfun(@(x) [cap_p, x], W_end, 'UniformOutput', false);
                W_distribution = [W_all,W2];
                cost = (5+demand(c)).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4)*cost./V;
            end
            Max_W=cellfun(@(x) max(x(1:end)), W_distribution);  %获取所有插入情况的最大值（超重限制阈值）
            route_c = arrayfun(@(pos) [route(1:pos-1), c, route(pos:end)], 2:length(route)+1, 'UniformOutput', false);  %前段路径
            loc_sat_c = cellfun(@(x) find(x <= num_sat), route_c, 'UniformOutput', false);    %添加新客户后的卫星坐标
            rback = cellfun(@(x) x, route_c, 'UniformOutput', false);
            for i = 1:length(route_c)% 更新 rback 中的每个 cell
                rback{i}(loc_sat_c{i}(2:end)) = route_c{i}(loc_sat_c{i}(1:end-1));
                route_c2{i} = [rback{i}(2:end),route_c{i}(loc_sat_c{i}(end))];
            end% 
%             route_c2 = cellfun(@(x) [x(2:end), x(loc_sat(end))], arrayfun(@(pos) [route(1:pos-1), c, route(pos:end)], 2:length(route)+1, 'UniformOutput', false), 'UniformOutput', false);%后段路径
            dis_route = cellfun(@(x, y) dis_sc(x + (y - 1) * nums_c), route_c, route_c2, 'UniformOutput', false);%完整路径对应的距离长度
            power_route = cellfun(@(W) (5+W).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4), W_distribution, 'UniformOutput', false);   %每个路段的功率
            Energy_distribution = cellfun(@(p, d) p.*d./V, power_route, dis_route, 'UniformOutput', false);  %插入不同位置的不同总功耗
            
            group = [];
            if length(loc_sat)>1
                for i=1:length(loc_sat)-1
                    group{i} = loc_sat(i):loc_sat(i+1)-1;
                end
                group{length(group)+1} = loc_sat(end):length(Energy_distribution);
            else
                group = {1:length(Energy_distribution)};
            end
            
            for i = 1:length(group)
                if i<length(group)
                    for j = group{i}
                        Energy_c {j} = sum(Energy_distribution{j}(loc_sat_c{j}(i):loc_sat_c{j}(i+1)-1));
                    end
                else
                    for j = group{i}
                        Energy_c {j} = sum(Energy_distribution{j}(loc_sat_c{j}(end):end));
                    end
                end
            end
            punish = [penalty * (max(Max_W-fleet(2,1),0)+max(cell2mat(Energy_c)-E,0)),penalty * max(num_fleet+1-fleet(2,2),0)];
            %             dis_sc(route+(route2-1)*nsc)
            cost_c     = [cell2mat(Energy_c)-Energy,cost*10]+punish;
            [~,loc_in] = min(cost_c);  % 找出最小代价的插入位置
            if loc_in <= len_route  % 如果插入位置在当前路径范围内
                % 在插入位置后移动现有路径中的客户
                seq_r(loc_in+2:len_route+1) = seq_r(loc_in+1:len_route);
                seq_r(loc_in+1) = c;
                len_route       = len_route+1;
                cap_s(1:numel(W_distribution{loc_in})) = W_distribution{loc_in};% 已有车辆的装载量更新 
            else
                % 如果插入位置超出当前路径范围，则将客户插入新的车辆中
                seq_r(len_route+1:len_route+2) = [s,c];
                if strcmp(cus_type(c-num_sat), 'delivery')
                    cap_s(len_route+1) = demand(c); % 记录新车辆的装载量
                else
                    cap_s(len_route+2) = demand(c);
                end
                len_route = len_route+2;
                num_fleet = num_fleet+1; % 车辆数量加一
            end
        else
            cap_s         = zeros(1,2*num_cus);                  %初始化路段实时重量
            seq_r(1:2) = [s,c]; %记录当前客户与卫星的对应关系
            len_route  = 2;
            num_fleet  = num_fleet+1;   %占用的无人机数量+1
            if strcmp(cus_type(c-num_sat), 'delivery')
                cap_s(1)   = demand(c);
            else
                cap_s(2)   = demand(c);
            end
        end
    end
    seq_r(len_route+1:end) = []; % 删除多余的空白部分
end
function fvrp = funcopt3(vrp2e,route)
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    loc_sat = find(route<=num_sat); 
    rback   = route; rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
    route2  = [rback(2:end),route(loc_sat(end))];
    cap_sc = energy(vrp2e,[route,route2(end)]);
    Dis_route = dis_sc(route+(route2-1)*(num_sat+num_cus));
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
    Energy_distribution = Energy_route.*Dis_route./vrp2e.V; 
    fvrp = sum(Energy_distribution);
end