%% --------------------------第二层路径优化-------------------------
function seq_r = repair2(vrp2e,seq_cs,seq_c,s_off,sm)
    if sm < 4  %(9-11)
        seq_r = greedyin2(vrp2e,seq_cs,seq_c,s_off);  %sm1=1为贪婪插入；2为扰动贪婪插入；3为禁止重复贪婪插入
    else
        seq_r = greedyinr2(vrp2e,seq_cs,seq_c,s_off);    %后悔插入
    end
end



%% ----贪婪插入算法----
%将seq_c中的客户插入到已有的队列seq_cs中，s_off：关闭或禁止的卫星
function seq_r = greedyin2(vrp2e,route,seq_c,s_off)
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
    num_fleet     = sum(route<=num_sat);
    cap_s         = zeros(1,2*num_cus);                  %初始化路段实时重量
    nums_c           = num_sat+num_cus;
    len_route     = numel(route);
    seq_r         = zeros(1,2*num_cus); %卫星-客户对应关系记录
    seq_r(1:len_route) = route;
    
    %----贪婪插入----
    for c = seq_c    %根据距离和约束条件，将每个客户插入到路径 route 中的合适位置
        cost_newr        = 2*dis_sc(c,1:num_sat);   %当前客户分别与所有卫星的距离
        cost_newr(s_off) = inf;                     %将要关闭的卫星距离设置为无限大
        [cost,s]         = min(cost_newr);          % 找出当前客户应该插入的最优位置
        if len_route > 0 
            route      = seq_r(1:len_route);        %已有路径
            sign_fleet = cumsum(route<=num_sat,2);  %卫星和客户对应的无人机（按行求累积和）
            loc_sat    = find(route<=num_sat);      %卫星在路径中的位置
             % 将卫星和车辆的相应路径记录
            rback      = route; 
            rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
            route2     = [rback(2:end),route(loc_sat(end))];    %返回卫星的路径
            cap_sc = energy(vrp2e,[route,0]);
            power = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);

%           %计算当前已有路径的总能耗
            Energy_route = power.* dis_sc(route+(route2-1)*nums_c)./V;
            Energy = [arrayfun(@(i)[sum(Energy_route(loc_sat(i):loc_sat(i+1)-1))], 1:length(loc_sat)-1, 'UniformOutput', false),sum(Energy_route(loc_sat(end):end))];  %计算原路径的总能耗
            Energy = arrayfun(@(x) Energy(x), sign_fleet);
            Energy = cell2mat(Energy);
            %根据取-送情况分别计算插入新增用户后可能的代价, 无人机载重+续航能力和数量是否超过约束
            if strcmp(cus_type(c-num_sat), 'pickup')
                %插入到哪个位置，该位置之后的cap_sc+demand(c),之前的不加
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
            cap_s         = zeros(1,2*num_cus);
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



%% ----后悔插入----
function seq_r = greedyinr2(vrp2e, route, seq_c, s_off)
    fleet = vrp2e.fleet;
    demand = vrp2e.demand;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;
    cus_type = vrp2e.type;
    E = vrp2e.E;
    V = vrp2e.V;
    penalty = 10000000;

    % 计算车辆负载和路径初始化
    cap_s = zeros(1, 2 * num_cus);
    nums_c = num_sat + num_cus;
    len_route = numel(route);
    seq_r = zeros(1, 2 * num_cus);
    seq_r(1:len_route) = route;
    num_c = numel(seq_c);
    num_fleet = sum(route <= num_sat);

    while num_c > 0
        % 后悔插入的成本计算
        cost_newr = 2 * dis_sc(seq_c, 1:num_sat);
        cost_newr(:, s_off) = inf;
        [cost, s_cus] = min(cost_newr, [], 2);

        if len_route > 0
            % 路径调整与后悔插入
            route = seq_r(1:len_route);
            sign_fleet = cumsum(route <= num_sat, 2);
            loc_sat = find(route <= num_sat);
            rback = route;
            rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
            route2 = [rback(2:end), route(loc_sat(end))];
            cap_sc = energy(vrp2e, [route, 0]);
            power = (5 + cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
            Energy_route = power .* dis_sc(route + (route2 - 1) * nums_c) ./ V;

            % 计算路径总能耗
            Energy = [arrayfun(@(i) sum(Energy_route(loc_sat(i):loc_sat(i + 1) - 1)), 1:length(loc_sat) - 1), sum(Energy_route(loc_sat(end):end))];
            Energy = arrayfun(@(x) Energy(x), sign_fleet);
            if strcmp(cus_type(seq_c(num_c)-num_sat), 'pickup')
                W_end = arrayfun(@(n) [cap_sc(loc_sat(end):n-1), demand(seq_c(num_c)) + cap_sc(n-1), cap_sc(n:end)+demand(seq_c(num_c))], loc_sat(end)+1:length(cap_sc)+1, 'UniformOutput', false);
                W_all = [];
                for i = 1:length(loc_sat)-1
                    cap_pre = cap_sc(1:loc_sat(i)-1);
                    cap_i = cap_sc(loc_sat(i):loc_sat(i+1)-1);
                    cap_after = cap_sc(loc_sat(i+1):end);
%                     diff_i = diff_all{i};
                    W_i = arrayfun(@(n) [cap_i(1:n-1), demand(seq_c(num_c)) + cap_i(n-1), cap_i(n:end)+demand(seq_c(num_c))], 2:length(cap_i)+1, 'UniformOutput', false);
                    W1 = cellfun(@(x) [cap_pre, x, cap_after], W_i, 'UniformOutput', false);
                    W_all = [W_all,W1];
                end
                cap_p = cap_sc(1:loc_sat(end)-1);
                W2 = cellfun(@(x) [cap_p, x], W_end, 'UniformOutput', false);
                W_distribution = [W_all,W2];
                cost = (5).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4)*cost./V;
            else
                W_end = arrayfun(@(n) [cap_sc(loc_sat(end):n-1)+demand(seq_c(num_c)), cap_sc(n-1), cap_sc(n:end)], loc_sat(end)+1:length(cap_sc)+1, 'UniformOutput', false);
                W_all = [];
                for i = 1:length(loc_sat)-1
                    cap_pre = cap_sc(1:loc_sat(i)-1);
                    cap_i = cap_sc(loc_sat(i):loc_sat(i+1)-1);
                    cap_after = cap_sc(loc_sat(i+1):end);
                    W_i = arrayfun(@(n) [cap_i(1:n-1)+demand(seq_c(num_c)), cap_i(n-1), cap_i(n:end)], 2:length(cap_i)+1, 'UniformOutput', false);
                    W1 = cellfun(@(x) [cap_pre, x, cap_after], W_i, 'UniformOutput', false);
                    W_all = [W_all,W1];
                end
                cap_p = cap_sc(1:loc_sat(end)-1);
                W2 = cellfun(@(x) [cap_p, x], W_end, 'UniformOutput', false);
                W_distribution = [W_all,W2];
                cost = (5+demand(seq_c(num_c))).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4)*cost./V;
            end
            Max_W=cellfun(@(x) max(x(1:end)), W_distribution);  %获取所有插入情况的最大值（超重限制阈值）
            route_c = arrayfun(@(pos) [route(1:pos-1), seq_c(num_c), route(pos:end)], 2:length(route)+1, 'UniformOutput', false);  %前段路径
            loc_sat_c = cellfun(@(x) find(x <= num_sat), route_c, 'UniformOutput', false);    %添加新客户后的卫星坐标
            rback = cellfun(@(x) x, route_c, 'UniformOutput', false);
            for i = 1:length(route_c)% 更新 rback 中的每个 cell
                rback{i}(loc_sat_c{i}(2:end)) = route_c{i}(loc_sat_c{i}(1:end-1));
                route_c2{i} = [rback{i}(2:end),route_c{i}(loc_sat_c{i}(end))];
            end% 
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
            cost_c = [cell2mat(Energy_c)-Energy,cost(num_c)*10] + punish;

            [val_cost, srt_c] = sort(cost_c, 2);
            [~, c_sel] = max(sum(val_cost(:, 2:3), 2) - 2 * val_cost(:, 1));
            loc_in = srt_c(c_sel, 1);
            c = seq_c(c_sel);
            % 插入新的客户
            if loc_in <= len_route
                seq_r(loc_in + 2:len_route + 1) = seq_r(loc_in + 1:len_route);
                seq_r(loc_in + 1) = c;
                len_route = len_route + 1;
                cap_s(sign_fleet(loc_in)) = cap_s(sign_fleet(loc_in)) + demand(c);
            else
                % 如果插入位置超出当前路径范围，则将客户插入新的车辆中
                seq_r(len_route+1:len_route+2) = [s_cus(c_sel),c];
                if strcmp(cus_type(c-num_sat), 'delivery')
                    cap_s(len_route+1) = demand(c); % 记录新车辆的装载量
                else
                    cap_s(len_route+2) = demand(c);
                end
                len_route = len_route+2;
                num_fleet = num_fleet+1; % 车辆数量加一
            end
        else
            [~, c_sel] = max(cost);
            c = seq_c(c_sel);
            cap_s         = zeros(1,2*num_cus);                  %初始化路段实时重量
            seq_r(1:2) = [s_cus(c_sel),c]; %记录当前客户与卫星的对应关系
            len_route  = 2;
            num_fleet  = num_fleet+1;   %占用的无人机数量+1
            if strcmp(cus_type(c-num_sat), 'delivery')
                cap_s(1)   = demand(c);
            else
                cap_s(2)   = demand(c);
            end
        end
        % 移除已插入客户
        seq_c(c_sel) = [];
        num_c = num_c - 1;
    end
    seq_r(len_route + 1:end) = [];
end
