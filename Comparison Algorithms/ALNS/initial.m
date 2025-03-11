%% --------------------------初始化第二层--------------------------
%第二层初始化方式：
%step1: 按照客户对卫星的距离将客户分给卫星
%step2: 将每个卫星中的客户按照CW策略进行排序
%step3: 将第一层按CW策略进行排序
function [seq_cus,seq_sat,cap_sat] = initial(vrp2e)
    fleet = vrp2e.fleet;
    demand = vrp2e.demand; 
    dis_ds = vrp2e.dis_ds; 
    dis_sc = vrp2e.dis_sc;  
    num_dep = length(vrp2e.rad_ds(:,1));
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    seq_cus   = zeros(1,2*(num_cus+fleet(2,2)));                      %预分配存储客户容量为长度2*nc，超出空间删除
    seq_sat   = zeros(1,2*fleet(1,2)+2*num_sat);         %预分配存储卫星序号2*fleet(1,2)+2*ns，超出空间删除
    cap_sat   = zeros(1,2*fleet(1,2)+2*num_sat);         %预分配存储卫星容量2*fleet(1,2)+2*ns，超出空间删除
   %% -=--初始化第二层----
    %----按照距离采用轮盘赌将客户分配给卫星----
    sat_cus = zeros(1,num_cus);              %存储每个客户归属的卫星
    for c = 1:num_cus
        sat_cus(c) = roulette(1./dis_sc(1:num_sat,c+num_sat),1);
    end
    %----提出卫星客户并按照CW算法排序
    cap_s = zeros(1,num_sat);
    nrs   = 0;   %路径总长度
    for s = 1:num_sat
        cus_s                 = find(sat_cus==s)+num_sat;
        cap_s(s)              = sum(demand(cus_s));
        route                 = algcw2(vrp2e,s,cus_s);
        nr                     = numel(route);
        seq_cus(nrs+1:nrs+nr) = route;
        nrs                   = nrs+nr;
        seq_cus(nrs+1:end)    = [];
    end
    %% -=--初始化第一层----
    %----按照距离采用轮盘赌将卫星分配给仓库----
    dep_sat = zeros(1,num_sat);              %存储每个客户归属的卫星
    for c = 1:num_sat
        dep_sat(c) = roulette(1./dis_sc(1:num_dep,c+num_dep),1);
    end
    nrs   = 0;   %路径总长度
    for s = 1:num_sat
        while cap_s(s)>=fleet(1,1)
            nrs          = nrs+2;
            seq_sat(nrs) = s;
            cap_sat(nrs) = fleet(1,1);
            cap_s(s)     = cap_s(s)-fleet(1,1);
        end
    end
    for s = 1:num_dep
%         cap_d(s)              = sum(cap_s(find(dep_sat==s)));
        dep_s                 = find(dep_sat==s)+num_dep;
        route                 = algcw1(dis_ds(s,dep_s),dis_ds(dep_s,dep_s),cap_s,fleet(1,1));
        loc                   = find(route>0);
        route(loc)            = dep_s(route(loc));
        cap_r                 = route;
        cap_r(loc)            = cap_s(route(loc)-num_dep);
        route(route==0)       = s;
        nr                    = numel(route);
        seq_sat(nrs+1:nrs+nr) = route;
        cap_sat(nrs+1:nrs+nr) = cap_r;
        nrs                   = nrs+nr;
        seq_sat(nrs+1:end)    = [];
        cap_sat(nrs+1:end)    = [];
    end
end

 %% --------------将客户初排序(基于CW)，客户货物量不可拆分--------------------
%dis_stoc:客户到卫星的距离; %dis_ctoc:客户到客户的距离; %demand_cus:客户的需求demand; %capcity:车载最大容量; %seq_c中的0为卫星编号
function rount = algcw2(vrp2e,s,cus_s)
    V = vrp2e.V;
    E = vrp2e.E;
    num_sat = vrp2e.num_sat;
    demand = vrp2e.demand;
    capacity = vrp2e.fleet(2,1);
    dis_sc = vrp2e.dis_sc;
    demand_cus = demand(cus_s);
    dis_stoc = dis_sc(s,cus_s);
    c_type = vrp2e.type;
    c_type = c_type(cus_s-num_sat);
    nc         = numel(dis_stoc);     %客户总数
    link       = zeros(2,nc);         %客户连接的其它客户：存储与本客户连接的其它客户
    nlink      = zeros(1,nc);         %客户连接的次数：用于识别该客户是否还可连接
    seq_veh    = 1:nc;                %客户对应的车辆序号：用于计算车辆的载货量
    cost_stoc = (5+5+demand_cus).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4).*dis_stoc./V;
    c_rou = arrayfun(@(i) [s,cus_s(i),s],1:length(cus_s), 'UniformOutput', false);
    cap_veh = cellfun(@(i)energy(vrp2e,i),c_rou, 'UniformOutput', false);
    rou = arrayfun(@(i, j) [s,cus_s(i), cus_s(j),s], repmat((1:length(cus_s))', 1, length(cus_s)), repmat(1:length(cus_s), length(cus_s), 1), 'UniformOutput', false);
    cap_ctoc = cellfun(@(i)energy(vrp2e,i),rou, 'UniformOutput', false);
    dis_rou = cellfun(@(route) arrayfun(@(i) dis_sc(route(i), route(i+1)), 1:length(route)-1), rou, 'UniformOutput', false);
    Energy_c = cellfun(@(i,j)sum((5+i)*(3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4).*(j/V)), cap_ctoc,dis_rou, 'UniformOutput', false);
    cw_cc      = reshape(zeroDiagonal(bsxfun(@plus,cost_stoc,cost_stoc'))-zeroDiagonal(cell2mat(Energy_c))+0.01,1,[]);  %计算节省的费用
    [~,srt_cp] = sort(cw_cc,'descend');    %将节省量进行排序
    npair      = nc*(nc-1);    %所有的配对
    srt_cp     = srt_cp(1:npair);          %提取上三角矩阵的序号
    cr         = mod(srt_cp-1,nc)+1;  %排序值对应的行(行与列对应的是连接关系)
    cl         = ceil(srt_cp/nc);     %排序值对应的列
    sorted_pairs = sort([cr(:) cl(:)], 2);  % 将每个配对中的元素排序，以便识别重复的元素对
    unique_idx = true(1, length(cr)); % 初始化用于保存唯一对的索引
    rount   = zeros(1,2*nc+1);          %最长可能返回排序序列
    % 遍历每个元素对，找到重复的元素对，并标记为删除
    for i = 1:length(cr)-1
        for j = i+1:length(cr)
            if isequal(sorted_pairs(i,:), sorted_pairs(j,:))
                unique_idx(j) = false;
            end
        end
    end
    % 使用唯一索引提取保留的元素
    cr = cr(unique_idx);
    cl = cl(unique_idx);
%     [cr, cl] = adjust_indices(cr, cl);
    % 初始化循环的索引
    l = 1;
    % 循环遍历数组
    while l <= length(cr)
        % 获取当前 cr 和 cl 中的元素
        current_cr = cr(l);
        current_cl = cl(l);
        % 找到所有与当前 cr(i) 或 cl(i) 相同的元素索引
        indices_to_remove = (cr == current_cr) | (cl == current_cl);
        % 保留当前元素，但删除其他相同的元素
        indices_to_remove(l) = false;  % 保留当前元素，避免将其删除
        % 删除这些索引处的元素
        cr(indices_to_remove) = [];
        cl(indices_to_remove) = [];
        % 移动到下一个元素
        l = l + 1;
    end
    [cr, cl] = adjust_indices(cr, cl);
    [cr, cl] = remove_conflicting_elements(cr, cl);
    B = setdiff(1:nc, [cr cl]);
    % 逐一将B中的元素添加到cr和cl中
    for i = 1:length(B)
        if ~isempty(cr)
            cl = [cr(1),cl]; 
            cr = [B(i),cr];
        else
            cr = B(i);
            cl = B(i);
        end
    end
    seq_cus = zeros(1,2*nc+1);
    if isempty(cr)
        rount = [];
    else
        seq_cus(1:2) = [s,cus_s(cr(1))];
    end
    k = 1;
    j = 0;
    for i = 1:length(cr)
        nv1 = seq_veh(cr(i));                %提取客户1对应的车辆序号
        nv2 = seq_veh(cl(i));                %提取客户2对应的车辆序号 nv1<nv2
        if strcmp(c_type(cl(i)), 'pickup')
            cap = [cap_veh{cr(i)},cap_veh{cr(i)}(end)+demand_cus(cl(i))];
        else
            cap = [cap_veh{cr(i)}+demand_cus(cl(i)),cap_veh{cr(i)}(end)];
        end
        if i > 1
            if cr(i) == cl(i-1)   %依然连续(超载后、非超载均可)
                route_cap = [s,seq_cus(seq_cus > num_sat),cus_s(cl(i)),s];
                j = 0;
            elseif cr(i) ~= cl(i-1)  && j == 1
                route_cap = [s,seq_cus(seq_cus > num_sat),s];
                rount(k:k+length(route_cap)-2) = route_cap(1:end-1);
                k = k + length(route_cap)-1;
                seq_cus = zeros(1,2*nc+1);
                seq_cus(1:2) = [s,cus_s(cr(i))];
                route_cap = [s,seq_cus(seq_cus > num_sat),cus_s(cl(i)),s];
                j = 0;
            else
                rount(k:k+length(route_cap)-2) = route_cap(1:end-1);
                k = k + length(route_cap)-1;
                seq_cus = zeros(1,2*nc+1);
                seq_cus(1:2) = [s,cus_s(cr(i))];
                route_cap = [s,seq_cus(seq_cus > num_sat),cus_s(cl(i)),s];
            end
        else
            route_cap = [s,seq_cus(seq_cus > num_sat),cus_s(cl(i)),s];
        end
        seq_cus(1:length(route_cap)) = route_cap;
        dis_cap = cell2mat(arrayfun(@(i) dis_sc(route_cap(i), route_cap(i+1)), 1:length(route_cap)-1,'UniformOutput', false));
        Energy_cap = sum(cell2mat(arrayfun(@(i,j)sum((5+i)*(3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4).*(j/V)), cap,dis_cap, 'UniformOutput', false))); 
        %----满足可连接(前后不重复)、不在同一路径、容量不超----
        if nlink(cr(i))<2 && nlink(cl(i))<2 && nv1~= nv2 && max(cap) <= capacity && Energy_cap <= E
                seq_veh(seq_veh==nv2) = nv1;                        %将车辆nv2中客户对应车辆号设为nv1
                indices = find(seq_veh == nv1);
                % 使用 cellfun 对这些索引对应的 cell 进行赋值
                cap_veh(indices) = cellfun(@(x) cap, cap_veh(indices), 'UniformOutput', false);
                nlink(cr(i)) = nlink(cr(i))+1;                      %更新客户1的连接数目
                nlink(cl(i)) = nlink(cl(i))+1;                      %更新客户2的连接数目
        else
            rount(k:k+length(route_cap)-3) = route_cap(1:end-2);
            k = k + length(route_cap)-2;
            seq_cus = zeros(1,2*nc+1);
            seq_cus(1:2) = [s,cus_s(cl(i))];
            route_cap = [s,seq_cus(seq_cus > num_sat),s];
            j = 1;
        end
        if i == length(cr) && cr(i)~=cl(i)
            rount(k:k+length(route_cap)-2) = route_cap(1:end-1);
            k = k + length(route_cap)-1;
        end
    end    
    rount(k:end) = []; %清除没有存储客户的空间
end

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

function A = zeroDiagonal(A)
    % 将矩阵A的对角线元素置为0
    [n, m] = size(A);
    if n ~= m
        error('矩阵必须是方阵');
    end
    A(1:n+1:n^2) = 0; % 将对角线元素置为0
end

function [cr_new, cl_new] = adjust_indices(cr, cl)
    % 初始化 cr_new 和 cl_new 为原始数组
    cr_new = cr;
    cl_new = cl;
    
    % 获取数组的长度
    n = length(cr_new);
    
    % 第一步：确保相同元素在 cl 中的顺序早于 cr
    for i = 1:n
        for j = 1:n
            % 检查 cl 和 cr 中当前列的顺序
            if cl_new(i) == cr_new(j) && i > j
                % 将 cl_new(i) 移动到 cl_new(j) 的前面
                cl_new = [cl_new(1:j-1), cl_new(i), cl_new(j:i-1), cl_new(i+1:end)];
                
                % 将 cr_new(i) 移动到 cr_new(j) 的前面
                cr_new = [cr_new(1:j-1), cr_new(i), cr_new(j:i-1), cr_new(i+1:end)];
                % 更新索引，避免重复处理
                break;
            end
        end
    end
    
    % 第二步：通过插入操作确保 cl(i) = cr(i+1)
    i = 1;
    while i < n
        % 在 cr_new 中查找 cl_new 对应的元素
        index_cr = find(cr_new == cl_new(i), 1);
        if ~isempty(index_cr) && index_cr > i
            % 插入找到的元素到合适的位置
            if i+1 <= n
                cr_new = [cr_new(1:i), cr_new(index_cr), cr_new(i+1:index_cr-1), cr_new(index_cr+1:end)];
                cl_new = [cl_new(1:i), cl_new(index_cr), cl_new(i+1:index_cr-1), cl_new(index_cr+1:end)];
            end
            
            % 更新长度
            n = length(cr_new);
        end
        % 移动到下一个位置
        i = i + 1;
    end
end

function [cr_new, cl_new] = remove_conflicting_elements(cr, cl)
    % 初始化 cr_new 和 cl_new 为原始数组
    cr_new = cr;
    cl_new = cl;
    
    % 初始化要删除的索引
    indices_to_remove_cl = [];
    indices_to_remove_cr = [];
    
    % 遍历 cl 中的每个元素
    for i = 1:length(cl)
        % 获取当前元素
        element = cl(i);
        
        % 查找当前元素在 cr 中的位置
        index_cr = find(cr == element);
        
        % 如果 cl 中的索引大于 cr 中的索引，则标记删除
        if ~isempty(index_cr) && i > index_cr
            % 标记要删除的 cl 索引
            indices_to_remove_cl = [indices_to_remove_cl, i];
            % 标记要删除的 cr 索引（对应 cl 索引）
            indices_to_remove_cr = [indices_to_remove_cr, index_cr];
        end
    end
    
    % 删除 cl 中不符合要求的元素
    cl_new(indices_to_remove_cl) = [];
    
    % 更新 cr 索引，因为 cl 被删除后，cr 的索引也需要调整
    cr_new(indices_to_remove_cl) = [];
end