%% --------------------------第二层路径优化-------------------------
function [seq_cs,sat_offl] = evosecond(vrp2e,seq_cs,sm1,sm2,sm3,sat_offl,opt_cus_pre,epsilon)
    %global dis_sc; global neib_cc; global num_sat; global num_cus;
    dis_sc = vrp2e.dis_sc;  
    neib_cc = vrp2e.neib_cc; 
    neib_sc = vrp2e.neib_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    seq_cs_ini = seq_cs;
    [vrp_ini,vlt_ini] = msecond(vrp2e,seq_cs_ini);
    s_offs = [];              %删除客户对应的卫星(修复用)
    seq_cs(seq_cs==0) = [];     %将路径中填充的0元素去掉

    %% 全局搜索
    if sm1 == 1
        %----关闭一个开放的卫星，如果全部关闭则开放一个卫星:Satellite Removal----
        if sum(sat_offl) == 1 %如果只有一个开放的卫星
            s_off  = find(sat_offl==0);              %关闭的卫星
            sat_offl = zeros(1,num_sat);                  %卫星的状态
            sat_offl(s_off(ceil(rand*(num_sat-1)))) = 1;  %打开一个关闭的卫星
            seq_cs  = [];                             %设置路径为空
            seq_c   = num_sat+1:num_sat+num_cus;                    %设置全部客户待插入
        else
            s_on              = find(sat_offl==1);                   %开放的卫星
            s_off             = s_on(ceil(rand*numel(s_on)));        %待关闭的卫星
            sat_offl(s_off)   = 0;                                   %关闭卫星
            sat_fleet         = seq_cs(seq_cs<=num_sat);                    %车辆对应的卫星
            sign_fleet_s      = find(sat_fleet==s_off);              %卫星s对应的车辆号码
            sign_fleet        = cumsum(seq_cs<=num_sat,2);                 %卫星和客户对应的车辆
            [logic_loc,~]     = ismember(sign_fleet,sign_fleet_s);   %卫星s及对应客户参数为1
            cus_sat           = seq_cs(logic_loc);                    %卫星s的所有路径（含s）
            seq_c             = cus_sat(cus_sat>num_sat);                 %提取路径中的客户点（不含s）
            seq_cs(logic_loc)  = [];                                  %删除s_off对应的所有路径
        end
    elseif sm1 == 2
        %----开放一个关闭的卫星，然后删除与之临近的q个客户:Satellite Open----
        ns_off   = num_sat-sum(sat_offl);                     %seq_offl中0为关闭，1为开放
        if ns_off>0
            s_off          = find(sat_offl==0);          %关闭的卫星
            s_on           = s_off(ceil(rand*ns_off));   %开放的卫星           
            sat_offl(s_on) = 1;                          %开放的卫星设置为1
            ndl            = ceil(rand*min(60,0.4*num_cus));  %待删除的客户数目
            seq_c          = neib_sc(s_on,1:ndl);        %选取与客户邻近的ndl个客户   
            [~,loc_c]      = ismember(seq_c,seq_cs);      %多个客户在路径中的位置
            sign_fleet     = cumsum(seq_cs<=num_sat,2);        %卫星和客户对应的车辆
            fleet_cus      = sign_fleet(loc_c);          %指定客户对应的车辆号
            sat_fleet      = seq_cs(seq_cs<=num_sat);           %车辆对应的卫星
            s_offs         = sat_fleet(fleet_cus);       %客户seq_c对应的卫星
            seq_cs(loc_c)  = [];                          %从路径中删除客户
            len_route      = numel(seq_cs);                              %已有的路径长度
            loc_sat        = [find(seq_cs<=num_sat),len_route+1];             %卫星在路径中的位置
            seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
        else
            seq_c  = [];
            s_offs = [];
        end
    elseif sm1 == 3
        %----先关闭一个卫星，再基于距离按轮盘赌选择开放一个卫星:Satellite Swap----
        s_on              = find(sat_offl==1);                   %开放的卫星
        s_off             = s_on(ceil(rand*numel(s_on)));        %待关闭的卫星
        sat_offl(s_off)   = 0;                                   %将卫星关闭
        sat_fleet         = seq_cs(seq_cs<=num_sat);                    %车辆对应的卫星
        sign_fleet_s      = find(sat_fleet==s_off);              %卫星s对应的车辆号码
        sign_fleet        = cumsum(seq_cs<=num_sat,2);                 %卫星和客户对应的车辆
        [logic_loc,~]     = ismember(sign_fleet,sign_fleet_s);   %卫星s及对应客户参数为1
        cus_sat           = seq_cs(logic_loc);                    %卫星s的所有路径（含s）
        seq_c             = cus_sat(cus_sat>num_sat);                 %提取路径中的客户点（不含s）
        seq_cs(logic_loc) = [];                                   %删除s_off对应的所有路径
        dis               = dis_sc(s_off,1:num_sat);                  %删除卫星与其它卫星间的距离
        dis(s_off)        = inf;                                 %设置删除卫星到自己的距离为inf
        s_on = roulette(1./dis,1);                               %基于轮盘赌开放一个卫星
        sat_offl(s_on) = 1;                                      %设置开放的卫星
    end
    %% 局部搜索
    len_route = numel(seq_cs);                                %队列长度
    loc_sat   = [find(seq_cs<=num_sat),len_route+1];               %卫星对应的序号
    seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = []; %删除空车从所在路径
    ndl       = ceil(rand*min(60,0.4*num_cus));                   %随机删除的客户数目
    if sm2 == 1
        %----随机删除ndl个客户并贪婪插入:Random Remove----
        seq_c          = randperm(num_cus,ndl)+num_sat;
        [~,loc_c]      = ismember(seq_c,seq_cs);     %多个客户在路径中的位置
        sign_fleet     = cumsum(seq_cs<=num_sat,2);       %卫星和客户对应的车辆
        fleet_cus      = sign_fleet(loc_c);          %指定客户对应的车辆号
        sat_fleet      = seq_cs(seq_cs<=num_sat);          %车辆对应的卫星
        s_offs         = sat_fleet(fleet_cus);       %客户seq_c对应的卫星
        seq_cs(loc_c)   = [];                         %从路径中删除客户
        len_route      = numel(seq_cs);                              %已有的路径长度
        loc_sat        = [find(seq_cs<=num_sat),len_route+1];             %卫星在路径中的位置
        seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
    elseif sm2 == 2
                %----随机删除ndl个客户并贪婪插入:Random Remove----
        seq_c          = randperm(num_cus,ndl)+num_sat;
        [~,loc_c]      = ismember(seq_c,seq_cs);     %多个客户在路径中的位置
        sign_fleet     = cumsum(seq_cs<=num_sat,2);       %卫星和客户对应的车辆
        fleet_cus      = sign_fleet(loc_c);          %指定客户对应的车辆号
        sat_fleet      = seq_cs(seq_cs<=num_sat);          %车辆对应的卫星
        s_offs         = sat_fleet(fleet_cus);       %客户seq_c对应的卫星
        seq_cs(loc_c)   = [];                         %从路径中删除客户
        len_route      = numel(seq_cs);                              %已有的路径长度
        loc_sat        = [find(seq_cs<=num_sat),len_route+1];             %卫星在路径中的位置
        seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
    elseif sm2 == 3
        %----随机删除临近的ndl个客户并贪婪插入:Related Removal----
        rc            = ceil(rand*num_cus);              %随机选取一个客户点
        seq_c         = neib_cc(rc,1:ndl);          %选取与客户邻近的ndl个客户
        [~,loc_c]     = ismember(seq_c,seq_cs);      %多个客户在路径中的位置
        sign_fleet    = cumsum(seq_cs<=num_sat,2);        %卫星和客户对应的车辆
        fleet_cus     = sign_fleet(loc_c);          %指定客户对应的车辆号
        sat_fleet     = seq_cs(seq_cs<=num_sat);            %车辆对应的卫星
        s_offs        = sat_fleet(fleet_cus);        %客户seq_c对应的卫星
        seq_cs(loc_c)  = [];                          %从路径中删除客户
        len_route     = numel(seq_cs);                              %已有的路径长度
        loc_sat       = [find(seq_cs<=num_sat),len_route+1];             %卫星在路径中的位置
        seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
    elseif sm2 == 4
        %----随机删除一条路径：Route Removal----
        sign_fleet  = cumsum(seq_cs<=num_sat,2);           %卫星和客户对应的车辆
        num_fleet   = sign_fleet(end);               %路径总数
        
        rf           = ceil(rand*num_fleet);         %随机选取一条路径
        loc_r        = find(sign_fleet==rf);         %提取该路径对应的位置  
        seq_c        = seq_cs(loc_r(2:end));          %提取路径中的客户
        if sum(sat_offl) == 1 && num_fleet ==1         %如果只有一个开放的卫星且只有一条路径
            s_off  = find(sat_offl==0);                %关闭的卫星
            sat_offl = zeros(1,num_sat);                    %卫星的状态
            sat_offl(s_off(ceil(rand*(num_sat-1)))) = 1;    %打开一个关闭的卫星
            s_offs = [];
        else
            s_offs = seq_cs(loc_r(1))*ones(1,numel(seq_c));%客户不可插入的卫星
        end
        seq_cs(loc_r) = [];                           %删除该路径
    elseif sm2 == 5
        %----每个开放的卫星删除k条路径进行重组----
        num_son     = sum(sat_offl);                   %开放卫星的总数
        if num_son>1
            sign_fleet  = cumsum(seq_cs<=num_sat,2);       %卫星和客户对应的车辆
            num_fleet   = sign_fleet(end);           %路径总数
            dis_f       = zeros(1,num_fleet);        %路径中的客户在本卫星和其它卫星间的最短距离
            for f = 1:num_fleet
                cus_f    = seq_cs((sign_fleet==f));    %第f条路径
                s        = cus_f(1);                  %第条路径的卫星
                cus_f    = cus_f(2:end);              %第f条路径中的客户
                s_on     = sat_offl;  s_on(s)=0;        %其它开放的卫星
                dis_f(f) = (0.8+0.4*rand)*min(min(bsxfun(@plus,dis_sc(s,cus_f),dis_sc(s_on==1,cus_f))));
            end
            [~,lst_f]    = sort(dis_f);               %将每个路径的距离进行排序
            sat_fleet    = seq_cs(seq_cs<=num_sat);          %车辆对应的卫星
            num_sdel     = ceil(rand(1,num_sat)*3);        %每个开放的卫星待删除的路径数目
            f_del        = zeros(1,num_fleet);        %待删除的路径标号，1为删除
            for f = lst_f
                if num_sdel(sat_fleet(f))>0
                    f_del(f) = 1;
                    num_sdel(sat_fleet(f)) = num_sdel(sat_fleet(f))-1;
                end
            end
            f_del   = find(f_del==1);
            log_cs  = ismember(sign_fleet,f_del);      %点对应的路径是否被删除
            seq_c   = seq_cs(log_cs);                   %提取待删除的路径
            seq_c(seq_c<=num_sat) = [];                %提取路径中的客户
            [~,pos_cus] = ismember(seq_c,seq_cs);       %客户在路径中的位置
            fleet_cus   = sign_fleet(pos_cus);         %指定客户对应的车辆号
            s_offs      = sat_fleet(fleet_cus);        %客户seq_c对应的卫星
            seq_cs(log_cs) = [];                        %从总路径中删除部分路径
        else
            seq_c       = [];
            s_offs      = [];
        end
    end

    off = 1:num_sat;
    if sm3 == 3  %非原路径修复方式
        s_off = [repmat(off(sat_offl==0),[numel(seq_c),1]),s_offs'];
    elseif sm2 == 4 && numel(s_offs)>0 %单条路径删除：原路径不可重现
        s_off = [off(sat_offl==0),s_offs(1)]; 
    else
        s_off = off(sat_offl==0);
    end
    seq_cs_pre = greedyin2(vrp2e,seq_cs,seq_c,s_off);
    [vrp_pre,vlt_pre] = msecond(vrp2e,seq_cs_pre);
    if (vlt_pre <= vlt_ini ||vlt_pre <= epsilon) && vrp_pre < vrp_ini
        seq_cs = seq_cs_pre;
    else
        vlt_pre = vlt_ini;
        vrp_pre = vrp_ini;
        seq_cs = seq_cs_ini;
    end
    seq_cs_pre = seq_cs;

    %% 局部搜索
    sm = ceil(rand*2);
    if sm == 1
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
    seq_cs_new = greedyinr2(vrp2e,seq_cs,seq_c);
    [vrp_cs,vlt_cs] = msecond(vrp2e,seq_cs_new);
    if vlt_cs <= vlt_pre && vrp_cs < vrp_pre
        seq_cs = seq_cs_new;
        vrp_pre = vrp_cs;
        vlt_pre = vlt_cs;
    else
        seq_cs = seq_cs_pre;
    end

    %% 基因交叉
    opt_cus_pre(opt_cus_pre==0)  = [];
    if ~isequal(seq_cs, opt_cus_pre)
        [seq_cs_temp, opt_cus_pre_temp, seq_cs_waite, opt_cus_pre_waite] = crossover(seq_cs, opt_cus_pre, num_sat);
%         seq_cs_temp = [seq_cs_temp, zeros(1, numel(seq_cs) - numel(seq_cs_temp))];
        if ~isempty(seq_cs_waite)
            seq_cs_temp = greedyinr2(vrp2e,seq_cs_temp,seq_cs_waite);
        end
%         opt_cus_pre_temp = [opt_cus_pre_temp, zeros(1, numel(seq_cs) - numel(opt_cus_pre_temp))];
        if ~isempty(opt_cus_pre_waite)
            opt_cus_pre_temp = greedyinr2(vrp2e,opt_cus_pre_temp,opt_cus_pre_waite);
        end
        [vrp_cs_temp,vlt_cs] = msecond(vrp2e,seq_cs_temp);
        [vrp_opt,vlt_opt] = msecond(vrp2e,opt_cus_pre_temp);
        if (vlt_cs<=vlt_pre || vlt_cs<=epsilon) && (vlt_opt<=vlt_pre || vlt_opt<=epsilon)
            if vrp_cs_temp<vrp_opt && vrp_cs_temp<vrp_pre
                seq_cs = seq_cs_temp;
                vrp_pre = vrp_cs_temp;
                vlt_pre = vlt_cs;
            elseif vrp_opt<vrp_cs_temp && vrp_opt<vrp_pre
                seq_cs = opt_cus_pre_temp;
                vrp_pre = vrp_opt;
                vlt_pre = vlt_opt;
            end
        end
    end
    seq_cs(seq_cs==0)  = [];
    if sm2==0
        seq_cs1 = move2(vrp2e,seq_cs); %随机删除所有客户并重新插入完整路段对比结果
        seq_cs2 = swap2(vrp2e,seq_cs1); %随机将所有客户与其单一+连续邻居交换后对比结果
        seq_cs3 = opt2(vrp2e,seq_cs2);  %每个子路段内，遍历逆转部分子路段
        seq_cs4 = opt2x(vrp2e,seq_cs3); %拼接同一卫星下不同子路段内的子路段
        [vrp_cs1,vlt_cs1] = msecond(vrp2e,seq_cs1);
        [vrp_cs2,vlt_cs2] = msecond(vrp2e,seq_cs2);
        [vrp_cs3,vlt_cs3] = msecond(vrp2e,seq_cs3);
        [vrp_cs4,vlt_cs4] = msecond(vrp2e,seq_cs4);
        vrp = [vrp_cs1,vrp_cs2,vrp_cs3,vrp_cs4];
        vlt = [vlt_cs1,vlt_cs2,vlt_cs3,vlt_cs4];
        if ~isempty(min(vrp(vlt<=vlt_pre | vlt<=epsilon)))
            vrp_csn = vrp(vrp==min(vrp(vlt<=vlt_pre | vlt<=epsilon)));
        else
            vrp_csn = Inf;
        end
        seq = {seq_cs1,seq_cs2,seq_cs3,seq_cs4};
        if vrp_csn < vrp_pre
            seq_cs = seq{vrp==min(vrp(vlt<=vlt_pre | vlt<=epsilon))};
        end
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


%将seq_c中的客户插入到已有的队列seq_cs中，s_off：关闭或禁止的卫星
function seq_r = greedyinr2(vrp2e,route,seq_c)
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