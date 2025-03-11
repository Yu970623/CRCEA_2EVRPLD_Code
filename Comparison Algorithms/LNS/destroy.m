%% --------------------------第二层路径局部破坏-------------------------
function [seq_cs,seq_c,s_off,sm] = destroy(vrp2e,seq_cs,ps,s_off,gter,gmax)
    [seq_cs,seq_c1]          = remove1(vrp2e,seq_cs,ps(1));  %随机某个客户临近的几个客户后的路径和删除掉的客户
    [seq_cs,seq_c2]          = remove2(vrp2e,seq_cs,ps(2));  %删除几个插入成本较高的客户（轮盘赌）
    [seq_cs,seq_c3]          = remove3(vrp2e,seq_cs,ps(3));  %随机删除部分整条路段
    [seq_cs,seq_c4]          = remove4(vrp2e,seq_cs,ps(4));  %随机删除某个客户
    [seq_cs,seq_c5,sm,s_off] = remove5(vrp2e,seq_cs,s_off,gter,gmax,ps(5));
    seq_c = [seq_c1,seq_c2,seq_c3,seq_c4,seq_c5];  %保留所有删除操作删掉的客户
end

%----1随机删除临近选取客户点rc的ndl个客户并贪婪插入:Related Removal----
function [seq_cs,seq_c] = remove1(vrp2e,seq_cs,p1)
    neib_cc = vrp2e.neib_cc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;
    
    ndl           = ceil(rand*p1*num_cus);       %随机删除的客户数目
    rc            = ceil(rand*num_cus);          %随机选取一个客户点
    seq_c         = neib_cc(rc,1:ndl);           %选取与客户邻近的ndl个客户 
    [~,loc_c]     = ismember(seq_c,seq_cs);      %多个客户在路径中的位置
    seq_cs(loc_c) = [];
    len_route     = numel(seq_cs);                               %已有的路径长度
    loc_sat       = [find(seq_cs<=num_sat),len_route+1];         %卫星在路径中的位置
    seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
end

%----2删除插入代价较高的客户: WorstRemoval----
function [seq_cs,seq_c] = remove2(vrp2e,seq_cs,p2)
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;
    loc_cus       = find(seq_cs>num_sat);           %客户在路径中的位置
    ndl           = ceil(rand*p2*length(loc_cus));  %随机出即将要删掉的客户数
    cost_route = funcopt3(vrp2e,seq_cs);
    no_cus = arrayfun(@(i) [seq_cs(1:i-1), seq_cs(i+1:end)], loc_cus, 'UniformOutput', false);
    cost_cus = arrayfun(@(i) cost_route-funcopt3(vrp2e,no_cus{i}), 1:length(no_cus), 'UniformOutput', false);
    cost_cus = cell2mat(cost_cus);
    %     cus_front     = seq_cs(loc_cus-1);              %所有客户的前一个客户保留（删掉每个路段最后一个客户）
%     rback         = seq_cs;
%     loc_sat       = find(seq_cs<=num_sat);
%     rback(loc_sat(2:end)) = seq_cs(loc_sat(1:end-1));
%     route2        = [rback(2:end),seq_cs(loc_sat(end))];
%     cus_back      = route2(loc_cus);                %所有客户的前一个客户保留（删掉每个路段第一个客户）
%     nsc           = num_sat+num_cus;
%     cost_cus      = dis_sc(cus_front+(seq_cs(loc_cus)-1)*nsc)+dis_sc(seq_cs(loc_cus)+(cus_back-1)*nsc)-dis_sc(cus_front+(cus_back-1)*nsc);
    del_c         = zeros(1,ndl);
    for i = 1:ndl
        cl           = roulette(cost_cus,1);
        del_c(i)     = loc_cus(cl);
        cost_cus(cl) = 0;
    end
    seq_c     = seq_cs(del_c);
    seq_cs(del_c) = [];
    len_route     = numel(seq_cs);                               %已有的路径长度
    loc_sat       = [find(seq_cs<=num_sat),len_route+1];         %卫星在路径中的位置
    seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
end
    
%----3删除k条路径进行重组--------
function [seq_cs,seq_c] = remove3(vrp2e,seq_cs,p3)
    demand = vrp2e.demand;
    fleet = vrp2e.fleet;
    num_sat = vrp2e.num_sat;
    k          = ceil(rand*p3*ceil(sum(demand)/fleet(2,1)));   %待删除的路径数目
    sign_fleet = cumsum(seq_cs<=num_sat,2);                    %卫星和客户对应的车辆
    if sign_fleet(end) > k
        f_del  = ceil(rand(1,sign_fleet(end))*k);              %待删除的路径编号
    else
        f_del  = 1:sign_fleet(end);
    end
    log_cs  = ismember(sign_fleet,f_del);      %点对应的路径是否被删除
    seq_c   = seq_cs(log_cs);                  %提取待删除的路径
    seq_c(seq_c<=num_sat) = [];                %提取路径中的客户
    seq_cs(log_cs) = [];                       %从总路径中删除部分路径
end 

%----4以概率删除单点路径：Route Removal------------
function [seq_cs,seq_c] = remove4(vrp2e,seq_cs,p4)
    num_sat = vrp2e.num_sat;
    
    if rand < p4
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
%----5以概率关闭一个卫星或开启所有卫星并判断是否可以关闭------------
function [seq_cs,seq_c,sm,s_off] = remove5(vrp2e,seq_cs,s_off,gter,gmax,p5)
    num_sat = vrp2e.num_sat;
    
    sm    = 0;
    seq_c = [];
    if gter > gmax
        rnd = rand();
        if rnd < p5
            num_on  = sum(s_off);
            if num_on > 1
                sm                = 1;
                loc_on            = find(s_off==1);                    %开放的卫星
                s_sel             = loc_on(ceil(rand*num_on));         %选择一个卫星关闭
                s_off(s_sel)      = 0;                                 %将选取的卫星关闭
                sat_fleet         = seq_cs(seq_cs<=num_sat);           %车辆对应的卫星
                sign_fleet_s      = find(sat_fleet==s_sel);            %卫星s对应的车辆号码
                sign_fleet        = cumsum(seq_cs<=num_sat,2);         %卫星和客户对应的车辆
                [logic_loc,~]     = ismember(sign_fleet,sign_fleet_s); %卫星s及对应客户参数为1
                cus_sat           = seq_cs(logic_loc);                 %卫星s的所有路径（含s）
                seq_c             = cus_sat(cus_sat>num_sat);          %提取路径中的客户点（不含s）
                seq_cs(logic_loc) = [];                                %删除s_off对应的所有路径
            end
        elseif rnd < p5*(1.0+1.0/num_sat)
            sm    = 1;
            s_off = 1;           %开放所有卫星
        end
    end
end

function fvrp = funcopt3(vrp2e,route)
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    cap_sc = energy(vrp2e,route);
    loc_sat = find(route<=num_sat); 
    rback   = route; rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
    route2  = [rback(2:end),route(loc_sat(end))];
    Dis_route = dis_sc(route+(route2-1)*(num_sat+num_cus));
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
    Energy_distribution = Energy_route.*Dis_route./vrp2e.V; 
    fvrp = sum(Energy_distribution);
end