%% --------------------------第一层路径优化-------------------------
function [seq_sat,cap_sat] = evofirst(vrp2e,seq_ss,cap_ss,sm)
    %global num_sat; global fleet;
    num_sat = vrp2e.num_sat;
    num_dep = length(vrp2e.rad_ds(:,1));
    fleet = vrp2e.fleet;
    seq_ss(find(seq_ss ~= 0, 1, 'last')+1:end) = [];
    cap_ss(find(cap_ss ~= 0, 1, 'last')+1:end) = [];
    if length(seq_ss)~=length(cap_ss)
        seq_ss
    end
%     cap_ss = cap_ss(1:length(seq_ss));
    if sum(seq_ss<=num_dep)>1 && length(unique(seq_ss(seq_ss>num_dep)))>1
        if sm == 1
           %----从分配好的队列中随机删除ndl个卫星，并贪婪插入队列中----
            loc_s  = find(seq_ss>num_dep);      %卫星在一级路段中所处下标
            uni_sat = unique(seq_ss(seq_ss>num_dep));
            len_s  = length(uni_sat);     %实际卫星个数
            ndl    = ceil(rand*len_s*0.4);       %随机删除和临近点删除的卫星数目
            [del_s,~]  = ismember(seq_ss, uni_sat(randperm(len_s,ndl))); %要删除的卫星下标
            del_s = find(del_s);
            add_sat = unique(seq_ss(setdiff(loc_s, del_s)))-num_dep;    %除了被删掉卫星之外的卫星序号
            %----将已删除卫星所服务的客户随机分配给其他卫星
            cap_s = zeros(1,num_sat);
            for k = 1:length(add_sat)
                cap_s(add_sat(k))=sum(cap_ss(seq_ss==(add_sat(k)+num_dep)));
            end
            for i = 1:ndl   %所有被删除的卫星所含的货运重量
                sat_added = add_sat(randi(numel(add_sat)));  %随机插入到剩余未被删除的卫星所在下标
                cap_s(sat_added) = cap_s(sat_added)+sum(cap_ss(seq_ss == seq_ss(del_s(i))));
            end
        else
           %----从分配好的队列中随机删除多条路径并贪婪插入----
            num_f   = ceil(sum(cap_ss)/fleet(1,1));                       %计算第一层最少使用的车辆数
            ndf     = randperm(num_f,ceil(0.4*rand*num_f));               %随机删除的两条路径序号
            sign_fleet = cumsum(seq_ss<=num_dep,2);                            %卫星和客户对应的车辆
            loc_del    = ismember(sign_fleet,ndf);
            seq_st = seq_ss(loc_del(1:length(seq_ss)));   %删掉的路段中的卫星
            cap_st = cap_ss(loc_del(1:length(cap_ss)));   %删掉的路段中卫星对应的货物重量
            cap = cap_st(cap_st>0);
            add_sat = seq_ss(~loc_del(1:length(seq_ss)));    %除了被删掉卫星之外的卫星序号
            add_cap = cap_ss(~loc_del(1:length(cap_ss))); 
            %----将已删除卫星所服务的客户随机分配给其他卫星
            cap_s = zeros(1,num_sat);
            cap_s(add_sat(add_sat>num_dep)-num_dep) = add_cap(add_cap>0);
            for i = 1:length(cap)   %所有被删除的卫星所含的货运重量
                sat = add_sat(add_sat>num_dep)-num_dep;
                sat_added = sat(randi(numel(sat)));  %随机插入到剩余未被删除的卫星所在下标
                cap_s(sat_added) = cap_s(sat_added)+cap(i);
            end
        end
        [seq_ss,cap_ss] = greedyin1(vrp2e,cap_s);
    end
    len_route1 = 2*(fleet(1,2)+num_sat);
    len_r   = length(seq_ss);
    seq_sat = [seq_ss,zeros(1,len_route1-len_r)];
    cap_sat = [cap_ss,zeros(1,len_route1-len_r)];
end

%% ----贪婪方式修复第一层路径----
function [seq_sat,cap_sat] = greedyin1(vrp2e,cap_r)
    fleet = vrp2e.fleet;
    dis_ds = vrp2e.dis_ds;
    num_sat = vrp2e.num_sat;
    num_dep = length(dis_ds)-num_sat;
    
    %----将超容量卫星预排序----
    cap_v     = zeros(1,2*fleet(1,2)+2*num_sat);        %设置每个车辆的可装载的容量
    sat_unset   = zeros(1,2*fleet(1,2)+2*num_sat);        %预分配存储卫星序号2*fleet(1,2)+2*ns，超出空间删除
    seq_sat   = zeros(1,2*fleet(1,2)+2*num_sat);        %预分配存储卫星序号2*fleet(1,2)+2*ns，超出空间删除
    cap_sat   = zeros(1,2*fleet(1,2)+2*num_sat);        %预分配存储卫星容量2*fleet(1,2)+2*ns，超出空间删除
    len_route = 0;
    n = 0;
    for i = 1:num_sat
        num_s = ceil(cap_r(i)/fleet(1,1));
        if num_s == 1
            sat_unset(i+n) = i+num_dep; %待分配的卫星序列
            cap_v(i+n) = cap_r(i);
        elseif num_s > 1
            sat_unset(i:i+num_s-1) = i+num_dep;
            cap_v(i:i+num_s-2) = fleet(1,1);
            cap_v(i+num_s-1) = cap_r(i)-fleet(1,1)*(num_s-1);
            n = n + num_s - 1;
        end
    end
    %----贪婪插入----
    sat_unset = sat_unset(sat_unset~=0);
    cap_v = cap_v(cap_v~=0);
    penalty   = 100000000;
    nsc = num_sat+num_dep;
    for i = 1:length(sat_unset)   %逐一插入路径
        cost_news = dis_ds(sat_unset(i),1:num_dep);
        [cost,l]  = min(cost_news);
        if len_route > 0 
            route = seq_sat(1:len_route);
            num_fleet     = sum(route<=num_dep);
            sign_fleet = cumsum(route<=num_dep,2);
            logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');
            cap_s         = zeros(1,2*fleet(1,2));
            for j = 1:num_fleet
                cap_s(j)  = sum(cap_sat(logical_fleet(j,:)));
            end
            cap_ds     = cap_s(sign_fleet);
            loc_depot = find(route<=num_dep);
            rback = route; rback(loc_depot(2:end)) = route(loc_depot(1:end-1));
            route2     = [rback(2:end),route(loc_depot(end))];
            punish     = [penalty *max(cap_ds+cap_v(i)-fleet(1,1),0),penalty *max(num_fleet+1-fleet(1,2),0)];
            cost_c     = [dis_ds(sat_unset(i),route)+dis_ds(sat_unset(i),route2)-dis_ds(route+(route2-1)*nsc),cost]+punish;
            [~,loc_in] = min(cost_c);

            if loc_in <= len_route
                seq_sat(loc_in+2:len_route+1) = seq_sat(loc_in+1:len_route);
                seq_sat(loc_in+1) = sat_unset(i);
                cap_sat(loc_in+2:len_route+1) = cap_sat(loc_in+1:len_route);
                cap_sat(loc_in+1) = cap_v(i);
                len_route       = len_route+1;
            else
                seq_sat(len_route+1:len_route+2) = [l,sat_unset(i)];
                cap_sat(len_route+2) = cap_v(i);
                len_route = len_route+2;
            end
        else
            seq_sat(1:2) = [l,sat_unset(i)];
            len_route  = 2;
            cap_sat(2) = cap_v(i);
        end
    end
    seq_sat(len_route+1:end) = [];
    cap_sat(len_route+1:end) = [];
end