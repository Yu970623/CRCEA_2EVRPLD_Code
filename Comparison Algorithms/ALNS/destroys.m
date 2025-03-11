%s_offs:记录一出的客户对应的卫星
%% --------------------------第二层路径局部破坏-------------------------
function [route,seq_c,s_offs,s_offl] = destroys(vrp2e,route,s_offl,sm)
    dis_sc = vrp2e.dis_sc;  
    neib_cc = vrp2e.neib_cc; 
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;

    len_route = numel(route);                                %队列长度
    loc_sat   = [find(route<=num_sat),len_route+1];               %卫星对应的序号
    route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = []; %删除空车从所在路径
    ndl       = ceil(rand*min(60,0.4*num_cus));                   %随机删除的客户数目
    if sm == 4
        %----随机删除ndl个客户并贪婪插入:Random Remove----
        seq_c          = randperm(num_cus,ndl)+num_sat;
        [~,loc_c]      = ismember(seq_c,route);     %多个客户在路径中的位置
        sign_fleet     = cumsum(route<=num_sat,2);       %卫星和客户对应的车辆
        fleet_cus      = sign_fleet(loc_c);          %指定客户对应的车辆号
        sat_fleet      = route(route<=num_sat);          %车辆对应的卫星
        s_offs         = sat_fleet(fleet_cus);       %客户seq_c对应的卫星
        route(loc_c)   = [];                         %从路径中删除客户
        len_route      = numel(route);                              %已有的路径长度
        loc_sat        = [find(route<=num_sat),len_route+1];             %卫星在路径中的位置
        route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
    elseif sm == 5
                %----随机删除ndl个客户并贪婪插入:Random Remove----
        seq_c          = randperm(num_cus,ndl)+num_sat;
        [~,loc_c]      = ismember(seq_c,route);     %多个客户在路径中的位置
        sign_fleet     = cumsum(route<=num_sat,2);       %卫星和客户对应的车辆
        fleet_cus      = sign_fleet(loc_c);          %指定客户对应的车辆号
        sat_fleet      = route(route<=num_sat);          %车辆对应的卫星
        s_offs         = sat_fleet(fleet_cus);       %客户seq_c对应的卫星
        route(loc_c)   = [];                         %从路径中删除客户
        len_route      = numel(route);                              %已有的路径长度
        loc_sat        = [find(route<=num_sat),len_route+1];             %卫星在路径中的位置
        route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
        %----删除插入代价较高的客户: WorstRemoval----
%         loc_cus       = find(route>num_sat);                %客户在路径中的位置
%         cus_front     = route(loc_cus-1);              %客户的前一个点
%         rback         = route;
%         loc_sat       = find(route<=num_sat);
%         rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
%         route2        = [rback(2:end),route(loc_sat(end))];
%         loc_cus       = find(route2>num_sat);                %客户在路径中的位置
%         cus_back      = route2(loc_cus+1);              %客户的后一个点
%         cus_r         = route2(loc_cus);                %提取全部的客户
%         nsc           = num_sat+num_cus;
%         dis_front     = dis_sc(cus_front+(cus_r-1)*nsc);
%         dis_back      = dis_sc(cus_r+(cus_back-1)*nsc);
%         dis_fb        = dis_sc(cus_front+(cus_back-1)*nsc);
%         perturb       = 0.8+0.4*rand(1,num_cus); %代价扰动
%         cost_cus      = perturb.*(dis_front+dis_back-dis_fb)./(dis_front+dis_back);
%         [~,srt_c]     = sort(cost_cus,'descend');
%         seq_c         = cus_r(srt_c(1:ndl));
%         [~,loc_c]     = ismember(seq_c,route);      %多个客户在路径中的位置
%         sign_fleet    = cumsum(route<=num_sat,2);        %卫星和客户对应的车辆
%         fleet_cus     = sign_fleet(loc_c);          %指定客户对应的车辆号
%         sat_fleet     = route(route<=num_sat);            %车辆对应的卫星
%         s_offs        = sat_fleet(fleet_cus);        %客户seq_c对应的卫星
%         route(loc_c)  = [];                          %从路径中删除客户
%         len_route     = numel(route);                              %已有的路径长度
%         loc_sat       = [find(route<=num_sat),len_route+1];             %卫星在路径中的位置
%         route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
    elseif sm == 6
        %----随机删除临近的ndl个客户并贪婪插入:Related Removal----
        rc            = ceil(rand*num_cus);              %随机选取一个客户点
        seq_c         = neib_cc(rc,1:ndl);          %选取与客户邻近的ndl个客户
        [~,loc_c]     = ismember(seq_c,route);      %多个客户在路径中的位置
        sign_fleet    = cumsum(route<=num_sat,2);        %卫星和客户对应的车辆
        fleet_cus     = sign_fleet(loc_c);          %指定客户对应的车辆号
        sat_fleet     = route(route<=num_sat);            %车辆对应的卫星
        s_offs        = sat_fleet(fleet_cus);        %客户seq_c对应的卫星
        route(loc_c)  = [];                          %从路径中删除客户
        len_route     = numel(route);                              %已有的路径长度
        loc_sat       = [find(route<=num_sat),len_route+1];             %卫星在路径中的位置
        route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
    elseif sm == 7
        %----随机删除一条路径：Route Removal----
        sign_fleet  = cumsum(route<=num_sat,2);           %卫星和客户对应的车辆
        num_fleet   = sign_fleet(end);               %路径总数
        
        rf           = ceil(rand*num_fleet);         %随机选取一条路径
        loc_r        = find(sign_fleet==rf);         %提取该路径对应的位置  
        seq_c        = route(loc_r(2:end));          %提取路径中的客户
        if sum(s_offl) == 1 && num_fleet ==1         %如果只有一个开放的卫星且只有一条路径
            s_off  = find(s_offl==0);                %关闭的卫星
            s_offl = zeros(1,num_sat);                    %卫星的状态
            s_offl(s_off(ceil(rand*(num_sat-1)))) = 1;    %打开一个关闭的卫星
            s_offs = [];
        else
            s_offs = route(loc_r(1))*ones(1,numel(seq_c));%客户不可插入的卫星
        end
        route(loc_r) = [];                           %删除该路径
    elseif sm == 8
        %----每个开放的卫星删除k条路径进行重组----
        num_son     = sum(s_offl);                   %开放卫星的总数
        if num_son>1
            sign_fleet  = cumsum(route<=num_sat,2);       %卫星和客户对应的车辆
            num_fleet   = sign_fleet(end);           %路径总数
            dis_f       = zeros(1,num_fleet);        %路径中的客户在本卫星和其它卫星间的最短距离
            for f = 1:num_fleet
                cus_f    = route((sign_fleet==f));    %第f条路径
                s        = cus_f(1);                  %第条路径的卫星
                cus_f    = cus_f(2:end);              %第f条路径中的客户
                s_on     = s_offl;  s_on(s)=0;        %其它开放的卫星
                dis_f(f) = (0.8+0.4*rand)*min(min(bsxfun(@plus,dis_sc(s,cus_f),dis_sc(s_on==1,cus_f))));
            end
            [~,lst_f]    = sort(dis_f);               %将每个路径的距离进行排序
            sat_fleet    = route(route<=num_sat);          %车辆对应的卫星
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
            seq_c   = route(log_cs);                   %提取待删除的路径
            seq_c(seq_c<=num_sat) = [];                %提取路径中的客户
            [~,pos_cus] = ismember(seq_c,route);       %客户在路径中的位置
            fleet_cus   = sign_fleet(pos_cus);         %指定客户对应的车辆号
            s_offs      = sat_fleet(fleet_cus);        %客户seq_c对应的卫星
            route(log_cs) = [];                        %从总路径中删除部分路径
        else
            seq_c       = [];
            s_offs      = [];
        end
    else
        disp('sm不能超过5！');
        seq_c = [];
        s_offs = [];
    end
end
