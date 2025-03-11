%% --------------------------第二层路径全局破坏-------------------------
%seq_cs:当前路径，seq_c：删除的客户
function [route,seq_c,sat_offl,s_offs] = destroyl(vrp2e,route,sat_offl,sm)
    dis_sc = vrp2e.dis_sc;  
    neib_sc = vrp2e.neib_sc; 
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;

    s_offs = [];              %删除客户对应的卫星
    route(route==0) = [];     %将路径中填充的0元素去掉
    if sm == 1
        %----关闭一个开放的卫星，如果全部关闭则开放一个卫星:Satellite Removal----
        if sum(sat_offl) == 1 %如果只有一个开放的卫星
            s_off  = find(sat_offl==0);              %关闭的卫星
            sat_offl = zeros(1,num_sat);                  %卫星的状态
            sat_offl(s_off(ceil(rand*(num_sat-1)))) = 1;  %打开一个关闭的卫星
            route  = [];                             %设置路径为空
            seq_c   = num_sat+1:num_sat+num_cus;                    %设置全部客户待插入
        else
            s_on              = find(sat_offl==1);                   %开放的卫星
            s_off             = s_on(ceil(rand*numel(s_on)));        %待关闭的卫星
            sat_offl(s_off)   = 0;                                   %关闭卫星
            sat_fleet         = route(route<=num_sat);                    %车辆对应的卫星
            sign_fleet_s      = find(sat_fleet==s_off);              %卫星s对应的车辆号码
            sign_fleet        = cumsum(route<=num_sat,2);                 %卫星和客户对应的车辆
            [logic_loc,~]     = ismember(sign_fleet,sign_fleet_s);   %卫星s及对应客户参数为1
            cus_sat           = route(logic_loc);                    %卫星s的所有路径（含s）
            seq_c             = cus_sat(cus_sat>num_sat);                 %提取路径中的客户点（不含s）
            route(logic_loc)  = [];                                  %删除s_off对应的所有路径
        end
    elseif sm == 2
        %----开放一个关闭的卫星，然后删除与之临近的q个客户:Satellite Open----
        ns_off   = num_sat-sum(sat_offl);                     %seq_offl中0为关闭，1为开放
        if ns_off>0
            s_off          = find(sat_offl==0);          %关闭的卫星
            s_on           = s_off(ceil(rand*ns_off));   %开放的卫星           
            sat_offl(s_on) = 1;                          %开放的卫星设置为1
            ndl            = ceil(rand*min(60,0.4*num_cus));  %待删除的客户数目
            seq_c          = neib_sc(s_on,1:ndl);        %选取与客户邻近的ndl个客户   
            [~,loc_c]      = ismember(seq_c,route);      %多个客户在路径中的位置
            sign_fleet     = cumsum(route<=num_sat,2);        %卫星和客户对应的车辆
            fleet_cus      = sign_fleet(loc_c);          %指定客户对应的车辆号
            sat_fleet      = route(route<=num_sat);           %车辆对应的卫星
            s_offs         = sat_fleet(fleet_cus);       %客户seq_c对应的卫星
            route(loc_c)  = [];                          %从路径中删除客户
            len_route      = numel(route);                              %已有的路径长度
            loc_sat        = [find(route<=num_sat),len_route+1];             %卫星在路径中的位置
            route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
        else
            seq_c  = [];
            s_offs = [];
        end
    elseif sm == 3
        %----先关闭一个卫星，再基于距离按轮盘赌选择开放一个卫星:Satellite Swap----
        s_on              = find(sat_offl==1);                   %开放的卫星
        s_off             = s_on(ceil(rand*numel(s_on)));        %待关闭的卫星
        sat_offl(s_off)   = 0;                                   %将卫星关闭
        sat_fleet         = route(route<=num_sat);                    %车辆对应的卫星
        sign_fleet_s      = find(sat_fleet==s_off);              %卫星s对应的车辆号码
        sign_fleet        = cumsum(route<=num_sat,2);                 %卫星和客户对应的车辆
        [logic_loc,~]     = ismember(sign_fleet,sign_fleet_s);   %卫星s及对应客户参数为1
        cus_sat           = route(logic_loc);                    %卫星s的所有路径（含s）
        seq_c             = cus_sat(cus_sat>num_sat);                 %提取路径中的客户点（不含s）
        route(logic_loc) = [];                                   %删除s_off对应的所有路径
        dis               = dis_sc(s_off,1:num_sat);                  %删除卫星与其它卫星间的距离
        dis(s_off)        = inf;                                 %设置删除卫星到自己的距离为inf
        s_on = roulette(1./dis,1);                               %基于轮盘赌开放一个卫星
        sat_offl(s_on) = 1;                                      %设置开放的卫星
    else 
        disp('sm不能超过3！');
        seq_c = [];
    end
end