%% ---------------------计算第一层的vrp值---------------------------
%seq_sat(:,i),cap_sat(:,i)对应问题第一层的第i个解，penalty1为第一层超出容量的惩罚。
function [e_cost1,c_cost1,fvlt1] = mfirst(vrp2e,route,capacity)
    dis_ds = vrp2e.dis_ds;
    fleet = vrp2e.fleet;
    num_sat = vrp2e.num_sat;
    num_dep = length(dis_ds)-num_sat;
    fvlt1 = 0;
    nds    = size(dis_ds,1);
    %----消除空路径----
    loc_del            = find(route(2:end)-route(1:end-1)==0);   %空路径所在的位置
    route (loc_del)    = [];
    capacity (loc_del) = [];
    %----计算路径数目和容量----
    num_fleet     = sum(route<=num_dep);                               %路径数目(不包含空车辆）
    sign_fleet    = cumsum(route<=num_dep,2);                          %卫星和客户对应的车辆
    logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');       %提取车辆的逻辑地址
    for i = 1:num_fleet
        fvlt1     = fvlt1+max(sum(capacity(logical_fleet(i,:)))-fleet(1,1),0);
    end
    fvlt1  = fvlt1+max(num_fleet-fleet(1,2),0);
    %----计算路径的运输代价---- 
    e_cost1 = num_fleet * 200;
    loc_depot = find(route<=num_dep);
    rback = route; rback(loc_depot(2:end)) = route(loc_depot(1:end-1));
    route2     = [rback(2:end),route(loc_depot(end))];
    c_cost1 = sum(dis_ds(route+(route2-1)*nds))*100;
end