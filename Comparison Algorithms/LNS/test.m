clc;clear;
seq   = [1,2,3,4,5,6,7,8, 9,10,11,12];
route = [1,5,9,7,2,8,6,3,10,11, 2,12,3,4];
demand= [0,0,0,0,1,2,3,4,5,6,7,8];
seq_c = [9,11,5];
s  = 2;
f  = 3;
ns = 4;
%1----消除空路径----
ns = 4;
len_route   = numel(route);                                 %已有的路径长度
loc_sat     = [find(route<=ns),len_route+1];                %卫星在路径中的位置
route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径
route

%----客户的信息----
[~,pos_cus] = ismember(seq_c,route);      %多个客户在路径中的位置
% pos_cus     = find(route==seq_c);       %单个客户在路径中的位置
pos_cus

%指定客户所在的车辆
[~,pos_cus] = ismember(seq_c,route);      %客户在路径中的位置
sign_fleet  = cumsum(route<=ns,2);        %卫星和客户对应的车辆
fleet_cus   = sign_fleet(pos_cus);
fleet_cus

%所有客户对应的车辆号（前面包含了卫星)
[~,pos_cus] = find(route>ns);           %客户在路径中的位置
sign_fleet  = cumsum(route<=ns,2);      %卫星和客户对应的车辆
fleet_cus   = sign_fleet(pos_cus);       %客户的逻辑位置对应的车辆号
fleet_cus(route(pos_cus)-ns) = fleet_cus;   %所有客户对应的车辆号
fleet_cus

%指定客户seq_c对应的卫星
[~,pos_cus] = ismember(seq_c,route);       %客户在路径中的位置
sign_fleet  = cumsum(route<=ns,2);           %卫星和客户对应的车辆
fleet_cus   = sign_fleet(pos_cus);         %指定客户对应的车辆号
sat_fleet   = route(route<=ns);             %车辆对应的卫星
sat_cus     = sat_fleet(fleet_cus);        %客户seq_c对应的卫星
sat_cus

%所有客户对应的卫星
[~,pos_cus] = find(route>ns);          %客户在路径中的位置
sign_fleet  = cumsum(route<=ns,2);     %卫星和客户对应的车辆
fleet_cus   = sign_fleet(pos_cus);     %客户的逻辑位置对应的车辆号
fleet_cus(route(pos_cus)-ns) = fleet_cus;   %所有客户对应的车辆号
sat_fleet   = route(route<=ns);             %车辆对应的卫星
sat_cus     = sat_fleet(fleet_cus); 
sat_cus

%----卫星的信息(可能一个卫星对应多个车辆)----
pos_sat    = find(route<=ns);             %卫星在路径中的位置
pos_sat

%卫星s对应的车辆号码
sat_fleet    = route(route<=ns);          %车辆对应的卫星
sign_fleet_s = find(sat_fleet==s);        %卫星s对应的车辆号码
sign_fleet_s

%卫星s的所有路径
sat_fleet     = route(route<=ns);                   %车辆对应的卫星
sign_fleet_s  = find(sat_fleet==s);                 %卫星s对应的车辆号码
sign_fleet    = cumsum(route<=ns,2);                  %卫星和客户对应的车辆
[logic_loc,~] = ismember(sign_fleet,sign_fleet_s);  %卫星s及对应客户参数为1
cus_sat       = route(logic_loc);                   %卫星s的所有路径（含s）
% cus_sat       = cus_sat(cus_sat>ns);                 %提取路径中的客户点（不含s）
cus_sat

%----路径（车辆）信息-----
%路径（车辆）对应的卫星
pos_sat    = find(route<=ns);            %卫星在路径中的位置
sat_fleet  = route(pos_sat);             %路径对应的卫星
sat_fleet

%单个路径（车辆f）中的所有客户
sign_fleet = cumsum(route<=ns,2);     %卫星和客户对应的车辆
cus_fleet  = route((sign_fleet==f));  %第f条路径（含卫星）
cus_fleet

%计算各个路径的总的容量
num_fleet     = sum(route<=ns);          %路径数目除去空车辆
sign_fleet    = cumsum(route<=ns,2);     %卫星和客户对应的车辆
logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');  %提取车辆的逻辑地址
cap_fleet     = zeros(1,num_fleet);
for i = 1:num_fleet
    cap_fleet(i) = sum(demand(route(logical_fleet(i,:))));
end
cap_fleet

%所有解中逻辑的点（含卫星和客户）对应的车辆容量
num_fleet     = sum(route<=ns);          %路径数目除去空车辆
sign_fleet    = cumsum(route<=ns,2);     %卫星和客户对应的车辆
logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');  %提取车辆的逻辑地址
cap_fleet     = zeros(1,num_fleet);
for i = 1:num_fleet
    cap_fleet(i) = sum(demand(route(logical_fleet(i,:))));
end
cap_fleet

%路径中每一点对应的车辆容量
num_fleet     = sum(route<=ns);          %路径数目除去空车辆
sign_fleet    = cumsum(route<=ns,2);     %卫星和客户对应的车辆
logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');  %提取车辆的逻辑地址
cap_fleet     = zeros(1,num_fleet);
for i = 1:num_fleet
    cap_fleet(i) = sum(demand(route(logical_fleet(i,:))));
end
cap_sc = cap_fleet(sign_fleet);
cap_sc






