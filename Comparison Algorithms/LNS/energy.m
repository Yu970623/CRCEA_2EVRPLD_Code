function cap_sc = energy(vrp2e,route)
    fleet = vrp2e.fleet;  %获取车队信息，如每辆车的最大容量等。
    demand = vrp2e.demand; %获取客户的需求量
    dis_sc = vrp2e.dis_sc;  %卫星与客户之间的距离 
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    cus_type = vrp2e.type;

    num_fleet = sum(route<=num_sat);        %计算当前已分配的车辆数量
    sign_fleet = cumsum(route<=num_sat,2);   %卫星和客户对应的无人机（按行求累积和）
    cap_s = zeros(1,2*num_cus);         %初始化路段实时重量
    loc_sat    = find(route<=num_sat);
    nums_c = num_sat+num_cus;
    back = 0;
    for i = 1:length(route)
        if ismember(i, loc_sat)
            cap_s(i) = 0;
            back = 0;
        else
            if strcmp(cus_type(route(i)-num_sat), 'pickup')
                cap_s(i) = cap_s(i-1) + demand(route(i));
%                 back = back + demand(route(i));
            else
                cap_s(i) = cap_s(i-1);
                cap_s(loc_sat(find(loc_sat < i, 1, 'last')):i-1) = cap_s(loc_sat(find(loc_sat < i, 1, 'last')):i-1) + demand(route(i));
            end
        end
    end
    cap_sc = cap_s(1:length(route));
end