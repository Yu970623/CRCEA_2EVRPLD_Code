function cap_sc = energy(vrp2e,route)
    demand = vrp2e.demand; %获取客户的需求量
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    cus_type = vrp2e.type;
    route = route(1:end-1);
    cap_s = zeros(1,2*num_cus);         %初始化路段实时重量
    loc_sat    = find(route<=num_sat);
    for i = 1:length(route)
        if ismember(i, loc_sat)
            cap_s(i) = 0;
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