%% ---------------------------����ڶ����vrpֵ---------------------
%seq_cus(:,i)��Ӧ����ڶ���ĵ�i���⣬penalty2Ϊ�ڶ��㳬�������ĳͷ���
function [e_cost2, c_cost2,fvlt2] = msecond(vrp2e,route)
    fleet = vrp2e.fleet;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    V = vrp2e.V;
    E = vrp2e.E;
    %----������·��----
    len_route     = numel(route);                                 %���е�·������
    loc_sat1       = [find(route<=num_sat),len_route+1];           %������·���е�λ��
    route(loc_sat1(loc_sat1(2:end)-loc_sat1(1:end-1)==1)) = [];      %ɾ���ճ�������·��
    %----����·����Ŀ������----
    num_fleet     = sum(route<=num_sat);                               %·����Ŀ(�������ճ�����
%     sign_fleet    = cumsum(route<=num_sat,2);                          %���ǺͿͻ���Ӧ�ĳ���
%     logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');             %��ȡ�������߼���ַ
    cap_sc = energy(vrp2e,route);
    loc_sat = find(route<=num_sat); 
    rback   = route; rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
    route2  = [rback(2:end),route(loc_sat(end))];
    Dis_route = dis_sc(route+(route2-1)*(num_sat+num_cus));
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
    Energy_distribution = Energy_route.*Dis_route./V; 
    Energy_sat = arrayfun(@(i) sum(Energy_distribution(loc_sat(i):loc_sat(i+1)-1)), 1:length(loc_sat)-1);
    Energy_sat = [Energy_sat, sum(Energy_distribution(loc_sat(end):end))];
    fvlt2 = max(max(cap_sc - fleet(2,1)),0) + max(max(Energy_sat-E),0) + max(num_fleet-fleet(2,2),0);  %����Լ��������Լ��������Լ��
    %----����·�����������---- 

    e_cost2 = num_fleet*100; %���˻��豸�ɱ�
    c_cost2 = sum(Energy_distribution);%���˻��ܺĳɱ�
    %����ÿ���������������������Լ��
    for i = 1:num_sat
        fvlt2 = fvlt2 + max(sum(route2==i)-vrp2e.maxFleetPerSat,0);
    end
end