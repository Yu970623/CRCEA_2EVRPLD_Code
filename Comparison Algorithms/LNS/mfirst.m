%% ---------------------�����һ���vrpֵ---------------------------
%seq_sat(:,i),cap_sat(:,i)��Ӧ�����һ��ĵ�i���⣬penalty1Ϊ��һ�㳬�������ĳͷ���
function [e_cost1,c_cost1,fvlt1] = mfirst(vrp2e,route,capacity)
    dis_ds = vrp2e.dis_ds;
    fleet = vrp2e.fleet;
    num_sat = vrp2e.num_sat;
    num_dep = length(dis_ds)-num_sat;
    fvlt1 = 0;
    nds    = size(dis_ds,1);
    %----������·��----
    loc_del            = find(route(2:end)-route(1:end-1)==0);   %��·�����ڵ�λ��
    route (loc_del)    = [];
    capacity (loc_del) = [];
    %----����·����Ŀ������----
    num_fleet     = sum(route<=num_dep);                               %·����Ŀ(�������ճ�����
    sign_fleet    = cumsum(route<=num_dep,2);                          %���ǺͿͻ���Ӧ�ĳ���
    logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');       %��ȡ�������߼���ַ
    for i = 1:num_fleet
        fvlt1     = fvlt1+max(sum(capacity(logical_fleet(i,:)))-fleet(1,1),0);
    end
    fvlt1  = fvlt1+max(num_fleet-fleet(1,2),0);
    %----����·�����������---- 
    e_cost1 = num_fleet * 200;
    loc_depot = find(route<=num_dep);
    rback = route; rback(loc_depot(2:end)) = route(loc_depot(1:end-1));
    route2     = [rback(2:end),route(loc_depot(end))];
    c_cost1 = sum(dis_ds(route+(route2-1)*nds))*100;
end