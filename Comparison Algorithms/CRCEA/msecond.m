%% ---------------------------����ڶ����vrpֵ---------------------
function [cost2,fvlt2] = msecond(vrp2e,route)
    %global dis_sc; global demand; global fleet; global num_sat; global num_cus;
    fleet = vrp2e.fleet;
    dis_sc = vrp2e.dis_sc;  
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus; 
    V = vrp2e.V;
    E = vrp2e.E;
    num_pop = size(route,1);
    cost2   = zeros(1,num_pop);
    fvlt2   = zeros(1,num_pop);
    for i = 1:num_pop
         %----������·��----
        seq_cus = route(i,:);
        seq_cus(seq_cus(1:1:end) == 0)=[];
        len_route     = numel(seq_cus);                                 %���е�·������
        loc_sat1      = [find(seq_cus<=num_sat),len_route+1];           %������·���е�λ��
        seq_cus(loc_sat1(loc_sat1(2:end)-loc_sat1(1:end-1)==1)) = [];      %ɾ���ճ�������·��
        %----����·����Ŀ������----
        num_fleet     = sum(seq_cus<=num_sat);                               %·����Ŀ(�������ճ�����
    %     sign_fleet    = cumsum(route<=num_sat,2);                          %���ǺͿͻ���Ӧ�ĳ���
    %     logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');             %��ȡ�������߼���ַ
        loc_sat = find(seq_cus<=num_sat);     
        cap_sc = energy(vrp2e,[seq_cus,seq_cus(loc_sat(end))]);
        rback   = seq_cus; rback(loc_sat(2:end)) = seq_cus(loc_sat(1:end-1));
        route2  = [rback(2:end),seq_cus(loc_sat(end))];
        Dis_route = dis_sc(seq_cus+(route2-1)*(num_sat+num_cus));
        Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
        Energy_distribution = Energy_route.*Dis_route./V; 
        Energy_sat = arrayfun(@(i) sum(Energy_distribution(loc_sat(i):loc_sat(i+1)-1)), 1:length(loc_sat)-1);
        Energy_sat = [Energy_sat, sum(Energy_distribution(loc_sat(end):end))];
        fvlt2(i) = max(max(cap_sc - fleet(2,1)),0) + max(max(Energy_sat-E),0) + max(num_fleet-fleet(2,2),0);  %����Լ��������Լ��������Լ��
        %----����·�����������---- 
        cost2(i) = sum(Energy_distribution)+num_fleet*100; %���˻��豸�ɱ�
    end
end
