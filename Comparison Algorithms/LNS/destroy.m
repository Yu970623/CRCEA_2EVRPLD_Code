%% --------------------------�ڶ���·���ֲ��ƻ�-------------------------
function [seq_cs,seq_c,s_off,sm] = destroy(vrp2e,seq_cs,ps,s_off,gter,gmax)
    [seq_cs,seq_c1]          = remove1(vrp2e,seq_cs,ps(1));  %���ĳ���ͻ��ٽ��ļ����ͻ����·����ɾ�����Ŀͻ�
    [seq_cs,seq_c2]          = remove2(vrp2e,seq_cs,ps(2));  %ɾ����������ɱ��ϸߵĿͻ������̶ģ�
    [seq_cs,seq_c3]          = remove3(vrp2e,seq_cs,ps(3));  %���ɾ����������·��
    [seq_cs,seq_c4]          = remove4(vrp2e,seq_cs,ps(4));  %���ɾ��ĳ���ͻ�
    [seq_cs,seq_c5,sm,s_off] = remove5(vrp2e,seq_cs,s_off,gter,gmax,ps(5));
    seq_c = [seq_c1,seq_c2,seq_c3,seq_c4,seq_c5];  %��������ɾ������ɾ���Ŀͻ�
end

%----1���ɾ���ٽ�ѡȡ�ͻ���rc��ndl���ͻ���̰������:Related Removal----
function [seq_cs,seq_c] = remove1(vrp2e,seq_cs,p1)
    neib_cc = vrp2e.neib_cc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;
    
    ndl           = ceil(rand*p1*num_cus);       %���ɾ���Ŀͻ���Ŀ
    rc            = ceil(rand*num_cus);          %���ѡȡһ���ͻ���
    seq_c         = neib_cc(rc,1:ndl);           %ѡȡ��ͻ��ڽ���ndl���ͻ� 
    [~,loc_c]     = ismember(seq_c,seq_cs);      %����ͻ���·���е�λ��
    seq_cs(loc_c) = [];
    len_route     = numel(seq_cs);                               %���е�·������
    loc_sat       = [find(seq_cs<=num_sat),len_route+1];         %������·���е�λ��
    seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
end

%----2ɾ��������۽ϸߵĿͻ�: WorstRemoval----
function [seq_cs,seq_c] = remove2(vrp2e,seq_cs,p2)
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;
    loc_cus       = find(seq_cs>num_sat);           %�ͻ���·���е�λ��
    ndl           = ceil(rand*p2*length(loc_cus));  %���������Ҫɾ���Ŀͻ���
    cost_route = funcopt3(vrp2e,seq_cs);
    no_cus = arrayfun(@(i) [seq_cs(1:i-1), seq_cs(i+1:end)], loc_cus, 'UniformOutput', false);
    cost_cus = arrayfun(@(i) cost_route-funcopt3(vrp2e,no_cus{i}), 1:length(no_cus), 'UniformOutput', false);
    cost_cus = cell2mat(cost_cus);
    %     cus_front     = seq_cs(loc_cus-1);              %���пͻ���ǰһ���ͻ�������ɾ��ÿ��·�����һ���ͻ���
%     rback         = seq_cs;
%     loc_sat       = find(seq_cs<=num_sat);
%     rback(loc_sat(2:end)) = seq_cs(loc_sat(1:end-1));
%     route2        = [rback(2:end),seq_cs(loc_sat(end))];
%     cus_back      = route2(loc_cus);                %���пͻ���ǰһ���ͻ�������ɾ��ÿ��·�ε�һ���ͻ���
%     nsc           = num_sat+num_cus;
%     cost_cus      = dis_sc(cus_front+(seq_cs(loc_cus)-1)*nsc)+dis_sc(seq_cs(loc_cus)+(cus_back-1)*nsc)-dis_sc(cus_front+(cus_back-1)*nsc);
    del_c         = zeros(1,ndl);
    for i = 1:ndl
        cl           = roulette(cost_cus,1);
        del_c(i)     = loc_cus(cl);
        cost_cus(cl) = 0;
    end
    seq_c     = seq_cs(del_c);
    seq_cs(del_c) = [];
    len_route     = numel(seq_cs);                               %���е�·������
    loc_sat       = [find(seq_cs<=num_sat),len_route+1];         %������·���е�λ��
    seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
end
    
%----3ɾ��k��·����������--------
function [seq_cs,seq_c] = remove3(vrp2e,seq_cs,p3)
    demand = vrp2e.demand;
    fleet = vrp2e.fleet;
    num_sat = vrp2e.num_sat;
    k          = ceil(rand*p3*ceil(sum(demand)/fleet(2,1)));   %��ɾ����·����Ŀ
    sign_fleet = cumsum(seq_cs<=num_sat,2);                    %���ǺͿͻ���Ӧ�ĳ���
    if sign_fleet(end) > k
        f_del  = ceil(rand(1,sign_fleet(end))*k);              %��ɾ����·�����
    else
        f_del  = 1:sign_fleet(end);
    end
    log_cs  = ismember(sign_fleet,f_del);      %���Ӧ��·���Ƿ�ɾ��
    seq_c   = seq_cs(log_cs);                  %��ȡ��ɾ����·��
    seq_c(seq_c<=num_sat) = [];                %��ȡ·���еĿͻ�
    seq_cs(log_cs) = [];                       %����·����ɾ������·��
end 

%----4�Ը���ɾ������·����Route Removal------------
function [seq_cs,seq_c] = remove4(vrp2e,seq_cs,p4)
    num_sat = vrp2e.num_sat;
    
    if rand < p4
        len_route = length(seq_cs);
        loc_s     = fliplr([find(seq_cs<=num_sat),len_route+1]);    %�����ڽ��ж�Ӧ��λ��
        nr        = numel(loc_s)-1;
        seq_c     = zeros(1,len_route);
        for i = 1:nr
            if loc_s(i+1)-loc_s(i)==2
                seq_c(loc_s(i)+1:loc_s(i+1)-1) = seq_cs(loc_s(i)+1:loc_s(i+1)-1);
                seq_cs(loc_s(i):loc_s(i+1)-1)  = [];
            end
        end
        seq_c(seq_c==0)  = [];
    else
        seq_c = [];
    end
end  
%----5�Ը��ʹر�һ�����ǻ����������ǲ��ж��Ƿ���Թر�------------
function [seq_cs,seq_c,sm,s_off] = remove5(vrp2e,seq_cs,s_off,gter,gmax,p5)
    num_sat = vrp2e.num_sat;
    
    sm    = 0;
    seq_c = [];
    if gter > gmax
        rnd = rand();
        if rnd < p5
            num_on  = sum(s_off);
            if num_on > 1
                sm                = 1;
                loc_on            = find(s_off==1);                    %���ŵ�����
                s_sel             = loc_on(ceil(rand*num_on));         %ѡ��һ�����ǹر�
                s_off(s_sel)      = 0;                                 %��ѡȡ�����ǹر�
                sat_fleet         = seq_cs(seq_cs<=num_sat);           %������Ӧ������
                sign_fleet_s      = find(sat_fleet==s_sel);            %����s��Ӧ�ĳ�������
                sign_fleet        = cumsum(seq_cs<=num_sat,2);         %���ǺͿͻ���Ӧ�ĳ���
                [logic_loc,~]     = ismember(sign_fleet,sign_fleet_s); %����s����Ӧ�ͻ�����Ϊ1
                cus_sat           = seq_cs(logic_loc);                 %����s������·������s��
                seq_c             = cus_sat(cus_sat>num_sat);          %��ȡ·���еĿͻ��㣨����s��
                seq_cs(logic_loc) = [];                                %ɾ��s_off��Ӧ������·��
            end
        elseif rnd < p5*(1.0+1.0/num_sat)
            sm    = 1;
            s_off = 1;           %������������
        end
    end
end

function fvrp = funcopt3(vrp2e,route)
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    cap_sc = energy(vrp2e,route);
    loc_sat = find(route<=num_sat); 
    rback   = route; rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
    route2  = [rback(2:end),route(loc_sat(end))];
    Dis_route = dis_sc(route+(route2-1)*(num_sat+num_cus));
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
    Energy_distribution = Energy_route.*Dis_route./vrp2e.V; 
    fvrp = sum(Energy_distribution);
end