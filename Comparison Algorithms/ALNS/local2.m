%% �ڶ���·���ľֲ�����----
function seq_c = local2(vrp2e,seq_c)
    %----5�־ֲ�����----
%     seq_c = split2(vrp2e,seq_c);
    seq_c = move2(vrp2e,seq_c); %���ɾ�����пͻ������²�������·�ζԱȽ��
    seq_c = swap2(vrp2e,seq_c); %��������пͻ����䵥һ+�����ھӽ�����ԱȽ��
    seq_c = opt2(vrp2e,seq_c);  %ÿ����·���ڣ�������ת������·��
    seq_c = opt2x(vrp2e,seq_c); %ƴ��ͬһ�����²�ͬ��·���ڵ���·��
    %----end----
end

%% --------------split2����:�ڶ���·���Ż�--------------------
function seq_cs = split2(vrp2e,route)
    fleet = vrp2e.fleet;
    demand = vrp2e.demand;  
    dis_sc = vrp2e.dis_sc;  
    num_sat = vrp2e.num_sat;  

    len_route = numel(route);
    seq_cs    = zeros(1,len_route+fleet(2,2));
    rg        = 0;
    %----����s������·��----
    sat_fleet     = route(route<=num_sat);                      %������Ӧ������
    sign_fleet    = cumsum(route<=num_sat,2);                    %���ǺͿͻ���Ӧ�ĳ���
    for s = 1:num_sat
        sign_fleet_s        = find(sat_fleet==s);                 %����s��Ӧ�ĳ�������
        [logic_loc,~]       = ismember(sign_fleet,sign_fleet_s);  %����s����Ӧ�ͻ�����Ϊ1
        cus_sat             = route(logic_loc);                   %����s������·������s��
        route_s             = cus_sat(cus_sat>num_sat);                %��ȡ·���еĿͻ��㣨����s��
        len_s               = numel(sign_fleet_s);
        if len_s >1
            seq_c           = splitopt(dis_sc(s,route_s),dis_sc(route_s,route_s),demand(route_s),fleet(2,1));
            loc_c           = find(seq_c>0);
            seq_c(loc_c)    = route_s(seq_c(loc_c));
            seq_c(seq_c==0) = s;
            len_r           = numel(seq_c);
            seq_cs(rg+1:rg+len_r) = seq_c;
            rg              = rg+len_r;
        else
            len_r           = numel(route_s)+1;
            seq_cs(rg+1:rg+len_r) = [s,route_s];
            rg              = rg+len_r;
        end
    end
    seq_cs(rg+1:end) = [];
end
% ----����split�㷨��·���������·ָ�----
 function seq_c = splitopt(dis_sc,dis_cc,require,capcity)
    nc        = length(dis_sc);                  %��ѡȡ��������Ŀ
    point     = zeros(1,nc);                    %ÿ���ͻ���ǰ��ָ��
    cost_c    = inf*ones(1,nc);                 %����ǰ�ͻ���Ҫ��·������
    for i = 1:nc
        load  = 0;                              %��ǰ�ĳ�������
        cost  = 2*dis_sc(i);                    %��ǰ��·������
        j     = i;
        while j<=nc && load<=capcity
            load  = load+require(j);
            if j>i
                cost = cost-dis_sc(j-1)+dis_cc(j-1,j)+dis_sc(j); %���β������Ǵ���
            end
            if load<=capcity  %�����Ƿ������ж�
                if i-1 ==0
                    cost_c(j) = cost;
                    point(j)  = i-1;
                elseif cost_c(i-1)+cost<cost_c(j)  %�������ȴ��۱Ƚ�
                    cost_c(j) = cost_c(i-1)+cost;
                    point(j)  = i-1;
                end
                j = j+1;
            end
        end
    end
    %----����ָ����·���в������ǵ�----
    seq_c = [1:nc,zeros(1,nc)];
    r     = 1;                        %ͳ��·������Ŀ
    i     = point(nc);                %�������ǵ��λ��
    seq_c(i+2:end) = seq_c(i+1:end-1);
    seq_c(i+1) = 0;
    while i>0
        i = point(i);
        r = r+1;
        seq_c(i+2:end) = seq_c(i+1:end-1);
        seq_c(i+1) = 0;
    end
    seq_c(nc+r+1:end) = [];
 end
 
%% --------------------------2-opt����:�ڶ���·���Ż�(ͬһ·���е������Ż�)-------------------------
function seq_cs = opt2(vrp2e,seq_cs)
    num_sat = vrp2e.num_sat;
    loc_s = [find(seq_cs<=num_sat),numel(seq_cs)+1];         %���Ƕ�Ӧ�����
    nfs   = length(loc_s)-1;                            %�������� 
    for i = 1:nfs
        if loc_s(i+1)-loc_s(i)>3                        %��Ҫ�Ż����������ͻ���Ŀ>2
           seq_cs(loc_s(i):loc_s(i+1)-1) = opttwo(vrp2e,[seq_cs(loc_s(i):loc_s(i+1)-1),seq_cs(loc_s(i))]);  %��һ��ÿ���ó��еĿͻ����н���
        end
    end
end
%------------����2-opt���Խ�ͬһ��·������------------------
function route = opttwo(vrp2e,route)  %��ת������·��
    Energy_distribution = funcopt2(vrp2e,route);
    len_route = numel(route);   %�ͻ���Ŀ
    i         = 2;
    while i <= len_route-2
        j = i+1;
        while j <= len_route-1
            new_route = [route(1:i-1), route(j:-1:i), route(j+1:end)];
            newEnergy_distribution = funcopt2(vrp2e,new_route);
            diff_dis = sum(Energy_distribution)-sum(newEnergy_distribution);  %������·�������ܺ�
            if diff_dis > 0
                Energy_distribution = newEnergy_distribution;
                route(i:j) = route(j:-1:i); 
                i = 2;
                j = i+1;
            else
                j = j+1;
            end
        end
        i = i+1;
    end
    route = route(1:end-1);              %�����µ�·��
end
%% --------------------------2-opt*����:�ڶ���·���Ż�-------------------------
function seq_cs = opt2x(vrp2e,seq_cs)
    num_sat = vrp2e.num_sat;
    
    len_route   = numel(seq_cs);
    loc_s = [find(seq_cs<=num_sat),len_route+1];              %���Ƕ�Ӧ�����
    nfs     = length(loc_s)-1;                   %��������
    route   = cell(nfs,1);                       %��ÿ��·�������Ĵ洢
    for i = 1:nfs
        route{i} = [seq_cs(loc_s(i):loc_s(i+1)-1),seq_cs(loc_s(i))];
    end
    %----2-opt*----
    for i = 1:nfs-1
        for j = i+1:nfs
            if route{i}(1)==route{j}(1) && numel(route{i})>2 && numel(route{j})>2
               [route{i},route{j}] = opttwox(vrp2e,route{i},route{j});  %ע�⽻��ͬ�����³�����·���еĿͻ�
            end
        end
    end
    %----·������----
    r = 0;
    for i = 1:nfs
        len = numel(route{i})-1;
        if len>1
            seq_cs(r+1:r+len) = route{i}(1:len);  %���¿ͻ�����������
            r = r+len;
        end
    end
    seq_cs(r+1:end) = [];
end
%------------����2-opt*���Խ�ͬһ���ǵ�����·������------------------
function [seq_c1,seq_c2] = opttwox(vrp2e,seq_c1,seq_c2)
    nc1 = numel(seq_c1);
    nc2 = numel(seq_c2);
    fr  = funcopt2(vrp2e,seq_c1)+funcopt2(vrp2e,seq_c2);  %����ԭʼ����·�����ܾ���
    i   = 1;
    while i <= nc1-1  %ʹ�ñ���i������һ��·��seq_c1�е����пͻ��������������һ���ͻ�
        j = 1;
        while j <= nc2-1    %ʹ�ñ���j�����ڶ���·��seq_c2�е����пͻ���ͬ�����������һ���ͻ�
            %----��һ����Ϸ�ʽ----
            ra1 = [seq_c1(1:i),seq_c2(j+1:end)];  %��seq_c1��ǰi���ͻ���seq_c2�Ĵ�j+1��ĩβ�Ŀͻ���ϳ��µ�·��ra1
            ra2 = [seq_c2(1:j),seq_c1(i+1:end)];  %��seq_c2��ǰj���ͻ���seq_c1�Ĵ�i+1��ĩβ�Ŀͻ���ϳ��µ�·��ra2
            fra    = funcopt2(vrp2e,ra1)+funcopt2(vrp2e,ra2);  %�ɱ����㷽��������Լ��Υ�����
            %----�ڶ�����Ϸ�ʽ----
            rb1 = [seq_c1(1:i),seq_c2(j:-1:1)];  %��seq_c1��ǰi���ͻ���seq_c2�Ĵ�j����һ���ͻ�������ϳ��µ�·��rb1
            rb2 = [seq_c1(end:-1:i+1),seq_c2(j+1:end)];  %��seq_c1�Ĵ����һ���ͻ�����i+1�Ŀͻ���seq_c2�Ĵ�j+1��ĩβ�Ŀͻ���ϳ��µ�·��rb2
            frb    = funcopt2(vrp2e,rb1)+funcopt2(vrp2e,rb2);  %�ɱ����㷽��������Լ��Υ�����
            %----�ж�����---- ÿ����Ϸ�ʽ�����µ��ܾ��루fra��frb��
            if fra <= frb
                if fra < fr
                    seq_c1 = ra1;
                    seq_c2 = ra2;
                    fr     = fra;
                    i      = 1;
                    j      = 0;
                end
            else
                if frb < fr
                    seq_c1 = rb1;
                    seq_c2 = rb2;
                    fr     = frb;
                    i      = 1;
                    j      = 0;
                end
            end
            nc1    = numel(seq_c1);
            nc2    = numel(seq_c2);
            j = j+1;    
       end
       i = i+1;    
    end
end
 %% --------------------------move2����:�ڶ���·���Ż�-------------------------
function route = move2(vrp2e,route)
    neib_cc = vrp2e.neib_cc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    tao     = min(25,num_cus-1);
    %----������Ŀͻ��������----
    seq_c   = randperm(num_cus)+num_sat;   %���ͻ���һ��λ���ƶ�����һ��λ��������·��

    for c = seq_c
        loc_c      = find(route==c);                            %ɾ���ͻ����ڽ��е�λ��
        %----����ԭ�ͻ��Ĵ���----
        cost_init = funcopt4(vrp2e,route);
        seq_r         = [route(1:loc_c-1),route(loc_c+1:end)];
        neib_c        = neib_cc(c-num_sat,2:tao+1);               %�ھӿͻ�
        [~,pos_in]    = ismember(neib_c,seq_r);                   %�ھӿͻ���·���е�λ�ã�������

        % ������뵽 rfront �����·��
        c_front = arrayfun(@(k) [seq_r(1:k-1), c, seq_r(k:end)], pos_in, 'UniformOutput', false);
        cost_front = arrayfun(@(k) [funcopt4(vrp2e,c_front{k})], 1:length(c_front), 'UniformOutput', false);
        % ������뵽 rback �����·��
        c_back = arrayfun(@(k) [seq_r(1:k), c, seq_r(k+1:end)], pos_in, 'UniformOutput', false);
        cost_back = arrayfun(@(k) [funcopt4(vrp2e,c_back{k})], 1:length(c_back), 'UniformOutput', false);
        [cost_c,loc]  = min([cell2mat(cost_front);cell2mat(cost_back)],[],1); %cost_c�洢�����ھ�Xǰ�����С�ɱ���loc�洢�����ھ�Xǰ���Ǻ�
        [cost_new,loc_in] = min(cost_c);  %��С�ھӵĳɱ��͸��ھ����
        if cost_new < cost_init
            loc_in       = pos_in(loc_in)+loc(loc_in)-2;
            route        = [seq_r(1:loc_in),c,seq_r(loc_in+1:end)];
        end
    end
    %----������·��----
    len_route   = numel(route);                                 %���е�·������
    loc_sat     = [find(route<=num_sat),len_route+1];           %������·���е�λ��
    route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��    
end

%% --------------------------swap2����:�ڶ���·���Ż�-------------------------
function route = swap2(vrp2e,route)
    neib_cc = vrp2e.neib_cc; 
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;

    tao     = min(25,num_cus-1);  %�����ھӿͻ�����tao
    %----������Ŀͻ��������----
    seq_c      = randperm(num_cus)+num_sat;  %�������һ��������Ŀͻ�����seq_c
    sign_fleet = cumsum(route<=num_sat,2);               %���ǺͿͻ���Ӧ�ĳ���
    %----�����ٽ��㽻��----
    for c1 = seq_c
        loc_c1 = find(route==c1);  %�ҵ��ͻ�c1��·���е�λ��loc_c1
        i = 2;
        while i <= tao+1
            c2     = neib_cc(c1-num_sat,i);  %���ڿͻ�c1��ÿ���ھӿͻ�c2
            loc_c2 = find(route==c2);
            route2 = route;
            route2(loc_c1) = route(loc_c2);  %����c1��c2��·���е�λ��
            route2(loc_c2) = route(loc_c1);
            fun1 = funcopt1(vrp2e,route(sign_fleet==sign_fleet(loc_c1)))+funcopt1(vrp2e,route(sign_fleet==sign_fleet(loc_c2)));  %���㽻��ǰ���·���ɱ�
            fun2 = funcopt1(vrp2e,route2(sign_fleet==sign_fleet(loc_c1)))+funcopt1(vrp2e,route2(sign_fleet==sign_fleet(loc_c2)));
            if fun2 < fun1
                route  = route2;
                loc_c1 = find(route==c1);    %�����ھӿͻ�λ�ã����¿�ʼ��������
                i     = 1;
            end
            i = i+1;
        end
    end
    %----�����ٽ��㽻��----
    for c1 = seq_c
        loc_c1     = find(route==c1);
        [~,pos_in] = ismember(neib_cc(c1-num_sat,2:tao+1),route);  %�ھӿͻ���·���е�λ�ã���������
        pos_in     = sort(pos_in);
        i          = 1;
        while i < tao
            if pos_in(i+1)-pos_in(i)==1
                fun1 = funcopt1(vrp2e,route(sign_fleet==sign_fleet(loc_c1)))+funcopt1(vrp2e,route(sign_fleet==sign_fleet(pos_in(i))));
                if loc_c1<pos_in(i)  %���ڿͻ�c1��ÿ�������ھӿͻ�
                    route2 = [route(1:loc_c1-1),route(pos_in(i):pos_in(i+1)),route(loc_c1+1:pos_in(i)-1),route(loc_c1),route(pos_in(i+1)+1:end)]; %����c1����������ھӿͻ���·���е�λ��
                else
                    route2 = [route(1:pos_in(i)-1),route(loc_c1),route(pos_in(i)+2:loc_c1-1),route(pos_in(i):pos_in(i+1)),route(loc_c1+1:end)];
                end
                sign_fleet2 = cumsum(route2<=num_sat,2);                 %���ǺͿͻ���Ӧ�ĳ���
                fun2 = funcopt1(vrp2e,route2(sign_fleet2==sign_fleet2(pos_in(i))))+funcopt1(vrp2e,route2(sign_fleet2==sign_fleet2(loc_c1)));
                if fun2 <fun1
                    route      = route2;
                    sign_fleet = sign_fleet2;
                    loc_c1     = find(route==c1);
                    [~,pos_in] = ismember(neib_cc(c1-num_sat,2:tao+1),route);  %�ھӿͻ���·���е�λ�ã���������
                    pos_in     = sort(pos_in);
                    i          = 0;
                end
            end
            i = i+1;
        end
    end
end
 

%% ----��·����Ӧֵ���������ǳ��أ�����Ϊ���ǵ�,βΪ�ͻ���----
function fvrp = funcopt1(vrp2e,route)
    fleet = vrp2e.fleet;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    cap_sc = energy(vrp2e,[route,0]);
    loc_sat = find(route<=num_sat); 
    rback   = route; rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
    route2  = [rback(2:end),route(loc_sat(end))];
    Dis_route = dis_sc(route+(route2-1)*(num_sat+num_cus));
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
    Energy_distribution = Energy_route.*Dis_route./vrp2e.V; 
    Energy_sat = arrayfun(@(i) sum(Energy_distribution(loc_sat(i):loc_sat(i+1)-1)), 1:length(loc_sat)-1);
    Energy_sat = [Energy_sat, sum(Energy_distribution(loc_sat(end):end))];
    penalty  = 100000000;
    fvrp     = sum(Energy_distribution)+penalty*(max(max(cap_sc - fleet(2,1)),0) + max(max(Energy_sat-vrp2e.E),0));
end
%% ----��·����Ӧֵ���������ǳ��أ�����βΪ���ǵ�----
function fvrp = funcopt2(vrp2e,route)
    fleet = vrp2e.fleet;
    dis_sc = vrp2e.dis_sc;
    cap_sc = energy(vrp2e,route);  %��ǰ��·�ε�ʵʱ��������
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);%��ǰ��·�ε�ʵʱ���ع���
    Dis_route = dis_sc(sub2ind(size(dis_sc), route(1:end-1), route(2:end)));
    Energy_distribution = Energy_route.*Dis_route./vrp2e.V;
    penalty  = 100000000;
    fvrp     = sum(Energy_distribution)+penalty*(max(max(cap_sc - fleet(2,1)),0) + max(sum(Energy_distribution)-vrp2e.E,0));
end

%% ----����·����Ӧֵ���������ǳ��أ�����βΪ�ͻ�----
function fvrp = funcopt4(vrp2e,route)
    fleet = vrp2e.fleet;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    cap_sc = energy(vrp2e,[route,0]);
    loc_sat = find(route<=num_sat); 
    rback   = route; rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
    route2  = [rback(2:end),route(loc_sat(end))];
    Dis_route = dis_sc(route+(route2-1)*(num_sat+num_cus));
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
    Energy_distribution = Energy_route.*Dis_route./vrp2e.V; 
    Energy_sat = arrayfun(@(i) sum(Energy_distribution(loc_sat(i):loc_sat(i+1)-1)), 1:length(loc_sat)-1);
    Energy_sat = [Energy_sat, sum(Energy_distribution(loc_sat(end):end))];
    penalty  = 100000000;
    fvrp     = sum(Energy_distribution)+penalty*(max(max(cap_sc - fleet(2,1)),0) + max(max(Energy_sat-vrp2e.E),0));
end