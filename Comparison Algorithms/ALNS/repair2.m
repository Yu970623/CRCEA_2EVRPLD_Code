%% --------------------------�ڶ���·���Ż�-------------------------
function seq_r = repair2(vrp2e,seq_cs,seq_c,s_off,sm)
    if sm < 4  %(9-11)
        seq_r = greedyin2(vrp2e,seq_cs,seq_c,s_off);  %sm1=1Ϊ̰�����룻2Ϊ�Ŷ�̰�����룻3Ϊ��ֹ�ظ�̰������
    else
        seq_r = greedyinr2(vrp2e,seq_cs,seq_c,s_off);    %��ڲ���
    end
end



%% ----̰�������㷨----
%��seq_c�еĿͻ����뵽���еĶ���seq_cs�У�s_off���رջ��ֹ������
function seq_r = greedyin2(vrp2e,route,seq_c,s_off)
    fleet = vrp2e.fleet;  %��ȡ������Ϣ����ÿ��������������ȡ�
    demand = vrp2e.demand; %��ȡ�ͻ���������
    dis_sc = vrp2e.dis_sc;  %������ͻ�֮��ľ��� 
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    cus_type = vrp2e.type;
    E = vrp2e.E;  %����ܹ���
    V = vrp2e.V;    %���˻������ٶ�
    penalty = 10000000;  %����һ���ͷ�ֵ������Լ��������Υ�����
    %----���㵱ǰ·�����ѷ���ĳ������������ۻ�װ����----
    num_fleet     = sum(route<=num_sat);
    cap_s         = zeros(1,2*num_cus);                  %��ʼ��·��ʵʱ����
    nums_c           = num_sat+num_cus;
    len_route     = numel(route);
    seq_r         = zeros(1,2*num_cus); %����-�ͻ���Ӧ��ϵ��¼
    seq_r(1:len_route) = route;
    
    %----̰������----
    for c = seq_c    %���ݾ����Լ����������ÿ���ͻ����뵽·�� route �еĺ���λ��
        cost_newr        = 2*dis_sc(c,1:num_sat);   %��ǰ�ͻ��ֱ����������ǵľ���
        cost_newr(s_off) = inf;                     %��Ҫ�رյ����Ǿ�������Ϊ���޴�
        [cost,s]         = min(cost_newr);          % �ҳ���ǰ�ͻ�Ӧ�ò��������λ��
        if len_route > 0 
            route      = seq_r(1:len_route);        %����·��
            sign_fleet = cumsum(route<=num_sat,2);  %���ǺͿͻ���Ӧ�����˻����������ۻ��ͣ�
            loc_sat    = find(route<=num_sat);      %������·���е�λ��
             % �����Ǻͳ�������Ӧ·����¼
            rback      = route; 
            rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
            route2     = [rback(2:end),route(loc_sat(end))];    %�������ǵ�·��
            cap_sc = energy(vrp2e,[route,0]);
            power = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);

%           %���㵱ǰ����·�������ܺ�
            Energy_route = power.* dis_sc(route+(route2-1)*nums_c)./V;
            Energy = [arrayfun(@(i)[sum(Energy_route(loc_sat(i):loc_sat(i+1)-1))], 1:length(loc_sat)-1, 'UniformOutput', false),sum(Energy_route(loc_sat(end):end))];  %����ԭ·�������ܺ�
            Energy = arrayfun(@(x) Energy(x), sign_fleet);
            Energy = cell2mat(Energy);
            %����ȡ-������ֱ������������û�����ܵĴ���, ���˻�����+���������������Ƿ񳬹�Լ��
            if strcmp(cus_type(c-num_sat), 'pickup')
                %���뵽�ĸ�λ�ã���λ��֮���cap_sc+demand(c),֮ǰ�Ĳ���
                W_end = arrayfun(@(n) [cap_sc(loc_sat(end):n-1), demand(c) + cap_sc(n-1), cap_sc(n:end)+demand(c)], loc_sat(end)+1:length(cap_sc)+1, 'UniformOutput', false);
                W_all = [];
                for i = 1:length(loc_sat)-1
                    cap_pre = cap_sc(1:loc_sat(i)-1);
                    cap_i = cap_sc(loc_sat(i):loc_sat(i+1)-1);
                    cap_after = cap_sc(loc_sat(i+1):end);
%                     diff_i = diff_all{i};
                    W_i = arrayfun(@(n) [cap_i(1:n-1), demand(c) + cap_i(n-1), cap_i(n:end)+demand(c)], 2:length(cap_i)+1, 'UniformOutput', false);
                    W1 = cellfun(@(x) [cap_pre, x, cap_after], W_i, 'UniformOutput', false);
                    W_all = [W_all,W1];
                end
                cap_p = cap_sc(1:loc_sat(end)-1);
                W2 = cellfun(@(x) [cap_p, x], W_end, 'UniformOutput', false);
                W_distribution = [W_all,W2];
                cost = (5).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4)*cost./V;
            else
                %���뵽�ĸ�λ�ã���λ��֮ǰ��cap_sc+demand(c)��֮��Ĳ���
                W_end = arrayfun(@(n) [cap_sc(loc_sat(end):n-1)+demand(c), cap_sc(n-1), cap_sc(n:end)], loc_sat(end)+1:length(cap_sc)+1, 'UniformOutput', false);
                W_all = [];
                for i = 1:length(loc_sat)-1
                    cap_pre = cap_sc(1:loc_sat(i)-1);
                    cap_i = cap_sc(loc_sat(i):loc_sat(i+1)-1);
                    cap_after = cap_sc(loc_sat(i+1):end);
                    W_i = arrayfun(@(n) [cap_i(1:n-1)+demand(c), cap_i(n-1), cap_i(n:end)], 2:length(cap_i)+1, 'UniformOutput', false);
                    W1 = cellfun(@(x) [cap_pre, x, cap_after], W_i, 'UniformOutput', false);
                    W_all = [W_all,W1];
                end
                cap_p = cap_sc(1:loc_sat(end)-1);
                W2 = cellfun(@(x) [cap_p, x], W_end, 'UniformOutput', false);
                W_distribution = [W_all,W2];
                cost = (5+demand(c)).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4)*cost./V;
            end
            Max_W=cellfun(@(x) max(x(1:end)), W_distribution);  %��ȡ���в�����������ֵ������������ֵ��
            route_c = arrayfun(@(pos) [route(1:pos-1), c, route(pos:end)], 2:length(route)+1, 'UniformOutput', false);  %ǰ��·��
            loc_sat_c = cellfun(@(x) find(x <= num_sat), route_c, 'UniformOutput', false);    %����¿ͻ������������
            rback = cellfun(@(x) x, route_c, 'UniformOutput', false);
            for i = 1:length(route_c)% ���� rback �е�ÿ�� cell
                rback{i}(loc_sat_c{i}(2:end)) = route_c{i}(loc_sat_c{i}(1:end-1));
                route_c2{i} = [rback{i}(2:end),route_c{i}(loc_sat_c{i}(end))];
            end% 
            dis_route = cellfun(@(x, y) dis_sc(x + (y - 1) * nums_c), route_c, route_c2, 'UniformOutput', false);%����·����Ӧ�ľ��볤��
            power_route = cellfun(@(W) (5+W).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4), W_distribution, 'UniformOutput', false);   %ÿ��·�εĹ���
            Energy_distribution = cellfun(@(p, d) p.*d./V, power_route, dis_route, 'UniformOutput', false);  %���벻ͬλ�õĲ�ͬ�ܹ���
            
            group = [];
            if length(loc_sat)>1
                for i=1:length(loc_sat)-1
                    group{i} = loc_sat(i):loc_sat(i+1)-1;
                end
                group{length(group)+1} = loc_sat(end):length(Energy_distribution);
            else
                group = {1:length(Energy_distribution)};
            end
            
            for i = 1:length(group)
                if i<length(group)
                    for j = group{i}
                        Energy_c {j} = sum(Energy_distribution{j}(loc_sat_c{j}(i):loc_sat_c{j}(i+1)-1));
                    end
                else
                    for j = group{i}
                        Energy_c {j} = sum(Energy_distribution{j}(loc_sat_c{j}(end):end));
                    end
                end
            end
            punish = [penalty * (max(Max_W-fleet(2,1),0)+max(cell2mat(Energy_c)-E,0)),penalty * max(num_fleet+1-fleet(2,2),0)];
            cost_c     = [cell2mat(Energy_c)-Energy,cost*10]+punish;
            [~,loc_in] = min(cost_c);  % �ҳ���С���۵Ĳ���λ��
            if loc_in <= len_route  % �������λ���ڵ�ǰ·����Χ��
                % �ڲ���λ�ú��ƶ�����·���еĿͻ�
                seq_r(loc_in+2:len_route+1) = seq_r(loc_in+1:len_route);
                seq_r(loc_in+1) = c;
                len_route       = len_route+1;
                cap_s(1:numel(W_distribution{loc_in})) = W_distribution{loc_in};% ���г�����װ��������
            else
                % �������λ�ó�����ǰ·����Χ���򽫿ͻ������µĳ�����
                seq_r(len_route+1:len_route+2) = [s,c];
                if strcmp(cus_type(c-num_sat), 'delivery')
                    cap_s(len_route+1) = demand(c); % ��¼�³�����װ����
                else
                    cap_s(len_route+2) = demand(c);
                end
                len_route = len_route+2;
                num_fleet = num_fleet+1; % ����������һ
            end
        else
            cap_s         = zeros(1,2*num_cus);
            seq_r(1:2) = [s,c]; %��¼��ǰ�ͻ������ǵĶ�Ӧ��ϵ
            len_route  = 2;
            num_fleet  = num_fleet+1;   %ռ�õ����˻�����+1
            if strcmp(cus_type(c-num_sat), 'delivery')
                cap_s(1)   = demand(c);
            else
                cap_s(2)   = demand(c);
            end
        end
    end
    seq_r(len_route+1:end) = []; % ɾ������Ŀհײ���
end



%% ----��ڲ���----
function seq_r = greedyinr2(vrp2e, route, seq_c, s_off)
    fleet = vrp2e.fleet;
    demand = vrp2e.demand;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;
    cus_type = vrp2e.type;
    E = vrp2e.E;
    V = vrp2e.V;
    penalty = 10000000;

    % ���㳵�����غ�·����ʼ��
    cap_s = zeros(1, 2 * num_cus);
    nums_c = num_sat + num_cus;
    len_route = numel(route);
    seq_r = zeros(1, 2 * num_cus);
    seq_r(1:len_route) = route;
    num_c = numel(seq_c);
    num_fleet = sum(route <= num_sat);

    while num_c > 0
        % ��ڲ���ĳɱ�����
        cost_newr = 2 * dis_sc(seq_c, 1:num_sat);
        cost_newr(:, s_off) = inf;
        [cost, s_cus] = min(cost_newr, [], 2);

        if len_route > 0
            % ·���������ڲ���
            route = seq_r(1:len_route);
            sign_fleet = cumsum(route <= num_sat, 2);
            loc_sat = find(route <= num_sat);
            rback = route;
            rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
            route2 = [rback(2:end), route(loc_sat(end))];
            cap_sc = energy(vrp2e, [route, 0]);
            power = (5 + cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
            Energy_route = power .* dis_sc(route + (route2 - 1) * nums_c) ./ V;

            % ����·�����ܺ�
            Energy = [arrayfun(@(i) sum(Energy_route(loc_sat(i):loc_sat(i + 1) - 1)), 1:length(loc_sat) - 1), sum(Energy_route(loc_sat(end):end))];
            Energy = arrayfun(@(x) Energy(x), sign_fleet);
            if strcmp(cus_type(seq_c(num_c)-num_sat), 'pickup')
                W_end = arrayfun(@(n) [cap_sc(loc_sat(end):n-1), demand(seq_c(num_c)) + cap_sc(n-1), cap_sc(n:end)+demand(seq_c(num_c))], loc_sat(end)+1:length(cap_sc)+1, 'UniformOutput', false);
                W_all = [];
                for i = 1:length(loc_sat)-1
                    cap_pre = cap_sc(1:loc_sat(i)-1);
                    cap_i = cap_sc(loc_sat(i):loc_sat(i+1)-1);
                    cap_after = cap_sc(loc_sat(i+1):end);
%                     diff_i = diff_all{i};
                    W_i = arrayfun(@(n) [cap_i(1:n-1), demand(seq_c(num_c)) + cap_i(n-1), cap_i(n:end)+demand(seq_c(num_c))], 2:length(cap_i)+1, 'UniformOutput', false);
                    W1 = cellfun(@(x) [cap_pre, x, cap_after], W_i, 'UniformOutput', false);
                    W_all = [W_all,W1];
                end
                cap_p = cap_sc(1:loc_sat(end)-1);
                W2 = cellfun(@(x) [cap_p, x], W_end, 'UniformOutput', false);
                W_distribution = [W_all,W2];
                cost = (5).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4)*cost./V;
            else
                W_end = arrayfun(@(n) [cap_sc(loc_sat(end):n-1)+demand(seq_c(num_c)), cap_sc(n-1), cap_sc(n:end)], loc_sat(end)+1:length(cap_sc)+1, 'UniformOutput', false);
                W_all = [];
                for i = 1:length(loc_sat)-1
                    cap_pre = cap_sc(1:loc_sat(i)-1);
                    cap_i = cap_sc(loc_sat(i):loc_sat(i+1)-1);
                    cap_after = cap_sc(loc_sat(i+1):end);
                    W_i = arrayfun(@(n) [cap_i(1:n-1)+demand(seq_c(num_c)), cap_i(n-1), cap_i(n:end)], 2:length(cap_i)+1, 'UniformOutput', false);
                    W1 = cellfun(@(x) [cap_pre, x, cap_after], W_i, 'UniformOutput', false);
                    W_all = [W_all,W1];
                end
                cap_p = cap_sc(1:loc_sat(end)-1);
                W2 = cellfun(@(x) [cap_p, x], W_end, 'UniformOutput', false);
                W_distribution = [W_all,W2];
                cost = (5+demand(seq_c(num_c))).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4)*cost./V;
            end
            Max_W=cellfun(@(x) max(x(1:end)), W_distribution);  %��ȡ���в�����������ֵ������������ֵ��
            route_c = arrayfun(@(pos) [route(1:pos-1), seq_c(num_c), route(pos:end)], 2:length(route)+1, 'UniformOutput', false);  %ǰ��·��
            loc_sat_c = cellfun(@(x) find(x <= num_sat), route_c, 'UniformOutput', false);    %����¿ͻ������������
            rback = cellfun(@(x) x, route_c, 'UniformOutput', false);
            for i = 1:length(route_c)% ���� rback �е�ÿ�� cell
                rback{i}(loc_sat_c{i}(2:end)) = route_c{i}(loc_sat_c{i}(1:end-1));
                route_c2{i} = [rback{i}(2:end),route_c{i}(loc_sat_c{i}(end))];
            end% 
            dis_route = cellfun(@(x, y) dis_sc(x + (y - 1) * nums_c), route_c, route_c2, 'UniformOutput', false);%����·����Ӧ�ľ��볤��
            power_route = cellfun(@(W) (5+W).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4), W_distribution, 'UniformOutput', false);   %ÿ��·�εĹ���
            Energy_distribution = cellfun(@(p, d) p.*d./V, power_route, dis_route, 'UniformOutput', false);  %���벻ͬλ�õĲ�ͬ�ܹ���
            
            group = [];
            if length(loc_sat)>1
                for i=1:length(loc_sat)-1
                    group{i} = loc_sat(i):loc_sat(i+1)-1;
                end
                group{length(group)+1} = loc_sat(end):length(Energy_distribution);
            else
                group = {1:length(Energy_distribution)};
            end
            
            for i = 1:length(group)
                if i<length(group)
                    for j = group{i}
                        Energy_c {j} = sum(Energy_distribution{j}(loc_sat_c{j}(i):loc_sat_c{j}(i+1)-1));
                    end
                else
                    for j = group{i}
                        Energy_c {j} = sum(Energy_distribution{j}(loc_sat_c{j}(end):end));
                    end
                end
            end
            punish = [penalty * (max(Max_W-fleet(2,1),0)+max(cell2mat(Energy_c)-E,0)),penalty * max(num_fleet+1-fleet(2,2),0)];
            cost_c = [cell2mat(Energy_c)-Energy,cost(num_c)*10] + punish;

            [val_cost, srt_c] = sort(cost_c, 2);
            [~, c_sel] = max(sum(val_cost(:, 2:3), 2) - 2 * val_cost(:, 1));
            loc_in = srt_c(c_sel, 1);
            c = seq_c(c_sel);
            % �����µĿͻ�
            if loc_in <= len_route
                seq_r(loc_in + 2:len_route + 1) = seq_r(loc_in + 1:len_route);
                seq_r(loc_in + 1) = c;
                len_route = len_route + 1;
                cap_s(sign_fleet(loc_in)) = cap_s(sign_fleet(loc_in)) + demand(c);
            else
                % �������λ�ó�����ǰ·����Χ���򽫿ͻ������µĳ�����
                seq_r(len_route+1:len_route+2) = [s_cus(c_sel),c];
                if strcmp(cus_type(c-num_sat), 'delivery')
                    cap_s(len_route+1) = demand(c); % ��¼�³�����װ����
                else
                    cap_s(len_route+2) = demand(c);
                end
                len_route = len_route+2;
                num_fleet = num_fleet+1; % ����������һ
            end
        else
            [~, c_sel] = max(cost);
            c = seq_c(c_sel);
            cap_s         = zeros(1,2*num_cus);                  %��ʼ��·��ʵʱ����
            seq_r(1:2) = [s_cus(c_sel),c]; %��¼��ǰ�ͻ������ǵĶ�Ӧ��ϵ
            len_route  = 2;
            num_fleet  = num_fleet+1;   %ռ�õ����˻�����+1
            if strcmp(cus_type(c-num_sat), 'delivery')
                cap_s(1)   = demand(c);
            else
                cap_s(2)   = demand(c);
            end
        end
        % �Ƴ��Ѳ���ͻ�
        seq_c(c_sel) = [];
        num_c = num_c - 1;
    end
    seq_r(len_route + 1:end) = [];
end
