%% --------------------------�ڶ���·���Ż�-------------------------
function seq_cs = evosecond1(vrp2e,seq_cs,sm,opt_cus_pre)
    %global dis_sc; global neib_cc; global num_sat; global num_cus;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;

    seq_cs_pre = seq_cs;
    ndl               = ceil(rand*min(60,0.4*num_cus));  %���ɾ���Ŀͻ���Ŀ
    if sm == 1
        %----���ɾ���ٽ���ndl���ͻ���̰������----
        neib_cc = vrp2e.neib_cc;
        num_sat = vrp2e.num_sat;
        num_cus = vrp2e.num_cus;
        ndl           = ceil(rand*0.3*num_cus);       %���ɾ���Ŀͻ���Ŀ
        rc            = ceil(rand*num_cus);          %���ѡȡһ���ͻ���
        seq_c         = neib_cc(rc,1:ndl);           %ѡȡ��ͻ��ڽ���ndl���ͻ� 
        [~,loc_c]     = ismember(seq_c,seq_cs);      %����ͻ���·���е�λ��
        seq_cs(loc_c) = [];
        len_route     = numel(seq_cs);                               %���е�·������
        loc_sat       = [find(seq_cs<=num_sat),len_route+1];         %������·���е�λ��
        seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
    elseif sm == 2
        %----���ɾ��ndl���ͻ���̰������----
        seq_c          = randperm(num_cus,ndl)+num_sat;
        [~,loc_c]      = ismember(seq_c,seq_cs);     %����ͻ���·���е�λ��
        seq_cs(loc_c)   = [];                         %��·����ɾ���ͻ�
        len_route      = numel(seq_cs);                              %���е�·������
        loc_sat        = [find(seq_cs<=num_sat),len_route+1];             %������·���е�λ��
        seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
    elseif sm == 3
        seq_cs(find(seq_cs ~= 0, 1, 'last')+1:end) = [];
        num_sat = vrp2e.num_sat;
        loc_cus       = find(seq_cs>num_sat);           %�ͻ���·���е�λ��
        ndl           = ceil(rand*length(loc_cus));  %���������Ҫɾ���Ŀͻ���
        cost_route = funcopt3(vrp2e,seq_cs);
        no_cus = arrayfun(@(i) [seq_cs(1:i-1), seq_cs(i+1:end)], loc_cus, 'UniformOutput', false);
        cost_cus = arrayfun(@(i) cost_route-funcopt3(vrp2e,no_cus{i}), 1:length(no_cus), 'UniformOutput', false);
        cost_cus = cell2mat(cost_cus);
        del_c         = zeros(1,ndl);
        for i = 1:ndl
            cl           = roulette(cost_cus,1);
            del_c(i)     = loc_cus(cl);
            cost_cus(cl) = 0;
        end
        seq_c     = seq_cs(del_c);
        seq_cs(del_c) = [];
    else
        num_sat = vrp2e.num_sat;
        if rand < 0.4
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
    seq_cs = greedyin2(vrp2e,seq_cs,seq_c);
    [vrp_cs,~] = msecond(vrp2e,seq_cs);
    if ~isequal(seq_cs, opt_cus_pre)
        [seq_cs_temp, opt_cus_pre_temp, seq_cs_waite, opt_cus_pre_waite] = crossover(seq_cs, opt_cus_pre, num_sat);
%         seq_cs_temp = [seq_cs_temp, zeros(1, numel(seq_cs) - numel(seq_cs_temp))];
        if ~isempty(seq_cs_waite)
            seq_cs_temp = greedyin2(vrp2e,seq_cs_temp,seq_cs_waite);
        end
%         opt_cus_pre_temp = [opt_cus_pre_temp, zeros(1, numel(seq_cs) - numel(opt_cus_pre_temp))];
        if ~isempty(opt_cus_pre_waite)
            opt_cus_pre_temp = greedyin2(vrp2e,opt_cus_pre_temp,opt_cus_pre_waite);
        end
        [vrp_cs_temp,vlt_cs] = msecond(vrp2e,seq_cs_temp);
        [vrp_opt,vlt_opt] = msecond(vrp2e,opt_cus_pre_temp);
        if vlt_cs<=0 && vlt_opt<=0
            if vrp_cs_temp<vrp_opt && vrp_cs_temp<vrp_cs
                seq_cs = seq_cs_temp;
                vrp_cs = vrp_cs_temp;
            elseif vrp_opt<vrp_cs_temp && vrp_opt<vrp_cs
                seq_cs = opt_cus_pre_temp;
                vrp_cs = vrp_opt;
            end
        end
    end
    if ~isequal(sort(seq_cs(seq_cs>num_sat)), num_sat+1:num_sat+num_cus)
        setdiff(sort(seq_cs(seq_cs>num_sat)),num_sat+1:num_sat+num_cus)
    end
    if ~isequal(sort(seq_cs_new(seq_cs_new>num_sat)), num_sat+1:num_sat+num_cus)
        sort(seq_cs_new(seq_cs_new>num_sat))
    end
    [vrp_cs_new,vlt_cs_new] = msecond(vrp2e,seq_cs_new);
    if vlt_cs<=0 && vlt_cs_new<=0
        if vrp_cs<vrp_cs_new
            seq_cs = seq_cs_pre;
        else
            seq_cs = seq_cs_new;
        end
    else
        seq_cs = seq_cs_pre;
    end
end

function [seq_cs_temp, opt_cus_temp, seq_cs_waite, opt_cus_waite] = crossover(seq_cs, opt_cus, num_sat)
    cs_sat = unique(seq_cs(seq_cs<=num_sat&seq_cs>0));
    opt_sat = unique(opt_cus(opt_cus<=num_sat&opt_cus>0));
    selected_cs = cs_sat(randi(length(cs_sat)));    %seq_cs���滻����
    selected_opt = opt_sat(randi(length(opt_sat))); %opt_cus���滻����
    
    sat_cs = arrayfun(@(x) find(seq_cs == x), cs_sat, 'UniformOutput', false);
    sat_cs = [sat_cs{:}];
    sat_cs = sort(sat_cs);      %�ҳ�seq_cs���������±�

    sat_opt = arrayfun(@(x) find(opt_cus == x), opt_sat, 'UniformOutput', false);
    sat_opt = [sat_opt{:}];
    sat_opt = sort(sat_opt);    %�ҳ�opt_cus���������±�


    seq_cs_temp = seq_cs;
    idx1 = find(seq_cs(sat_cs)==selected_cs);  %�ҳ����д��滻���ǵ��±�
    next_idx1 = idx1+1;
    for i = 1:length(idx1)
        if next_idx1(i) <= length(sat_cs)
            seq_cs_temp(sat_cs(idx1(i)):sat_cs(next_idx1(i))-1)=0;
        else
            seq_cs_temp(sat_cs(idx1(i)):end)=0;
        end
    end
    
    opt_cus_customers = [];
    idx2 = find(opt_cus(sat_opt)==selected_cs);
    next_idx2 = idx2+1;
    for i = 1:length(idx2)
        if next_idx2(i) <= length(sat_opt)
            opt_cus_customers = [opt_cus_customers, opt_cus(sat_opt(idx2(i)):sat_opt(next_idx2(i))-1)]; 
        else
            opt_cus_customers = [opt_cus_customers, opt_cus(sat_opt(idx2(i)):end)]; 
        end
    end

    opt_cus_temp = opt_cus;
    idx3 = find(opt_cus(sat_opt)==selected_opt);
    next_idx3 = idx3+1;
    for i = 1:length(idx3)
        if next_idx3(i) <= length(sat_opt)
            opt_cus_temp(sat_opt(idx3(i)):sat_opt(next_idx3(i))-1)=0;
        else
            opt_cus_temp(sat_opt(idx3(i)):end)=0;
        end
    end

    seq_cs_customers = [];
    idx4 = find(seq_cs(sat_cs)==selected_opt);
    next_idx4 = idx4+1;
    for i = 1:length(idx4)
        if next_idx4(i) <= length(sat_cs)
            seq_cs_customers = [seq_cs_customers, seq_cs(sat_cs(idx4(i)):sat_cs(next_idx4(i))-1)]; 
        else
            seq_cs_customers = [seq_cs_customers, seq_cs(sat_cs(idx4(i)):end)]; 
        end
    end

    seq_cs_temp = [seq_cs_temp,opt_cus_customers];
    opt_cus_temp = [opt_cus_temp,seq_cs_customers];

    cs_cus = seq_cs_temp(seq_cs_temp > num_sat);
    [unique_elements, ~, idx] = unique(cs_cus); % ��ȡΨһԪ�ؼ�������
    counts = accumarray(idx, 1); % ͳ��ÿ��Ԫ�صĳ��ִ���
    seq_cs_waite = unique_elements(counts > 1); % �ҳ��ظ���Ԫ��
    seq_cs_temp(ismember(seq_cs_temp, seq_cs_waite)) = 0;

    op_cus = opt_cus_temp(opt_cus_temp > num_sat);
    [unique_elements, ~, idx] = unique(op_cus); % ��ȡΨһԪ�ؼ�������
    counts_opt = accumarray(idx, 1); % ͳ��ÿ��Ԫ�صĳ��ִ���
    opt_cus_waite = unique_elements(counts_opt > 1); % �ҳ��ظ���Ԫ��
    opt_cus_temp(ismember(opt_cus_temp, opt_cus_waite)) = 0;

    seq_cs_temp(seq_cs_temp == 0) = [];
    opt_cus_temp(opt_cus_temp == 0) = [];

    % �� seq_cs_temp_unique �� seq_cs �Աȣ��ҵ� seq_cs_temp_unique ��û�е��Ҵ��� num_sat ��Ԫ��
    missing_cs = setdiff(seq_cs, seq_cs_temp);  % �ҵ� seq_cs_temp_unique ��û�е�Ԫ��
    missing_cs = missing_cs(missing_cs > num_sat);  % ɸѡ������ num_sat ��Ԫ��
    missing_opt = setdiff(opt_cus, opt_cus_temp);  % �ҵ� seq_cs_temp_unique ��û�е�Ԫ��
    missing_opt = missing_opt(missing_opt > num_sat);  % ɸѡ������ num_sat ��Ԫ��

    seq_cs_waite = unique([seq_cs_waite, missing_cs]);  % ȥ�ز��ϲ��� A ��
    opt_cus_waite = unique([opt_cus_waite, missing_opt]);  % ȥ�ز��ϲ��� A ��
end



%% ----̰�������㷨----
%��seq_c�еĿͻ����뵽���еĶ���seq_cs�У�s_off���رջ��ֹ������
function seq_r = greedyin2(vrp2e,route,seq_c)
    route(find(route ~= 0, 1, 'last')+1:end) = [];
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
    num_fleet     = sum(route<=num_sat);                    %���㵱ǰ�ѷ���ĳ�������
    cap_s         = zeros(1,2*num_cus);
    nums_c           = num_sat+num_cus;
    len_route     = numel(route);
    seq_r         = zeros(1,2*num_cus); %����-�ͻ���Ӧ��ϵ��¼
    seq_r(1:len_route) = route;

    %----̰������----
    for c = seq_c    %���ݾ����Լ����������ÿ���ͻ����뵽·�� route �еĺ���λ��
        cost_newr        = 2*dis_sc(c,1:num_sat);   %��ǰ�ͻ��ֱ����������ǵľ���
        [cost,s]         = min(cost_newr);          % �ҳ���ǰ�ͻ�Ӧ�ò��������λ��
        if len_route > 0 
            route      = seq_r(1:len_route);        %����·��
            sign_fleet = cumsum(route<=num_sat,2);  %���ǺͿͻ���Ӧ�����˻����������ۻ��ͣ�
%             cap_sc     = cap_s(sign_fleet);         %ÿһ��λ�ö�Ӧ������(�����ʽ��Ҫ����)
            loc_sat    = find(route<=num_sat);      %������·���е�λ��
             % �����Ǻͳ�������Ӧ·����¼
            rback      = route; 
            rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
            route2     = [rback(2:end),route(loc_sat(end))];    %�������ǵ�·��
            cap_sc = cap_s(1:len_route);
            power = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);

%           %���㵱ǰ����·�������ܺ�
            Energy_route = power.* dis_sc(route+(route2-1)*nums_c)./V;
            Energy = [arrayfun(@(i)[sum(Energy_route(loc_sat(i):loc_sat(i+1)-1))], 1:length(loc_sat)-1, 'UniformOutput', false),sum(Energy_route(loc_sat(end):end))];  %����ԭ·�������ܺ�
            Energy = arrayfun(@(x) Energy(x), sign_fleet);
            Energy = cell2mat(Energy);
            %����ȡ-������ֱ������������û�����ܵĴ���, ���˻�����+���������������Ƿ񳬹�Լ��
            if strcmp(cus_type(c-num_sat), 'pickup')
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
%                 arrayfun(@(n) [cap_sc(1:n-1)+demand(c), cap_sc(n-1), cap_sc(n:end)], 2:length(cap_sc)+1, 'UniformOutput', false)
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
%             route_c2 = cellfun(@(x) [x(2:end), x(loc_sat(end))], arrayfun(@(pos) [route(1:pos-1), c, route(pos:end)], 2:length(route)+1, 'UniformOutput', false), 'UniformOutput', false);%���·��
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
            %             dis_sc(route+(route2-1)*nsc)
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
            cap_s         = zeros(1,2*num_cus);                  %��ʼ��·��ʵʱ����
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
function fvrp = funcopt3(vrp2e,route)
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    loc_sat = find(route<=num_sat); 
    rback   = route; rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
    route2  = [rback(2:end),route(loc_sat(end))];
    cap_sc = energy(vrp2e,[route,route2(end)]);
    Dis_route = dis_sc(route+(route2-1)*(num_sat+num_cus));
    Energy_route = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
    Energy_distribution = Energy_route.*Dis_route./vrp2e.V; 
    fvrp = sum(Energy_distribution);
end