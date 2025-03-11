%% --------------------------��һ��·���Ż�-------------------------
function [seq_s,cap_s] = repair1(vrp2e,seq_r)
    demand = vrp2e.demand;
    num_sat = vrp2e.num_sat;
    %----ͳ����·����ÿ�����ǵ�������----
    sat_fleet = seq_r(seq_r<=num_sat);                           %������Ӧ������
    cap_r     = zeros(1,num_sat);
    for s = 1:num_sat
        sign_fleet_s  = find(sat_fleet==s);                      %����s��Ӧ�ĳ�������
        sign_fleet    = cumsum(seq_r<=num_sat,2);                %���ǺͿͻ���Ӧ�ĳ���
        [logic_loc,~] = ismember(sign_fleet,sign_fleet_s);       %����s����Ӧ�ͻ�����Ϊ1
        cap_r(s)      = sum(demand(seq_r(logic_loc)));           %����s������·��������
    end
    %----��һ���޸�����-------------------------
    [seq_s,cap_s]     = greedyin1(vrp2e,cap_r);     %̰���޸�,ȷ����������˳����������
    %----move��swap�Ż�-----
    [seq_s,cap_s]     = move1(vrp2e,seq_s,cap_s);   %������������˳�򼰶�Ӧ����������
    [seq_s,cap_s]     = swap1(vrp2e,seq_s,cap_s);   %������������˳�򼰶�Ӧ����������
end

%% ----̰����ʽ�޸���һ��·��----
function [seq_sat,cap_sat] = greedyin1(vrp2e,cap_r)
    fleet = vrp2e.fleet;
    dis_ds = vrp2e.dis_ds;
    num_sat = vrp2e.num_sat;
    num_dep = length(dis_ds)-num_sat;
    
    %----������������Ԥ����----
    cap_v     = zeros(1,2*fleet(1,2)+2*num_sat);        %����ÿ�������Ŀ�װ�ص�����
    sat_unset   = zeros(1,2*fleet(1,2)+2*num_sat);        %Ԥ����洢�������2*fleet(1,2)+2*ns�������ռ�ɾ��
    seq_sat   = zeros(1,2*fleet(1,2)+2*num_sat);        %Ԥ����洢�������2*fleet(1,2)+2*ns�������ռ�ɾ��
    cap_sat   = zeros(1,2*fleet(1,2)+2*num_sat);        %Ԥ����洢��������2*fleet(1,2)+2*ns�������ռ�ɾ��
    len_route = 0;
    n = 0;
    for i = 1:num_sat
        num_s = ceil(cap_r(i)/fleet(1,1));
        if num_s == 1
            sat_unset(i+n) = i+num_dep; %���������������
            cap_v(i+n) = cap_r(i);
        elseif num_s > 1
            sat_unset(i:i+num_s-1) = i+num_dep;
            cap_v(i:i+num_s-2) = fleet(1,1);
            cap_v(i+num_s-1) = cap_r(i)-fleet(1,1)*(num_s-1);
            n = n + num_s - 1;
        end
    end
    %----̰������----
    sat_unset = sat_unset(sat_unset~=0);
    cap_v = cap_v(cap_v~=0);
    penalty   = 100000000;
    nsc = num_sat+num_dep;
    for i = 1:length(sat_unset)   %��һ����·��
        cost_news = dis_ds(sat_unset(i),1:num_dep);
        [cost,l]  = min(cost_news);
        if len_route > 0 
            route = seq_sat(1:len_route);
            num_fleet     = sum(route<=num_dep);
            sign_fleet = cumsum(route<=num_dep,2);
            logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');
            cap_s         = zeros(1,2*fleet(1,2));
            for j = 1:num_fleet
                cap_s(j)  = sum(cap_sat(logical_fleet(j,:)));
            end
            cap_ds     = cap_s(sign_fleet);
            loc_depot = find(route<=num_dep);
            rback = route; rback(loc_depot(2:end)) = route(loc_depot(1:end-1));
            route2     = [rback(2:end),route(loc_depot(end))];
            punish     = [penalty *max(cap_ds+cap_v(i)-fleet(1,1),0),penalty *max(num_fleet+1-fleet(1,2),0)];
            cost_c     = [dis_ds(sat_unset(i),route)+dis_ds(sat_unset(i),route2)-dis_ds(route+(route2-1)*nsc),cost]+punish;
            [~,loc_in] = min(cost_c);

            if loc_in <= len_route
                seq_sat(loc_in+2:len_route+1) = seq_sat(loc_in+1:len_route);
                seq_sat(loc_in+1) = sat_unset(i);
                cap_sat(loc_in+2:len_route+1) = cap_sat(loc_in+1:len_route);
                cap_sat(loc_in+1) = cap_v(i);
                len_route       = len_route+1;
            else
                seq_sat(len_route+1:len_route+2) = [l,sat_unset(i)];
                cap_sat(len_route+2) = cap_v(i);
                len_route = len_route+2;
            end
        else
            seq_sat(1:2) = [l,sat_unset(i)];
            len_route  = 2;
            cap_sat(2) = cap_v(i);
        end
    end
    seq_sat(len_route+1:end) = [];
    cap_sat(len_route+1:end) = [];
end

%% --------------------------����move�Ż���һ��·��-------------------------
function [seq_s,cap_s] = move1(vrp2e,seq_s,cap_s)  %��һ�ƶ��������ǣ�����ƶ���·���ɱ�
    fleet = vrp2e.fleet;
    dis_ds = vrp2e.dis_ds;
    num_sat = vrp2e.num_sat;
    num_dep = length(dis_ds)-num_sat;

    len_route = numel(seq_s);                          %ȫ��·������
    loc_d     = find(seq_s<=num_dep);                  %�ֿ���·���е�λ��
    num_fleet = numel(loc_d)-1;                        %��������
    cap_v     = zeros(1,num_fleet+1);                  %�洢������װ�ػ�����
    for i = 1:num_fleet
        cap_v(i) = sum(cap_s(loc_d(i)+1:loc_d(i+1)-1));%ǰn-1�����ֱ�Ļ�������
    end
    cap_v(end)=sum(cap_s(loc_d(end):end));
    nm       = 1;                                      %��ǰ�ƶ��ĵ���·���е�λ�ã��ƶ��Ĵ�����
    penalty  = 100000000;                              %���سͷ�ֵ   
    while nm < len_route   
        if seq_s(nm)<=num_dep                                             %�������������ǳ���-1
            nm = nm+1;
        else
            s           = seq_s(nm);                                  %����������
            cap         = cap_s(nm);                                 %���������ǵ�����
            nf_s        = sum(abs(seq_s(1:nm))<=num_dep);            %�������������ڵĳ���
            cap_v(nf_s) = cap_v(nf_s)-cap;                           %��ǰ������װ�ص�������ȥ����ǰ���Ǻ�
            seq_t       = [seq_s(1:nm-1),seq_s(nm+1:end)];           %�������·����ȥ�������Ǻ��·����
            seq_r       = abs(seq_t);                                %�������·��
            cap_r       = [cap_s(1:nm-1),cap_s(nm+1:end)];           %������·�������Ƕ�Ӧ������
            sign_fleet  = cumsum(seq_r<=num_dep,2);                  %���ǺͲֿ��Ӧ�ĳ���
            cap_ds      = cap_v(sign_fleet(1:end-1));                %ÿһ��λ����װ�ص�����

            loc_depot = find(seq_r<=num_dep);
            rback = seq_r; rback(loc_depot(2:end)) = seq_r(loc_depot(1:end-1));
            seq_r2     = [rback(2:end),seq_r(loc_depot(end))];
            punish     = [penalty *max(cap_ds+cap-fleet(1,1),0),penalty *max(num_fleet+1-fleet(1,2),0)];
            cost_s     = dis_ds(s,seq_r)+dis_ds(s,seq_r2)-dis_ds(seq_r+(seq_r2-1)*(num_sat+num_dep))+punish;
            [~,loc] = min(cost_s);
            seq_s       = [seq_t(1:loc),-s,seq_t(loc+1:end)];        %Ϊ�����ظ��������������������ǳ���-1
            cap_s       = [cap_r(1:loc),cap,cap_r(loc+1:end)];
            cap_v(sign_fleet(loc)) = cap_v(sign_fleet(loc))+cap;
        end
    end
    seq_s  = abs(seq_s);
    loc_re = fliplr(find(seq_s(1:end-1) <= num_dep & seq_s(2:end) <= num_dep));
    for i = loc_re
        cap_s(i)   = cap_s(i)+cap_s(i+1);
        cap_s(i+1) = [];
        seq_s(i+1) = [];
    end
end

%% --------------------------����swap�Ż���һ��·��-------------------------
function [seq_s,cap_s] = swap1(vrp2e,seq_s,cap_s) %��һ��������������������ǣ����ɱ�
    fleet = vrp2e.fleet;
    dis_ds = vrp2e.dis_ds;
    num_sat = vrp2e.num_sat;
    num_dep = length(dis_ds)-num_sat;
    %----swap���Կ�ʼ:ÿ����ѡȡ������ֻ���������ǽ��н���----
    penalty  = 100000000;                              %���سͷ�ֵ   
    A = 0;
    while A == 0 && length(seq_s)>2
        seq_r       = seq_s;                                %�������·��
        loc_s       = find(seq_r>num_dep);                      %�������ڵ�λ�� 
        % ����ÿ�����ǣ����ɽ������
        seq_r_new = arrayfun(@(idx) ...
            arrayfun(@(swap_idx) ...
                swap(seq_r, loc_s(idx), loc_s(swap_idx)), ...
                idx+1:length(loc_s), 'UniformOutput', false), ...
            1:length(loc_s)-1, 'UniformOutput', false);
        cap_s_new = arrayfun(@(idx) ...
            arrayfun(@(swap_idx) ...
                swap(cap_s, loc_s(idx), loc_s(swap_idx)), ...
                idx+1:length(loc_s), 'UniformOutput', false), ...
            1:length(loc_s)-1, 'UniformOutput', false);
        seq_r_new = horzcat(seq_r_new{:});
        cap_s_new = horzcat(cap_s_new{:});
        seq_r_new2 = cellfun(@(x) generateSeqR2(x, num_dep), seq_r_new, 'UniformOutput', false);
        seq_r2     = generateSeqR2(seq_r, num_dep);
        cap_v_cell = cellfun(@(seq_r_new) calculateCapV(seq_r_new, num_dep, cap_s), seq_r_new, 'UniformOutput', false);
        punish     = cellfun(@(x) max(max(x)-fleet(1,1),0), cap_v_cell, 'UniformOutput', false);
        cost_r = cellfun(@(r, rf) sum(dis_ds(r + (rf - 1) * (num_sat + num_dep))), seq_r_new, seq_r_new2, 'UniformOutput', false);
        cost_r = cell2mat(cost_r)+penalty*cell2mat(punish);
        cost_s = sum(dis_ds(seq_r+(seq_r2-1)*(num_sat+num_dep)));  %�����пɽ���λ�ý���ǰ�Ĵ���
        cost   = cost_r-cost_s; %�������뽻��ǰ�Ĳ�ֵ
        [num,loc_ex]           = min(cost);
        if num < 0
            seq_s = seq_r_new{loc_ex};
            cap_s = cap_s_new{loc_ex};
            A = 0;
        else
            A = 1;
            break;
        end
    end
    loc_re = fliplr(find(seq_s(1:end-1) <= num_dep & seq_s(2:end) <= num_dep));
    for i = loc_re
        cap_s(i)   = cap_s(i)+cap_s(i+1);
        cap_s(i+1) = [];
        seq_s(i+1) = [];
    end
end
% ��������ʵ�ֽ����߼�
function new_seq = swap(seq, idx1, idx2)
    new_seq = seq;
    new_seq([idx1, idx2]) = seq([idx2, idx1]);
end

% ��������ʵ������ seq_r2 ���߼�
function seq_r2 = generateSeqR2(seq_r, num_dep)
    loc_depot = find(seq_r <= num_dep);
    rback = seq_r;
    rback(loc_depot(2:end)) = seq_r(loc_depot(1:end-1));
    seq_r2 = [rback(2:end), seq_r(loc_depot(end))];
end

% ��������ʵ��cap_v�ļ���
function cap_v = calculateCapV(seq_r, num_dep, cap_s)
    loc_d = find(seq_r <= num_dep);  % �ֿ���·���е�λ��
    num_fleet = numel(loc_d) - 1;        % ��������
    cap_v = zeros(1, num_fleet + 1);     % ��ʼ����װ�ػ�����
    
    for i = 1:num_fleet
        cap_v(i) = sum(cap_s(loc_d(i)+1:loc_d(i+1)-1));
    end
    cap_v(end)=sum(cap_s(loc_d(end):end));

end