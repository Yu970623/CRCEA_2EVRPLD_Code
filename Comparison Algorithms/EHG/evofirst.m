%% --------------------------��һ��·���Ż�-------------------------
function [seq_sat,cap_sat] = evofirst(vrp2e,seq_ss,cap_ss,sm)
    %global num_sat; global fleet;
    num_sat = vrp2e.num_sat;
    num_dep = length(vrp2e.rad_ds(:,1));
    fleet = vrp2e.fleet;
    seq_ss(find(seq_ss ~= 0, 1, 'last')+1:end) = [];
    cap_ss(find(cap_ss ~= 0, 1, 'last')+1:end) = [];
    if length(seq_ss)~=length(cap_ss)
        seq_ss
    end
%     cap_ss = cap_ss(1:length(seq_ss));
    if sum(seq_ss<=num_dep)>1 && length(unique(seq_ss(seq_ss>num_dep)))>1
        if sm == 1
           %----�ӷ���õĶ��������ɾ��ndl�����ǣ���̰�����������----
            loc_s  = find(seq_ss>num_dep);      %������һ��·���������±�
            uni_sat = unique(seq_ss(seq_ss>num_dep));
            len_s  = length(uni_sat);     %ʵ�����Ǹ���
            ndl    = ceil(rand*len_s*0.4);       %���ɾ�����ٽ���ɾ����������Ŀ
            [del_s,~]  = ismember(seq_ss, uni_sat(randperm(len_s,ndl))); %Ҫɾ���������±�
            del_s = find(del_s);
            add_sat = unique(seq_ss(setdiff(loc_s, del_s)))-num_dep;    %���˱�ɾ������֮����������
            %----����ɾ������������Ŀͻ�����������������
            cap_s = zeros(1,num_sat);
            for k = 1:length(add_sat)
                cap_s(add_sat(k))=sum(cap_ss(seq_ss==(add_sat(k)+num_dep)));
            end
            for i = 1:ndl   %���б�ɾ�������������Ļ�������
                sat_added = add_sat(randi(numel(add_sat)));  %������뵽ʣ��δ��ɾ�������������±�
                cap_s(sat_added) = cap_s(sat_added)+sum(cap_ss(seq_ss == seq_ss(del_s(i))));
            end
        else
           %----�ӷ���õĶ��������ɾ������·����̰������----
            num_f   = ceil(sum(cap_ss)/fleet(1,1));                       %�����һ������ʹ�õĳ�����
            ndf     = randperm(num_f,ceil(0.4*rand*num_f));               %���ɾ��������·�����
            sign_fleet = cumsum(seq_ss<=num_dep,2);                            %���ǺͿͻ���Ӧ�ĳ���
            loc_del    = ismember(sign_fleet,ndf);
            seq_st = seq_ss(loc_del(1:length(seq_ss)));   %ɾ����·���е�����
            cap_st = cap_ss(loc_del(1:length(cap_ss)));   %ɾ����·�������Ƕ�Ӧ�Ļ�������
            cap = cap_st(cap_st>0);
            add_sat = seq_ss(~loc_del(1:length(seq_ss)));    %���˱�ɾ������֮����������
            add_cap = cap_ss(~loc_del(1:length(cap_ss))); 
            %----����ɾ������������Ŀͻ�����������������
            cap_s = zeros(1,num_sat);
            cap_s(add_sat(add_sat>num_dep)-num_dep) = add_cap(add_cap>0);
            for i = 1:length(cap)   %���б�ɾ�������������Ļ�������
                sat = add_sat(add_sat>num_dep)-num_dep;
                sat_added = sat(randi(numel(sat)));  %������뵽ʣ��δ��ɾ�������������±�
                cap_s(sat_added) = cap_s(sat_added)+cap(i);
            end
        end
        [seq_ss,cap_ss] = greedyin1(vrp2e,cap_s);
    end
    len_route1 = 2*(fleet(1,2)+num_sat);
    len_r   = length(seq_ss);
    seq_sat = [seq_ss,zeros(1,len_route1-len_r)];
    cap_sat = [cap_ss,zeros(1,len_route1-len_r)];
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