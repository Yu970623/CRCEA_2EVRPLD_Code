%% --------------------------��ʼ����һ��--------------------------
%step1: �����ǰ��սǶ����򣬲�ѡ��Ƕȼ�����ĵ���Ϊ��㣬�����������������£����������������������
%step2: ����ӵ����ǰ��յ���ķ�ʽ���е�����
%step3: ��ͬһ���������ǰ��սǶȽ�������
function [seq_sats,cap_sats,vrp1,vlt1] = initsat(vrp2e,seq_r) 
    %global fleet; global demand; global dis_ds;  global num_sat;
    fleet = vrp2e.fleet;
    dis_ds = vrp2e.dis_ds; 
    dis_sc = vrp2e.dis_sc;  
    num_dep = length(vrp2e.rad_ds(:,1));
    demand = vrp2e.demand; 
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    % ----ͳ�Ƹ����ǵĻ�������----
    num_pop = size(seq_r,1);
    %----��ʼ������----
    len_route1 = 2*(fleet(1,2)+num_sat);
    seq_sats   = zeros(num_pop,len_route1);         %Ԥ����洢�������2*fleet(1,2)+2*ns�������ռ�ɾ��
    cap_sats   = zeros(num_pop,len_route1);         %Ԥ����洢��������2*fleet(1,2)+2*ns�������ռ�ɾ��
    vrp1     = zeros(1,num_pop);
    vlt1     = zeros(1,num_pop);


    %% -=--��ʼ����һ��----
    for i = 1:num_pop
        seq_sat   = zeros(1,len_route1);         %Ԥ����洢�������2*fleet(1,2)+2*ns�������ռ�ɾ��
        cap_sat   = zeros(1,len_route1);         %Ԥ����洢��������2*fleet(1,2)+2*ns�������ռ�ɾ��
        cap_s     = zeros(1,num_sat);
        sat_cus = zeros(1,num_cus);
        for c = 1:num_cus
            sat_cus(c) = seq_r(i, find(seq_r(i, 1:find(seq_r(i,:) == c + num_sat, 1)) <= num_sat, 1, 'last'));
        end
        %----���վ���������̶Ľ����Ƿ�����ֿ�----
        dep_sat = zeros(1,num_sat);              %�洢ÿ�����ǹ����Ĳֿ�
        for c = 1:num_sat
            if ismember(c, sat_cus)
                dep_sat(c) = roulette(1./dis_sc(1:num_dep,c+num_dep),1);
            else
                dep_sat(c) = 0;
            end
        end
        nrs   = 0;   %·���ܳ���
        for s = 1:num_sat
            cus_s = find(sat_cus==s)+num_sat;
            cap_s(s) = sum(demand(cus_s));
            while cap_s(s)>fleet(1,1)
                nrs          = nrs+2;
                seq_sat(nrs-1) = dep_sat(s);
                seq_sat(nrs) = s+num_dep;
                cap_sat(nrs) = fleet(1,1);
                cap_s(s)     = cap_s(s)-fleet(1,1);
            end
        end
        for d = 1:num_dep
            dep_s                 = find(dep_sat==d)+num_dep;  %��ǰ�ֿ��µ�����
            route                 = algcw1(dis_ds(d,dep_s),dis_ds(dep_s,dep_s),cap_s,fleet(1,1));
            loc                   = find(route>0);  %·��������λ��
            route(loc)            = dep_s(route(loc));  %������Ż��������������
            cap_r                 = route;
            if length(loc)==1 && cap_s(route(loc)-num_dep) == 0
                route = [];
                cap_r = [];
                nr=0;
            else
                cap_r(loc)            = cap_s(route(loc)-num_dep);
                route(route==0)       = d;  %��������滻0
                nr                    = numel(route);
            end
            seq_sat(nrs+1:nrs+nr) = route;
            cap_sat(nrs+1:nrs+nr) = cap_r;
            nrs                   = nrs+nr;
            seq_sat(nrs+1:end)    = [];
            cap_sat(nrs+1:end)    = [];
        end
        [vrp1(i),vlt1(i)] = mfirst(vrp2e,seq_sat,cap_sat);
        seq_sat(find(seq_sat ~= 0, 1, 'last')+1:end) = [];
        cap_sat(find(cap_sat ~= 0, 1, 'last')+1:end) = [];
        if length(seq_sat)~=length(cap_sat)
            seq_sat
        end
        seq_sats(i,:)=[seq_sat,zeros(1,len_route1-length(seq_sat))];
        cap_sats(i,:)=[cap_sat,zeros(1,len_route1-length(seq_sat))];
    end
end


%% --------------���ͻ�������(����CW)--------------------
function rount = algcw1(dis_dtos,dis_stos,demand_sat,capacity)
    nc         = length(dis_dtos);     %�ͻ�����
    link       = zeros(2,nc);         %�ͻ����ӵ������ͻ����洢�뱾�ͻ����ӵ������ͻ�
    nlink      = zeros(1,nc);         %�ͻ����ӵĴ���������ʶ��ÿͻ��Ƿ񻹿�����
    seq_veh    = 1:nc;                %�ͻ���Ӧ�ĳ�����ţ����ڼ��㳵�����ػ���
    cw_cc      = reshape(triu(bsxfun(@plus,dis_dtos,dis_dtos')-dis_stos+0.01,1),1,[]);  %�����ʡ�ķ���
    [~,srt_cp] = sort(cw_cc,'descend');    %����ʡ����������
    npair      = nc*(nc-1)/2;    %���е����
    srt_cp     = srt_cp(1:npair);          %��ȡ�����Ǿ�������
    cr         = mod(srt_cp-1,nc)+1;  %����ֵ��Ӧ����(�����ж�Ӧ�������ӹ�ϵ)
    cl         = ceil(srt_cp/nc);     %����ֵ��Ӧ����
    for i = 1:npair
        nv1 = seq_veh(cr(i));                %��ȡ�ͻ�1��Ӧ�ĳ������
        nv2 = seq_veh(cl(i));                %��ȡ�ͻ�2��Ӧ�ĳ������ nv1<nv2
        %----��������ӡ�����ͬһ·������������----
        if nlink(cr(i))<2 && nlink(cl(i))<2 && nv1~= nv2 && demand_sat(nv1)+demand_sat(nv2) <= capacity 
            demand_sat(nv1) = demand_sat(nv1)+demand_sat(nv2);  %�ϲ�����nv2�Ļ���������nv1
            demand_sat(nv2) = 0;                                %��ճ���nv2�Ļ���
            seq_veh(seq_veh==nv2) = nv1;                        %������nv2�пͻ���Ӧ��������Ϊnv1
            nlink(cr(i)) = nlink(cr(i))+1;                      %���¿ͻ�1��������Ŀ
            nlink(cl(i)) = nlink(cl(i))+1;                      %���¿ͻ�2��������Ŀ
            if link(1,cr(i))== 0
                link(1,cr(i)) = cl(i);
            else
                link(2,cr(i)) = cl(i);
            end
            if link(1,cl(i))== 0
                link(1,cl(i)) = cr(i);
            else
                link(2,cl(i)) = cr(i);
            end
        end  
    end
    rount   = zeros(1,2*nc+1);          %����ܷ�����������
    cus_isolate = find(nlink==0);       %Ѱ�ҹ����Ŀͻ�
    count = 2*numel(cus_isolate);
    if count>0
        rount(2:2:count) = cus_isolate; %�����㵥���γ�һ����·
    end
    cus_start = find(nlink==1);         %Ѱ�һ�·�����
    for nv = cus_start
        if sum(rount==nv)==0            %�жϸû�·���ͻ��Ƿ��ڻ�·��
           count = count+2; 
           rount(count) = nv;
           cus_next = link(1,nv);
           count = count+1;
           rount(count) = cus_next;
           while nlink(cus_next)==2
               count = count+1;
               if link(1,cus_next)==rount(count-2) %�ж��뵱ǰ�ͻ������ӵ�һ���ͻ��Ƿ�����·����
                   rount(count) = link(2,cus_next);%�洢�뵱ǰ�ͻ����ӵĵڶ����ͻ�
                   cus_next = link(2,cus_next);
               else
                   rount(count) = link(1,cus_next);%�洢�뵱ǰ�ͻ����ӵ�һ���ͻ�
                   cus_next = link(1,cus_next);
               end
           end
        end
    end
    rount(count+1:end) = []; %���û�д洢�ͻ��Ŀռ�
end