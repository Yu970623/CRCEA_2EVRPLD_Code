%s_offs:��¼һ���Ŀͻ���Ӧ������
%% --------------------------�ڶ���·���ֲ��ƻ�-------------------------
function [route,seq_c,s_offs,s_offl] = destroys(vrp2e,route,s_offl,sm)
    dis_sc = vrp2e.dis_sc;  
    neib_cc = vrp2e.neib_cc; 
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;

    len_route = numel(route);                                %���г���
    loc_sat   = [find(route<=num_sat),len_route+1];               %���Ƕ�Ӧ�����
    route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = []; %ɾ���ճ�������·��
    ndl       = ceil(rand*min(60,0.4*num_cus));                   %���ɾ���Ŀͻ���Ŀ
    if sm == 4
        %----���ɾ��ndl���ͻ���̰������:Random Remove----
        seq_c          = randperm(num_cus,ndl)+num_sat;
        [~,loc_c]      = ismember(seq_c,route);     %����ͻ���·���е�λ��
        sign_fleet     = cumsum(route<=num_sat,2);       %���ǺͿͻ���Ӧ�ĳ���
        fleet_cus      = sign_fleet(loc_c);          %ָ���ͻ���Ӧ�ĳ�����
        sat_fleet      = route(route<=num_sat);          %������Ӧ������
        s_offs         = sat_fleet(fleet_cus);       %�ͻ�seq_c��Ӧ������
        route(loc_c)   = [];                         %��·����ɾ���ͻ�
        len_route      = numel(route);                              %���е�·������
        loc_sat        = [find(route<=num_sat),len_route+1];             %������·���е�λ��
        route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
    elseif sm == 5
                %----���ɾ��ndl���ͻ���̰������:Random Remove----
        seq_c          = randperm(num_cus,ndl)+num_sat;
        [~,loc_c]      = ismember(seq_c,route);     %����ͻ���·���е�λ��
        sign_fleet     = cumsum(route<=num_sat,2);       %���ǺͿͻ���Ӧ�ĳ���
        fleet_cus      = sign_fleet(loc_c);          %ָ���ͻ���Ӧ�ĳ�����
        sat_fleet      = route(route<=num_sat);          %������Ӧ������
        s_offs         = sat_fleet(fleet_cus);       %�ͻ�seq_c��Ӧ������
        route(loc_c)   = [];                         %��·����ɾ���ͻ�
        len_route      = numel(route);                              %���е�·������
        loc_sat        = [find(route<=num_sat),len_route+1];             %������·���е�λ��
        route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
        %----ɾ��������۽ϸߵĿͻ�: WorstRemoval----
%         loc_cus       = find(route>num_sat);                %�ͻ���·���е�λ��
%         cus_front     = route(loc_cus-1);              %�ͻ���ǰһ����
%         rback         = route;
%         loc_sat       = find(route<=num_sat);
%         rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
%         route2        = [rback(2:end),route(loc_sat(end))];
%         loc_cus       = find(route2>num_sat);                %�ͻ���·���е�λ��
%         cus_back      = route2(loc_cus+1);              %�ͻ��ĺ�һ����
%         cus_r         = route2(loc_cus);                %��ȡȫ���Ŀͻ�
%         nsc           = num_sat+num_cus;
%         dis_front     = dis_sc(cus_front+(cus_r-1)*nsc);
%         dis_back      = dis_sc(cus_r+(cus_back-1)*nsc);
%         dis_fb        = dis_sc(cus_front+(cus_back-1)*nsc);
%         perturb       = 0.8+0.4*rand(1,num_cus); %�����Ŷ�
%         cost_cus      = perturb.*(dis_front+dis_back-dis_fb)./(dis_front+dis_back);
%         [~,srt_c]     = sort(cost_cus,'descend');
%         seq_c         = cus_r(srt_c(1:ndl));
%         [~,loc_c]     = ismember(seq_c,route);      %����ͻ���·���е�λ��
%         sign_fleet    = cumsum(route<=num_sat,2);        %���ǺͿͻ���Ӧ�ĳ���
%         fleet_cus     = sign_fleet(loc_c);          %ָ���ͻ���Ӧ�ĳ�����
%         sat_fleet     = route(route<=num_sat);            %������Ӧ������
%         s_offs        = sat_fleet(fleet_cus);        %�ͻ�seq_c��Ӧ������
%         route(loc_c)  = [];                          %��·����ɾ���ͻ�
%         len_route     = numel(route);                              %���е�·������
%         loc_sat       = [find(route<=num_sat),len_route+1];             %������·���е�λ��
%         route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
    elseif sm == 6
        %----���ɾ���ٽ���ndl���ͻ���̰������:Related Removal----
        rc            = ceil(rand*num_cus);              %���ѡȡһ���ͻ���
        seq_c         = neib_cc(rc,1:ndl);          %ѡȡ��ͻ��ڽ���ndl���ͻ�
        [~,loc_c]     = ismember(seq_c,route);      %����ͻ���·���е�λ��
        sign_fleet    = cumsum(route<=num_sat,2);        %���ǺͿͻ���Ӧ�ĳ���
        fleet_cus     = sign_fleet(loc_c);          %ָ���ͻ���Ӧ�ĳ�����
        sat_fleet     = route(route<=num_sat);            %������Ӧ������
        s_offs        = sat_fleet(fleet_cus);        %�ͻ�seq_c��Ӧ������
        route(loc_c)  = [];                          %��·����ɾ���ͻ�
        len_route     = numel(route);                              %���е�·������
        loc_sat       = [find(route<=num_sat),len_route+1];             %������·���е�λ��
        route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
    elseif sm == 7
        %----���ɾ��һ��·����Route Removal----
        sign_fleet  = cumsum(route<=num_sat,2);           %���ǺͿͻ���Ӧ�ĳ���
        num_fleet   = sign_fleet(end);               %·������
        
        rf           = ceil(rand*num_fleet);         %���ѡȡһ��·��
        loc_r        = find(sign_fleet==rf);         %��ȡ��·����Ӧ��λ��  
        seq_c        = route(loc_r(2:end));          %��ȡ·���еĿͻ�
        if sum(s_offl) == 1 && num_fleet ==1         %���ֻ��һ�����ŵ�������ֻ��һ��·��
            s_off  = find(s_offl==0);                %�رյ�����
            s_offl = zeros(1,num_sat);                    %���ǵ�״̬
            s_offl(s_off(ceil(rand*(num_sat-1)))) = 1;    %��һ���رյ�����
            s_offs = [];
        else
            s_offs = route(loc_r(1))*ones(1,numel(seq_c));%�ͻ����ɲ��������
        end
        route(loc_r) = [];                           %ɾ����·��
    elseif sm == 8
        %----ÿ�����ŵ�����ɾ��k��·����������----
        num_son     = sum(s_offl);                   %�������ǵ�����
        if num_son>1
            sign_fleet  = cumsum(route<=num_sat,2);       %���ǺͿͻ���Ӧ�ĳ���
            num_fleet   = sign_fleet(end);           %·������
            dis_f       = zeros(1,num_fleet);        %·���еĿͻ��ڱ����Ǻ��������Ǽ����̾���
            for f = 1:num_fleet
                cus_f    = route((sign_fleet==f));    %��f��·��
                s        = cus_f(1);                  %����·��������
                cus_f    = cus_f(2:end);              %��f��·���еĿͻ�
                s_on     = s_offl;  s_on(s)=0;        %�������ŵ�����
                dis_f(f) = (0.8+0.4*rand)*min(min(bsxfun(@plus,dis_sc(s,cus_f),dis_sc(s_on==1,cus_f))));
            end
            [~,lst_f]    = sort(dis_f);               %��ÿ��·���ľ����������
            sat_fleet    = route(route<=num_sat);          %������Ӧ������
            num_sdel     = ceil(rand(1,num_sat)*3);        %ÿ�����ŵ����Ǵ�ɾ����·����Ŀ
            f_del        = zeros(1,num_fleet);        %��ɾ����·����ţ�1Ϊɾ��
            for f = lst_f
                if num_sdel(sat_fleet(f))>0
                    f_del(f) = 1;
                    num_sdel(sat_fleet(f)) = num_sdel(sat_fleet(f))-1;
                end
            end
            f_del   = find(f_del==1);
            log_cs  = ismember(sign_fleet,f_del);      %���Ӧ��·���Ƿ�ɾ��
            seq_c   = route(log_cs);                   %��ȡ��ɾ����·��
            seq_c(seq_c<=num_sat) = [];                %��ȡ·���еĿͻ�
            [~,pos_cus] = ismember(seq_c,route);       %�ͻ���·���е�λ��
            fleet_cus   = sign_fleet(pos_cus);         %ָ���ͻ���Ӧ�ĳ�����
            s_offs      = sat_fleet(fleet_cus);        %�ͻ�seq_c��Ӧ������
            route(log_cs) = [];                        %����·����ɾ������·��
        else
            seq_c       = [];
            s_offs      = [];
        end
    else
        disp('sm���ܳ���5��');
        seq_c = [];
        s_offs = [];
    end
end
