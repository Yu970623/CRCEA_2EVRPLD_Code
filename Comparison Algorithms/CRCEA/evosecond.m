%% --------------------------�ڶ���·���Ż�-------------------------
function [seq_cs,sat_offl] = evosecond(vrp2e,seq_cs,sm1,sm2,sm3,sat_offl,opt_cus_pre,epsilon)
    %global dis_sc; global neib_cc; global num_sat; global num_cus;
    dis_sc = vrp2e.dis_sc;  
    neib_cc = vrp2e.neib_cc; 
    neib_sc = vrp2e.neib_sc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    seq_cs_ini = seq_cs;
    [vrp_ini,vlt_ini] = msecond(vrp2e,seq_cs_ini);
    s_offs = [];              %ɾ���ͻ���Ӧ������(�޸���)
    seq_cs(seq_cs==0) = [];     %��·��������0Ԫ��ȥ��

    %% ȫ������
    if sm1 == 1
        %----�ر�һ�����ŵ����ǣ����ȫ���ر��򿪷�һ������:Satellite Removal----
        if sum(sat_offl) == 1 %���ֻ��һ�����ŵ�����
            s_off  = find(sat_offl==0);              %�رյ�����
            sat_offl = zeros(1,num_sat);                  %���ǵ�״̬
            sat_offl(s_off(ceil(rand*(num_sat-1)))) = 1;  %��һ���رյ�����
            seq_cs  = [];                             %����·��Ϊ��
            seq_c   = num_sat+1:num_sat+num_cus;                    %����ȫ���ͻ�������
        else
            s_on              = find(sat_offl==1);                   %���ŵ�����
            s_off             = s_on(ceil(rand*numel(s_on)));        %���رյ�����
            sat_offl(s_off)   = 0;                                   %�ر�����
            sat_fleet         = seq_cs(seq_cs<=num_sat);                    %������Ӧ������
            sign_fleet_s      = find(sat_fleet==s_off);              %����s��Ӧ�ĳ�������
            sign_fleet        = cumsum(seq_cs<=num_sat,2);                 %���ǺͿͻ���Ӧ�ĳ���
            [logic_loc,~]     = ismember(sign_fleet,sign_fleet_s);   %����s����Ӧ�ͻ�����Ϊ1
            cus_sat           = seq_cs(logic_loc);                    %����s������·������s��
            seq_c             = cus_sat(cus_sat>num_sat);                 %��ȡ·���еĿͻ��㣨����s��
            seq_cs(logic_loc)  = [];                                  %ɾ��s_off��Ӧ������·��
        end
    elseif sm1 == 2
        %----����һ���رյ����ǣ�Ȼ��ɾ����֮�ٽ���q���ͻ�:Satellite Open----
        ns_off   = num_sat-sum(sat_offl);                     %seq_offl��0Ϊ�رգ�1Ϊ����
        if ns_off>0
            s_off          = find(sat_offl==0);          %�رյ�����
            s_on           = s_off(ceil(rand*ns_off));   %���ŵ�����           
            sat_offl(s_on) = 1;                          %���ŵ���������Ϊ1
            ndl            = ceil(rand*min(60,0.4*num_cus));  %��ɾ���Ŀͻ���Ŀ
            seq_c          = neib_sc(s_on,1:ndl);        %ѡȡ��ͻ��ڽ���ndl���ͻ�   
            [~,loc_c]      = ismember(seq_c,seq_cs);      %����ͻ���·���е�λ��
            sign_fleet     = cumsum(seq_cs<=num_sat,2);        %���ǺͿͻ���Ӧ�ĳ���
            fleet_cus      = sign_fleet(loc_c);          %ָ���ͻ���Ӧ�ĳ�����
            sat_fleet      = seq_cs(seq_cs<=num_sat);           %������Ӧ������
            s_offs         = sat_fleet(fleet_cus);       %�ͻ�seq_c��Ӧ������
            seq_cs(loc_c)  = [];                          %��·����ɾ���ͻ�
            len_route      = numel(seq_cs);                              %���е�·������
            loc_sat        = [find(seq_cs<=num_sat),len_route+1];             %������·���е�λ��
            seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
        else
            seq_c  = [];
            s_offs = [];
        end
    elseif sm1 == 3
        %----�ȹر�һ�����ǣ��ٻ��ھ��밴���̶�ѡ�񿪷�һ������:Satellite Swap----
        s_on              = find(sat_offl==1);                   %���ŵ�����
        s_off             = s_on(ceil(rand*numel(s_on)));        %���رյ�����
        sat_offl(s_off)   = 0;                                   %�����ǹر�
        sat_fleet         = seq_cs(seq_cs<=num_sat);                    %������Ӧ������
        sign_fleet_s      = find(sat_fleet==s_off);              %����s��Ӧ�ĳ�������
        sign_fleet        = cumsum(seq_cs<=num_sat,2);                 %���ǺͿͻ���Ӧ�ĳ���
        [logic_loc,~]     = ismember(sign_fleet,sign_fleet_s);   %����s����Ӧ�ͻ�����Ϊ1
        cus_sat           = seq_cs(logic_loc);                    %����s������·������s��
        seq_c             = cus_sat(cus_sat>num_sat);                 %��ȡ·���еĿͻ��㣨����s��
        seq_cs(logic_loc) = [];                                   %ɾ��s_off��Ӧ������·��
        dis               = dis_sc(s_off,1:num_sat);                  %ɾ���������������Ǽ�ľ���
        dis(s_off)        = inf;                                 %����ɾ�����ǵ��Լ��ľ���Ϊinf
        s_on = roulette(1./dis,1);                               %�������̶Ŀ���һ������
        sat_offl(s_on) = 1;                                      %���ÿ��ŵ�����
    end
    %% �ֲ�����
    len_route = numel(seq_cs);                                %���г���
    loc_sat   = [find(seq_cs<=num_sat),len_route+1];               %���Ƕ�Ӧ�����
    seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = []; %ɾ���ճ�������·��
    ndl       = ceil(rand*min(60,0.4*num_cus));                   %���ɾ���Ŀͻ���Ŀ
    if sm2 == 1
        %----���ɾ��ndl���ͻ���̰������:Random Remove----
        seq_c          = randperm(num_cus,ndl)+num_sat;
        [~,loc_c]      = ismember(seq_c,seq_cs);     %����ͻ���·���е�λ��
        sign_fleet     = cumsum(seq_cs<=num_sat,2);       %���ǺͿͻ���Ӧ�ĳ���
        fleet_cus      = sign_fleet(loc_c);          %ָ���ͻ���Ӧ�ĳ�����
        sat_fleet      = seq_cs(seq_cs<=num_sat);          %������Ӧ������
        s_offs         = sat_fleet(fleet_cus);       %�ͻ�seq_c��Ӧ������
        seq_cs(loc_c)   = [];                         %��·����ɾ���ͻ�
        len_route      = numel(seq_cs);                              %���е�·������
        loc_sat        = [find(seq_cs<=num_sat),len_route+1];             %������·���е�λ��
        seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
    elseif sm2 == 2
                %----���ɾ��ndl���ͻ���̰������:Random Remove----
        seq_c          = randperm(num_cus,ndl)+num_sat;
        [~,loc_c]      = ismember(seq_c,seq_cs);     %����ͻ���·���е�λ��
        sign_fleet     = cumsum(seq_cs<=num_sat,2);       %���ǺͿͻ���Ӧ�ĳ���
        fleet_cus      = sign_fleet(loc_c);          %ָ���ͻ���Ӧ�ĳ�����
        sat_fleet      = seq_cs(seq_cs<=num_sat);          %������Ӧ������
        s_offs         = sat_fleet(fleet_cus);       %�ͻ�seq_c��Ӧ������
        seq_cs(loc_c)   = [];                         %��·����ɾ���ͻ�
        len_route      = numel(seq_cs);                              %���е�·������
        loc_sat        = [find(seq_cs<=num_sat),len_route+1];             %������·���е�λ��
        seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
    elseif sm2 == 3
        %----���ɾ���ٽ���ndl���ͻ���̰������:Related Removal----
        rc            = ceil(rand*num_cus);              %���ѡȡһ���ͻ���
        seq_c         = neib_cc(rc,1:ndl);          %ѡȡ��ͻ��ڽ���ndl���ͻ�
        [~,loc_c]     = ismember(seq_c,seq_cs);      %����ͻ���·���е�λ��
        sign_fleet    = cumsum(seq_cs<=num_sat,2);        %���ǺͿͻ���Ӧ�ĳ���
        fleet_cus     = sign_fleet(loc_c);          %ָ���ͻ���Ӧ�ĳ�����
        sat_fleet     = seq_cs(seq_cs<=num_sat);            %������Ӧ������
        s_offs        = sat_fleet(fleet_cus);        %�ͻ�seq_c��Ӧ������
        seq_cs(loc_c)  = [];                          %��·����ɾ���ͻ�
        len_route     = numel(seq_cs);                              %���е�·������
        loc_sat       = [find(seq_cs<=num_sat),len_route+1];             %������·���е�λ��
        seq_cs(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
    elseif sm2 == 4
        %----���ɾ��һ��·����Route Removal----
        sign_fleet  = cumsum(seq_cs<=num_sat,2);           %���ǺͿͻ���Ӧ�ĳ���
        num_fleet   = sign_fleet(end);               %·������
        
        rf           = ceil(rand*num_fleet);         %���ѡȡһ��·��
        loc_r        = find(sign_fleet==rf);         %��ȡ��·����Ӧ��λ��  
        seq_c        = seq_cs(loc_r(2:end));          %��ȡ·���еĿͻ�
        if sum(sat_offl) == 1 && num_fleet ==1         %���ֻ��һ�����ŵ�������ֻ��һ��·��
            s_off  = find(sat_offl==0);                %�رյ�����
            sat_offl = zeros(1,num_sat);                    %���ǵ�״̬
            sat_offl(s_off(ceil(rand*(num_sat-1)))) = 1;    %��һ���رյ�����
            s_offs = [];
        else
            s_offs = seq_cs(loc_r(1))*ones(1,numel(seq_c));%�ͻ����ɲ��������
        end
        seq_cs(loc_r) = [];                           %ɾ����·��
    elseif sm2 == 5
        %----ÿ�����ŵ�����ɾ��k��·����������----
        num_son     = sum(sat_offl);                   %�������ǵ�����
        if num_son>1
            sign_fleet  = cumsum(seq_cs<=num_sat,2);       %���ǺͿͻ���Ӧ�ĳ���
            num_fleet   = sign_fleet(end);           %·������
            dis_f       = zeros(1,num_fleet);        %·���еĿͻ��ڱ����Ǻ��������Ǽ����̾���
            for f = 1:num_fleet
                cus_f    = seq_cs((sign_fleet==f));    %��f��·��
                s        = cus_f(1);                  %����·��������
                cus_f    = cus_f(2:end);              %��f��·���еĿͻ�
                s_on     = sat_offl;  s_on(s)=0;        %�������ŵ�����
                dis_f(f) = (0.8+0.4*rand)*min(min(bsxfun(@plus,dis_sc(s,cus_f),dis_sc(s_on==1,cus_f))));
            end
            [~,lst_f]    = sort(dis_f);               %��ÿ��·���ľ����������
            sat_fleet    = seq_cs(seq_cs<=num_sat);          %������Ӧ������
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
            seq_c   = seq_cs(log_cs);                   %��ȡ��ɾ����·��
            seq_c(seq_c<=num_sat) = [];                %��ȡ·���еĿͻ�
            [~,pos_cus] = ismember(seq_c,seq_cs);       %�ͻ���·���е�λ��
            fleet_cus   = sign_fleet(pos_cus);         %ָ���ͻ���Ӧ�ĳ�����
            s_offs      = sat_fleet(fleet_cus);        %�ͻ�seq_c��Ӧ������
            seq_cs(log_cs) = [];                        %����·����ɾ������·��
        else
            seq_c       = [];
            s_offs      = [];
        end
    end

    off = 1:num_sat;
    if sm3 == 3  %��ԭ·���޸���ʽ
        s_off = [repmat(off(sat_offl==0),[numel(seq_c),1]),s_offs'];
    elseif sm2 == 4 && numel(s_offs)>0 %����·��ɾ����ԭ·����������
        s_off = [off(sat_offl==0),s_offs(1)]; 
    else
        s_off = off(sat_offl==0);
    end
    seq_cs_pre = greedyin2(vrp2e,seq_cs,seq_c,s_off);
    [vrp_pre,vlt_pre] = msecond(vrp2e,seq_cs_pre);
    if (vlt_pre <= vlt_ini ||vlt_pre <= epsilon) && vrp_pre < vrp_ini
        seq_cs = seq_cs_pre;
    else
        vlt_pre = vlt_ini;
        vrp_pre = vrp_ini;
        seq_cs = seq_cs_ini;
    end
    seq_cs_pre = seq_cs;

    %% �ֲ�����
    sm = ceil(rand*2);
    if sm == 1
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
    seq_cs_new = greedyinr2(vrp2e,seq_cs,seq_c);
    [vrp_cs,vlt_cs] = msecond(vrp2e,seq_cs_new);
    if vlt_cs <= vlt_pre && vrp_cs < vrp_pre
        seq_cs = seq_cs_new;
        vrp_pre = vrp_cs;
        vlt_pre = vlt_cs;
    else
        seq_cs = seq_cs_pre;
    end

    %% ���򽻲�
    opt_cus_pre(opt_cus_pre==0)  = [];
    if ~isequal(seq_cs, opt_cus_pre)
        [seq_cs_temp, opt_cus_pre_temp, seq_cs_waite, opt_cus_pre_waite] = crossover(seq_cs, opt_cus_pre, num_sat);
%         seq_cs_temp = [seq_cs_temp, zeros(1, numel(seq_cs) - numel(seq_cs_temp))];
        if ~isempty(seq_cs_waite)
            seq_cs_temp = greedyinr2(vrp2e,seq_cs_temp,seq_cs_waite);
        end
%         opt_cus_pre_temp = [opt_cus_pre_temp, zeros(1, numel(seq_cs) - numel(opt_cus_pre_temp))];
        if ~isempty(opt_cus_pre_waite)
            opt_cus_pre_temp = greedyinr2(vrp2e,opt_cus_pre_temp,opt_cus_pre_waite);
        end
        [vrp_cs_temp,vlt_cs] = msecond(vrp2e,seq_cs_temp);
        [vrp_opt,vlt_opt] = msecond(vrp2e,opt_cus_pre_temp);
        if (vlt_cs<=vlt_pre || vlt_cs<=epsilon) && (vlt_opt<=vlt_pre || vlt_opt<=epsilon)
            if vrp_cs_temp<vrp_opt && vrp_cs_temp<vrp_pre
                seq_cs = seq_cs_temp;
                vrp_pre = vrp_cs_temp;
                vlt_pre = vlt_cs;
            elseif vrp_opt<vrp_cs_temp && vrp_opt<vrp_pre
                seq_cs = opt_cus_pre_temp;
                vrp_pre = vrp_opt;
                vlt_pre = vlt_opt;
            end
        end
    end
    seq_cs(seq_cs==0)  = [];
    if sm2==0
        seq_cs1 = move2(vrp2e,seq_cs); %���ɾ�����пͻ������²�������·�ζԱȽ��
        seq_cs2 = swap2(vrp2e,seq_cs1); %��������пͻ����䵥һ+�����ھӽ�����ԱȽ��
        seq_cs3 = opt2(vrp2e,seq_cs2);  %ÿ����·���ڣ�������ת������·��
        seq_cs4 = opt2x(vrp2e,seq_cs3); %ƴ��ͬһ�����²�ͬ��·���ڵ���·��
        [vrp_cs1,vlt_cs1] = msecond(vrp2e,seq_cs1);
        [vrp_cs2,vlt_cs2] = msecond(vrp2e,seq_cs2);
        [vrp_cs3,vlt_cs3] = msecond(vrp2e,seq_cs3);
        [vrp_cs4,vlt_cs4] = msecond(vrp2e,seq_cs4);
        vrp = [vrp_cs1,vrp_cs2,vrp_cs3,vrp_cs4];
        vlt = [vlt_cs1,vlt_cs2,vlt_cs3,vlt_cs4];
        if ~isempty(min(vrp(vlt<=vlt_pre | vlt<=epsilon)))
            vrp_csn = vrp(vrp==min(vrp(vlt<=vlt_pre | vlt<=epsilon)));
        else
            vrp_csn = Inf;
        end
        seq = {seq_cs1,seq_cs2,seq_cs3,seq_cs4};
        if vrp_csn < vrp_pre
            seq_cs = seq{vrp==min(vrp(vlt<=vlt_pre | vlt<=epsilon))};
        end
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


%��seq_c�еĿͻ����뵽���еĶ���seq_cs�У�s_off���رջ��ֹ������
function seq_r = greedyinr2(vrp2e,route,seq_c)
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