%% --------------------------�ڶ���·��ȫ���ƻ�-------------------------
%seq_cs:��ǰ·����seq_c��ɾ���Ŀͻ�
function [route,seq_c,sat_offl,s_offs] = destroyl(vrp2e,route,sat_offl,sm)
    dis_sc = vrp2e.dis_sc;  
    neib_sc = vrp2e.neib_sc; 
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;

    s_offs = [];              %ɾ���ͻ���Ӧ������
    route(route==0) = [];     %��·��������0Ԫ��ȥ��
    if sm == 1
        %----�ر�һ�����ŵ����ǣ����ȫ���ر��򿪷�һ������:Satellite Removal----
        if sum(sat_offl) == 1 %���ֻ��һ�����ŵ�����
            s_off  = find(sat_offl==0);              %�رյ�����
            sat_offl = zeros(1,num_sat);                  %���ǵ�״̬
            sat_offl(s_off(ceil(rand*(num_sat-1)))) = 1;  %��һ���رյ�����
            route  = [];                             %����·��Ϊ��
            seq_c   = num_sat+1:num_sat+num_cus;                    %����ȫ���ͻ�������
        else
            s_on              = find(sat_offl==1);                   %���ŵ�����
            s_off             = s_on(ceil(rand*numel(s_on)));        %���رյ�����
            sat_offl(s_off)   = 0;                                   %�ر�����
            sat_fleet         = route(route<=num_sat);                    %������Ӧ������
            sign_fleet_s      = find(sat_fleet==s_off);              %����s��Ӧ�ĳ�������
            sign_fleet        = cumsum(route<=num_sat,2);                 %���ǺͿͻ���Ӧ�ĳ���
            [logic_loc,~]     = ismember(sign_fleet,sign_fleet_s);   %����s����Ӧ�ͻ�����Ϊ1
            cus_sat           = route(logic_loc);                    %����s������·������s��
            seq_c             = cus_sat(cus_sat>num_sat);                 %��ȡ·���еĿͻ��㣨����s��
            route(logic_loc)  = [];                                  %ɾ��s_off��Ӧ������·��
        end
    elseif sm == 2
        %----����һ���رյ����ǣ�Ȼ��ɾ����֮�ٽ���q���ͻ�:Satellite Open----
        ns_off   = num_sat-sum(sat_offl);                     %seq_offl��0Ϊ�رգ�1Ϊ����
        if ns_off>0
            s_off          = find(sat_offl==0);          %�رյ�����
            s_on           = s_off(ceil(rand*ns_off));   %���ŵ�����           
            sat_offl(s_on) = 1;                          %���ŵ���������Ϊ1
            ndl            = ceil(rand*min(60,0.4*num_cus));  %��ɾ���Ŀͻ���Ŀ
            seq_c          = neib_sc(s_on,1:ndl);        %ѡȡ��ͻ��ڽ���ndl���ͻ�   
            [~,loc_c]      = ismember(seq_c,route);      %����ͻ���·���е�λ��
            sign_fleet     = cumsum(route<=num_sat,2);        %���ǺͿͻ���Ӧ�ĳ���
            fleet_cus      = sign_fleet(loc_c);          %ָ���ͻ���Ӧ�ĳ�����
            sat_fleet      = route(route<=num_sat);           %������Ӧ������
            s_offs         = sat_fleet(fleet_cus);       %�ͻ�seq_c��Ӧ������
            route(loc_c)  = [];                          %��·����ɾ���ͻ�
            len_route      = numel(route);                              %���е�·������
            loc_sat        = [find(route<=num_sat),len_route+1];             %������·���е�λ��
            route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
        else
            seq_c  = [];
            s_offs = [];
        end
    elseif sm == 3
        %----�ȹر�һ�����ǣ��ٻ��ھ��밴���̶�ѡ�񿪷�һ������:Satellite Swap----
        s_on              = find(sat_offl==1);                   %���ŵ�����
        s_off             = s_on(ceil(rand*numel(s_on)));        %���رյ�����
        sat_offl(s_off)   = 0;                                   %�����ǹر�
        sat_fleet         = route(route<=num_sat);                    %������Ӧ������
        sign_fleet_s      = find(sat_fleet==s_off);              %����s��Ӧ�ĳ�������
        sign_fleet        = cumsum(route<=num_sat,2);                 %���ǺͿͻ���Ӧ�ĳ���
        [logic_loc,~]     = ismember(sign_fleet,sign_fleet_s);   %����s����Ӧ�ͻ�����Ϊ1
        cus_sat           = route(logic_loc);                    %����s������·������s��
        seq_c             = cus_sat(cus_sat>num_sat);                 %��ȡ·���еĿͻ��㣨����s��
        route(logic_loc) = [];                                   %ɾ��s_off��Ӧ������·��
        dis               = dis_sc(s_off,1:num_sat);                  %ɾ���������������Ǽ�ľ���
        dis(s_off)        = inf;                                 %����ɾ�����ǵ��Լ��ľ���Ϊinf
        s_on = roulette(1./dis,1);                               %�������̶Ŀ���һ������
        sat_offl(s_on) = 1;                                      %���ÿ��ŵ�����
    else 
        disp('sm���ܳ���3��');
        seq_c = [];
    end
end