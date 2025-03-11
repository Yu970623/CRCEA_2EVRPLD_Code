function seq_c = adjustsat(vrp2e,seq_c,seq_s,cap_s)
    if rand < 1
        seq_c = adjustsat1(vrp2e,seq_c,seq_s,cap_s);
    else
        seq_c = adjustsat2(vrp2e,seq_c);
    end
end
%% --------------���ݵ�һ����Ϣ��������(�ڶ���ͻ�·��˳�򲻸ı䣬����)-----------------------
function seq_c = adjustsat1(vrp2e,seq_c,seq_s,cap_s)
    %global dis_ds; global fleet;
    dis_ds = vrp2e.dis_ds;
    fleet = vrp2e.fleet;
    %----��ѯ��һ����Ҫ���滻��������Ϣ----
    loc_d = find(seq_s == 0);                           %�������Ķ�Ӧ�����
    num_f = ceil(sum(cap_s)/fleet(1,1));                %��һ��·���ĳ�����
    s_re  = zeros(1,4);                                 %�洢���滻�����Ǻš����滻����С������������������ٵĴ��� 
    for i = 1:num_f

    end
end


%% --------------���ݵڶ�����Ϣ��������(�ڶ���ͻ�·��˳�򲻸ı䣬�������ǲ�����·��)-----------------------
function seq_c = adjustsat2(vrp2e,seq_c)
    %global dis_sc; global num_sat; global num_cus;
    dis_sc = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;
    
    loc_s = [find(seq_c<=num_sat),length(seq_c)+1];     %��һ��·�����������ڵ�λ��
    len_s = length(loc_s)-1;                            %��һ���ܵ�·����Ŀ
    for i = 1:len_s                   
        len_r = loc_s(i+1)-loc_s(i)-1;                  %��·�οͻ���Ŀ
        if len_r > 0
            route  = [seq_c(loc_s(i+1)-1),seq_c(loc_s(i)+1:loc_s(i+1)-1)];     %һ��·���е��������
            %----������С�����ǲ���λ��----
            [cost_s,s_ins]  = min(bsxfun(@minus,dis_sc(1:num_sat,route(1:end-1))+dis_sc(1:num_sat,route(2:end)),dis_sc(route(1:end-1)+(route(2:end)-1)*(num_sat+num_cus))),[],1);
            [~,loc_ins]     = min(cost_s);
            s               = s_ins(loc_ins);
            seq_c(loc_s(i)) = s;
            seq_c(loc_s(i)+1:loc_s(i+1)-1) = [route(loc_ins+1:end),route(2:loc_ins)];
        end
    end
end