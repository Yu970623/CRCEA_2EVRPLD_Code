%% --------------------------��ʼ���ڶ���--------------------------
    %�ڶ����ʼ����ʽ��
    %step1: ���տͻ������ǵ������Ⱥ�������������ĵľ���������ǻ�ȡ��Ȩ�أ�
    %step2: ����������С�������̶ķ�ʽ�����Ƿ��䳵����
    %step4�����ͻ����������
    %step3����ÿ����������Զ�ͻ�
    %step4�����ͻ���������ͬ�ĳ���
    %step5����ÿ��·���еĿͻ�����
function [route_2nd,vrp2,vlt2] = initcus(vrp2e,num_pop)
    %global fleet; global demand; global dis_ds; global dis_sc; global num_sat; global num_cus;
    fleet = vrp2e.fleet;
    demand = vrp2e.demand; 
    dis_sc = vrp2e.dis_sc;  
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    seq_cus   = zeros(1,2*num_cus);                      %Ԥ����洢�ͻ�����Ϊ����2*nc�������ռ�ɾ��
 
    num_p      = 2*num_pop;           %��Ⱥ��ģ
%     len_route2 = num_cus+fleet(2,2);
    route_2nd  = zeros(num_p,2*num_cus);     %�洢�ڶ������г�ʼ��·��

    for i = 1:num_p
        %----���վ���������̶Ľ��ͻ����������----
        sat_cus = zeros(1,num_cus);              %�洢ÿ���ͻ�����������
        for c = 1:num_cus
            sat_cus(c) = roulette(1./dis_sc(1:num_sat,c+num_sat),1);
        end
        %----������ǿͻ�������CW�㷨����
        cap_s = zeros(1,num_sat);
        nrs   = 0;   %·���ܳ���
        for s = 1:num_sat
            cus_s                 = find(sat_cus==s)+num_sat;
            cap_s(s)              = sum(demand(cus_s));
            route                 = algcw2(vrp2e,s,cus_s);
            nr                    = numel(route);
            seq_cus(nrs+1:nrs+nr) = route;
            nrs                   = nrs+nr;
        end
        route_2nd(i,:) =  seq_cus;
        seq_cus   = zeros(1,2*num_cus); 
    end
    %----ѡȡ�Ϻõ�num_pop������----
    [vrp2,vlt2] = msecond(vrp2e,route_2nd);
    [~,loc]     = sort(vrp2.*(vlt2*1000+1)); 
    route_2nd   = route_2nd(loc(1:num_pop),:);
    vrp2        = vrp2(loc(1:num_pop));
    vlt2        = vlt2(loc(1:num_pop));
end

 %% --------------���ͻ�������(����CW)--------------------
 function rount = algcw2(vrp2e,s,cus_s)
    V = vrp2e.V;
    E = vrp2e.E;
    num_sat = vrp2e.num_sat;
    demand = vrp2e.demand;
    capacity = vrp2e.fleet(2,1);
    dis_sc = vrp2e.dis_sc;
    demand_cus = demand(cus_s);
    dis_stoc = dis_sc(s,cus_s);
    c_type = vrp2e.type;
    c_type = c_type(cus_s-num_sat);
    nc         = numel(dis_stoc);     %�ͻ�����
    link       = zeros(2,nc);         %�ͻ����ӵ������ͻ����洢�뱾�ͻ����ӵ������ͻ�
    nlink      = zeros(1,nc);         %�ͻ����ӵĴ���������ʶ��ÿͻ��Ƿ񻹿�����
    seq_veh    = 1:nc;                %�ͻ���Ӧ�ĳ�����ţ����ڼ��㳵�����ػ���
    cost_stoc = (5+5+demand_cus).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4).*dis_stoc./V;
    c_rou = arrayfun(@(i) [s,cus_s(i),s],1:length(cus_s), 'UniformOutput', false);
    cap_veh = cellfun(@(i)energy(vrp2e,i),c_rou, 'UniformOutput', false);
    rou = arrayfun(@(i, j) [s,cus_s(i), cus_s(j),s], repmat((1:length(cus_s))', 1, length(cus_s)), repmat(1:length(cus_s), length(cus_s), 1), 'UniformOutput', false);
    cap_ctoc = cellfun(@(i)energy(vrp2e,i),rou, 'UniformOutput', false);
    dis_rou = cellfun(@(route) arrayfun(@(i) dis_sc(route(i), route(i+1)), 1:length(route)-1), rou, 'UniformOutput', false);
    Energy_c = cellfun(@(i,j)sum((5+i)*(3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4).*(j/V)), cap_ctoc,dis_rou, 'UniformOutput', false);
    cw_cc      = reshape(zeroDiagonal(bsxfun(@plus,cost_stoc,cost_stoc'))-zeroDiagonal(cell2mat(Energy_c))+0.01,1,[]);  %�����ʡ�ķ���
    [~,srt_cp] = sort(cw_cc,'descend');    %����ʡ����������
    npair      = nc*(nc-1);    %���е����
    srt_cp     = srt_cp(1:npair);          %��ȡ�����Ǿ�������
    cr         = mod(srt_cp-1,nc)+1;  %����ֵ��Ӧ����(�����ж�Ӧ�������ӹ�ϵ)
    cl         = ceil(srt_cp/nc);     %����ֵ��Ӧ����
    sorted_pairs = sort([cr(:) cl(:)], 2);  % ��ÿ������е�Ԫ�������Ա�ʶ���ظ���Ԫ�ض�
    unique_idx = true(1, length(cr)); % ��ʼ�����ڱ���Ψһ�Ե�����
    rount   = zeros(1,2*nc+1);          %����ܷ�����������
    % ����ÿ��Ԫ�ضԣ��ҵ��ظ���Ԫ�ضԣ������Ϊɾ��
    for i = 1:length(cr)-1
        for j = i+1:length(cr)
            if isequal(sorted_pairs(i,:), sorted_pairs(j,:))
                unique_idx(j) = false;
            end
        end
    end
    % ʹ��Ψһ������ȡ������Ԫ��
    cr = cr(unique_idx);
    cl = cl(unique_idx);
%     [cr, cl] = adjust_indices(cr, cl);
    % ��ʼ��ѭ��������
    l = 1;
    % ѭ����������
    while l <= length(cr)
        % ��ȡ��ǰ cr �� cl �е�Ԫ��
        current_cr = cr(l);
        current_cl = cl(l);
        % �ҵ������뵱ǰ cr(i) �� cl(i) ��ͬ��Ԫ������
        indices_to_remove = (cr == current_cr) | (cl == current_cl);
        % ������ǰԪ�أ���ɾ��������ͬ��Ԫ��
        indices_to_remove(l) = false;  % ������ǰԪ�أ����⽫��ɾ��
        % ɾ����Щ��������Ԫ��
        cr(indices_to_remove) = [];
        cl(indices_to_remove) = [];
        % �ƶ�����һ��Ԫ��
        l = l + 1;
    end
    [cr, cl] = adjust_indices(cr, cl);
    [cr, cl] = remove_conflicting_elements(cr, cl);
    B = setdiff(1:nc, [cr cl]);
    % ��һ��B�е�Ԫ����ӵ�cr��cl��
    for i = 1:length(B)
        if ~isempty(cr)
            cl = [cr(1),cl]; 
            cr = [B(i),cr];
        else
            cr = B(i);
            cl = B(i);
        end
    end
    seq_cus = zeros(1,2*nc+1);
    if isempty(cr)
        rount = [];
    else
        seq_cus(1:2) = [s,cus_s(cr(1))];
    end
    k = 1;
    j = 0;
    for i = 1:length(cr)
        nv1 = seq_veh(cr(i));                %��ȡ�ͻ�1��Ӧ�ĳ������
        nv2 = seq_veh(cl(i));                %��ȡ�ͻ�2��Ӧ�ĳ������ nv1<nv2
        if strcmp(c_type(cl(i)), 'pickup')
            cap = [cap_veh{cr(i)},cap_veh{cr(i)}(end)+demand_cus(cl(i))];
        else
            cap = [cap_veh{cr(i)}+demand_cus(cl(i)),cap_veh{cr(i)}(end)];
        end
        if i > 1
            if cr(i) == cl(i-1)   %��Ȼ����(���غ󡢷ǳ��ؾ���)
                route_cap = [s,seq_cus(seq_cus > num_sat),cus_s(cl(i)),s];
                j = 0;
            elseif cr(i) ~= cl(i-1)  && j == 1
                route_cap = [s,seq_cus(seq_cus > num_sat),s];
                rount(k:k+length(route_cap)-2) = route_cap(1:end-1);
                k = k + length(route_cap)-1;
                seq_cus = zeros(1,2*nc+1);
                seq_cus(1:2) = [s,cus_s(cr(i))];
                route_cap = [s,seq_cus(seq_cus > num_sat),cus_s(cl(i)),s];
                j = 0;
            else
                rount(k:k+length(route_cap)-2) = route_cap(1:end-1);
                k = k + length(route_cap)-1;
                seq_cus = zeros(1,2*nc+1);
                seq_cus(1:2) = [s,cus_s(cr(i))];
                route_cap = [s,seq_cus(seq_cus > num_sat),cus_s(cl(i)),s];
            end
        else
            route_cap = [s,seq_cus(seq_cus > num_sat),cus_s(cl(i)),s];
        end
        seq_cus(1:length(route_cap)) = route_cap;
        dis_cap = cell2mat(arrayfun(@(i) dis_sc(route_cap(i), route_cap(i+1)), 1:length(route_cap)-1,'UniformOutput', false));
        Energy_cap = sum(cell2mat(arrayfun(@(i,j)sum((5+i)*(3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4).*(j/V)), cap,dis_cap, 'UniformOutput', false))); 
        %----���������(ǰ���ظ�)������ͬһ·������������----
        if nlink(cr(i))<2 && nlink(cl(i))<2 && nv1~= nv2 && max(cap) <= capacity && Energy_cap <= E
                seq_veh(seq_veh==nv2) = nv1;                        %������nv2�пͻ���Ӧ��������Ϊnv1
                indices = find(seq_veh == nv1);
                % ʹ�� cellfun ����Щ������Ӧ�� cell ���и�ֵ
                cap_veh(indices) = cellfun(@(x) cap, cap_veh(indices), 'UniformOutput', false);
                nlink(cr(i)) = nlink(cr(i))+1;                      %���¿ͻ�1��������Ŀ
                nlink(cl(i)) = nlink(cl(i))+1;                      %���¿ͻ�2��������Ŀ
        else
            rount(k:k+length(route_cap)-3) = route_cap(1:end-2);
            k = k + length(route_cap)-2;
            seq_cus = zeros(1,2*nc+1);
            seq_cus(1:2) = [s,cus_s(cl(i))];
            route_cap = [s,seq_cus(seq_cus > num_sat),s];
            j = 1;
        end
        if i == length(cr) && cr(i)~=cl(i)
            rount(k:k+length(route_cap)-2) = route_cap(1:end-1);
            k = k + length(route_cap)-1;
        end
    end    
    rount(k:end) = []; %���û�д洢�ͻ��Ŀռ�
 end

 function A = zeroDiagonal(A)
    % ������A�ĶԽ���Ԫ����Ϊ0
    [n, m] = size(A);
    if n ~= m
        error('��������Ƿ���');
    end
    A(1:n+1:n^2) = 0; % ���Խ���Ԫ����Ϊ0
 end
 function [cr_new, cl_new] = adjust_indices(cr, cl)
    % ��ʼ�� cr_new �� cl_new Ϊԭʼ����
    cr_new = cr;
    cl_new = cl;
    
    % ��ȡ����ĳ���
    n = length(cr_new);
    
    % ��һ����ȷ����ͬԪ���� cl �е�˳������ cr
    for i = 1:n
        for j = 1:n
            % ��� cl �� cr �е�ǰ�е�˳��
            if cl_new(i) == cr_new(j) && i > j
                % �� cl_new(i) �ƶ��� cl_new(j) ��ǰ��
                cl_new = [cl_new(1:j-1), cl_new(i), cl_new(j:i-1), cl_new(i+1:end)];
                
                % �� cr_new(i) �ƶ��� cr_new(j) ��ǰ��
                cr_new = [cr_new(1:j-1), cr_new(i), cr_new(j:i-1), cr_new(i+1:end)];
                % ���������������ظ�����
                break;
            end
        end
    end
    
    % �ڶ�����ͨ���������ȷ�� cl(i) = cr(i+1)
    i = 1;
    while i < n
        % �� cr_new �в��� cl_new ��Ӧ��Ԫ��
        index_cr = find(cr_new == cl_new(i), 1);
        if ~isempty(index_cr) && index_cr > i
            % �����ҵ���Ԫ�ص����ʵ�λ��
            if i+1 <= n
                cr_new = [cr_new(1:i), cr_new(index_cr), cr_new(i+1:index_cr-1), cr_new(index_cr+1:end)];
                cl_new = [cl_new(1:i), cl_new(index_cr), cl_new(i+1:index_cr-1), cl_new(index_cr+1:end)];
            end
            
            % ���³���
            n = length(cr_new);
        end
        % �ƶ�����һ��λ��
        i = i + 1;
    end
end

function [cr_new, cl_new] = remove_conflicting_elements(cr, cl)
    % ��ʼ�� cr_new �� cl_new Ϊԭʼ����
    cr_new = cr;
    cl_new = cl;
    
    % ��ʼ��Ҫɾ��������
    indices_to_remove_cl = [];
    indices_to_remove_cr = [];
    
    % ���� cl �е�ÿ��Ԫ��
    for i = 1:length(cl)
        % ��ȡ��ǰԪ��
        element = cl(i);
        
        % ���ҵ�ǰԪ���� cr �е�λ��
        index_cr = find(cr == element);
        
        % ��� cl �е��������� cr �е�����������ɾ��
        if ~isempty(index_cr) && i > index_cr
            % ���Ҫɾ���� cl ����
            indices_to_remove_cl = [indices_to_remove_cl, i];
            % ���Ҫɾ���� cr ��������Ӧ cl ������
            indices_to_remove_cr = [indices_to_remove_cr, index_cr];
        end
    end
    
    % ɾ�� cl �в�����Ҫ���Ԫ��
    cl_new(indices_to_remove_cl) = [];
    
    % ���� cr ��������Ϊ cl ��ɾ����cr ������Ҳ��Ҫ����
    cr_new(indices_to_remove_cl) = [];
end