%% --------------------------��ȡԴ�ļ��е�����---------------------
function [vrp2e,coord_dep,coord_sat,coord_cus,fleet,demand,type,dis_ds,dis_sc,rad_ds,rad_sc,neib_ss,neib_cc,neib_sc,num_sat,num_cus] = extractdata(set,seq)
    %----Name of Set2-Set4----
    %----total of Set2 is 30----
    Set1 = {'Ca1-2,3,15','Ca1-2,3,30','Ca1-2,3,50','Ca1-2,3,100',...
        'Ca1-3,5,15','Ca1-3,5,30','Ca1-3,5,50','Ca1-3,5,100',...
        'Ca1-6,4,15','Ca1-6,4,30','Ca1-6,4,50','Ca1-6,4,100',...
        'Ca2-2,3,15','Ca2-2,3,30','Ca2-2,3,50','Ca2-2,3,100',...
        'Ca2-3,5,15','Ca2-3,5,30','Ca2-3,5,50','Ca2-3,5,100',...
        'Ca2-6,4,15','Ca2-6,4,30','Ca2-6,4,50','Ca2-6,4,100',...
        'Ca3-2,3,15','Ca3-2,3,30','Ca3-2,3,50','Ca3-2,3,100',...
        'Ca3-3,5,15','Ca3-3,5,30','Ca3-3,5,50','Ca3-3,5,100',...
        'Ca3-6,4,15','Ca3-6,4,30','Ca3-6,4,50','Ca3-6,4,100',...
        'Ca4-2,3,15','Ca4-2,3,30','Ca4-2,3,50','Ca4-2,3,100',...
        'Ca4-3,5,15','Ca4-3,5,30','Ca4-3,5,50','Ca4-3,5,100',...
        'Ca4-6,4,15','Ca4-6,4,30','Ca4-6,4,50','Ca4-6,4,100',...
        'Ca5-2,3,15','Ca5-2,3,30','Ca5-2,3,50','Ca5-2,3,100',...
        'Ca5-3,5,15','Ca5-3,5,30','Ca5-3,5,50','Ca5-3,5,100',...
        'Ca5-6,4,15','Ca5-6,4,30','Ca5-6,4,50','Ca5-6,4,100'};
    %----total of Set3 is 24----
    Set2 = {'Cb1-2,3,15','Cb1-2,3,30','Cb1-2,3,50','Cb1-2,3,100',...
        'Cb1-3,5,15','Cb1-3,5,30','Cb1-3,5,50','Cb1-3,5,100',...
        'Cb1-6,4,15','Cb1-6,4,30','Cb1-6,4,50','Cb1-6,4,100',...
        'Cb2-2,3,15','Cb2-2,3,30','Cb2-2,3,50','Cb2-2,3,100',...
        'Cb2-3,5,15','Cb2-3,5,30','Cb2-3,5,50','Cb2-3,5,100', ...
        'Cb2-6,4,15','Cb2-6,4,30','Cb2-6,4,50','Cb2-6,4,100',...
        'Cb3-2,3,15','Cb3-2,3,30','Cb3-2,3,50','Cb3-2,3,100',...
        'Cb3-3,5,15','Cb3-3,5,30','Cb3-3,5,50','Cb3-3,5,100',...
        'Cb3-6,4,15','Cb3-6,4,30','Cb3-6,4,50','Cb3-6,4,100',...
        'Cb4-2,3,15','Cb4-2,3,30','Cb4-2,3,50','Cb4-2,3,100',...
        'Cb4-3,5,15','Cb4-3,5,30','Cb4-3,5,50','Cb4-3,5,100',...
        'Cb4-6,4,15','Cb4-6,4,30','Cb4-6,4,50','Cb4-6,4,100',...
        'Cb5-2,3,15','Cb5-2,3,30','Cb5-2,3,50','Cb5-2,3,100',...
        'Cb5-3,5,15','Cb5-3,5,30','Cb5-3,5,50','Cb5-3,5,100',...
        'Cb5-6,4,15','Cb5-6,4,30','Cb5-6,4,50','Cb5-6,4,100'};

    Set3 = {'Cc1-2,3,15','Cc1-2,3,30','Cc1-2,3,50','Cc1-2,3,100',...
        'Cc1-3,5,15','Cc1-3,5,30','Cc1-3,5,50','Cc1-3,5,100',...
        'Cc1-6,4,15','Cc1-6,4,30','Cc1-6,4,50','Cc1-6,4,100',...
        'Cc2-2,3,15','Cc2-2,3,30','Cc2-2,3,50','Cc2-2,3,100',...
        'Cc2-3,5,15','Cc2-3,5,30','Cc2-3,5,50','Cc2-3,5,100', ...
        'Cc2-6,4,15','Cc2-6,4,30','Cc2-6,4,50','Cc2-6,4,100',...
        'Cc3-2,3,15','Cc3-2,3,30','Cc3-2,3,50','Cc3-2,3,100',...
        'Cc3-3,5,15','Cc3-3,5,30','Cc3-3,5,50','Cc3-3,5,100',...
        'Cc3-6,4,15','Cc3-6,4,30','Cc3-6,4,50','Cc3-6,4,100',...
        'Cc4-2,3,15','Cc4-2,3,30','Cc4-2,3,50','Cc4-2,3,100',...
        'Cc4-3,5,15','Cc4-3,5,30','Cc4-3,5,50','Cc4-3,5,100',...
        'Cc4-6,4,15','Cc4-6,4,30','Cc4-6,4,50','Cc4-6,4,100',...
        'Cc5-2,3,15','Cc5-2,3,30','Cc5-2,3,50','Cc5-2,3,100',...
        'Cc5-3,5,15','Cc5-3,5,30','Cc5-3,5,50','Cc5-3,5,100',...
        'Cc5-6,4,15','Cc5-6,4,30','Cc5-6,4,50','Cc5-6,4,100'};

    Set4 = {'Cd1-2,3,15','Cd1-2,3,30','Cd1-2,3,50','Cd1-2,3,100',...
        'Cd1-3,5,15','Cd1-3,5,30','Cd1-3,5,50','Cd1-3,5,100',...
        'Cd1-6,4,15','Cd1-6,4,30','Cd1-6,4,50','Cd1-6,4,100',...
        'Cd2-2,3,15','Cd2-2,3,30','Cd2-2,3,50','Cd2-2,3,100',...
        'Cd2-3,5,15','Cd2-3,5,30','Cd2-3,5,50','Cd2-3,5,100', ...
        'Cd2-6,4,15','Cd2-6,4,30','Cd2-6,4,50','Cd2-6,4,100',...
        'Cd3-2,3,15','Cd3-2,3,30','Cd3-2,3,50','Cd3-2,3,100',...
        'Cd3-3,5,15','Cd3-3,5,30','Cd3-3,5,50','Cd3-3,5,100',...
        'Cd3-6,4,15','Cd3-6,4,30','Cd3-6,4,50','Cd3-6,4,100',...
        'Cd4-2,3,15','Cd4-2,3,30','Cd4-2,3,50','Cd4-2,3,100',...
        'Cd4-3,5,15','Cd4-3,5,30','Cd4-3,5,50','Cd4-3,5,100',...
        'Cd4-6,4,15','Cd4-6,4,30','Cd4-6,4,50','Cd4-6,4,100',...
        'Cd5-2,3,15','Cd5-2,3,30','Cd5-2,3,50','Cd5-2,3,100',...
        'Cd5-3,5,15','Cd5-3,5,30','Cd5-3,5,50','Cd5-3,5,100',...
        'Cd5-6,4,15','Cd5-6,4,30','Cd5-6,4,50','Cd5-6,4,100'};

    %----·����ȡ----
    switch set%��ȡcase_name
        case 1
            case_name   = Set1{seq}; % caseName ���������� ����Set2a_E-n22-k4-s6-17
        case 2
            case_name   = Set2{seq};
        case 3
            case_name   = Set3{seq};
        case 4
            case_name   = Set4{seq};
    end
    filename     = ['E:\Yu\LNS\2EVRPTW/',case_name,'.csv'];
    disp([case_name,' read']);
    

    % ��ȡCSV�ļ���ָ����һ��Ϊ��ͷ����������һ��
    data = readtable(filename, 'HeaderLines', 1);
    
    % ��ȡ��Ҫ����
    coords = data{:, 1:2};  % ��һ�к͵ڶ�����������Ϣ
    attributes = data{:, 7};  % �������ǵ������
    demands = data{:, 5};  % �������ǻ���������
    customer_types = data{:, 10};  % ��ʮ���ǿͻ�����
    
    % �ҳ�����Ϊ'depot'��'sat'��'cust'��������
    depot_indices = find(strcmp(attributes, 'depot'));
    sat_indices = find(strcmp(attributes, 'sat'));
    cust_indices = find(strcmp(attributes, 'cust'));
    
    % ����������ȡ������Ϣ
    coord_dep = coords(depot_indices, :);  % depot���������Ϣ
%     coord_dep = coord_dep(1,:);
    coord_sat = coords(sat_indices, :);   % sat���������Ϣ
    coord_cus = coords(cust_indices, :);   % cust���������Ϣ
    
    % ��ȡcust����������Ϳͻ�����
    demand = demands(cust_indices)./10;   % cust��Ļ���������
    type = customer_types(cust_indices);  % cust��Ŀͻ�����
    num_dep = size(coord_dep,1);    %�ֿ����
    num_sat = size(coord_sat,1);    %���Ǹ���
    num_cus = size(coord_cus,1);    %�˿͸���
    % ��ʾ��ʹ����ȡ�ı���
%     disp('depot���������Ϣ:');
%     disp(coord_dep);
%     disp('sat���������Ϣ:');
%     disp(coord_sat);
%     disp('cust���������Ϣ:');
%     disp(coord_cus);
%     disp('cust��Ļ���������:');
%     disp(demand);
%     disp('cust��Ŀͻ�����:');
%     disp(type);

    fleet = zeros(2,2);
    fleet(1,1) = 100;        %һ����������
    fleet(1,2) = 3*ceil(sum(demand)/fleet(1,1));        %���һ����������
    fleet(2,1) = 5;          %�������˻�����
    fleet(2,2) = 3*ceil(sum(demand)/fleet(2,1));        %���������˻�����
    maxFleetPerSat = 2*ceil(fleet(2,2)/num_sat);    %ÿ�����ǵ�������˻���
    demand = demand';
    type = type';
%     type = [zeros(1,num_sat),type];
    demand = [zeros(1,num_sat),demand];     %�����ǵ�0������ӽ��ͻ������б�

    %% ----��ȡ��վ�����Ǽ���룬������ͻ�����룬�ͻ���ͻ�����룬������������Ŀ���ͻ�����----

    %----���� ��վ������֮�� �ľ���Ͷ�Ӧ�Ļ���----
    rad_ds = zeros(num_dep,num_sat);
    for i = 1:num_dep
        xy_coordinate   = bsxfun(@minus,coord_sat',coord_dep(i,:)');
        [rad_ds(i,:),~] = cart2pol(xy_coordinate(1,:),xy_coordinate(2,:));
    end
%     xy_coordinate   = bsxfun(@minus,coord_sat',coord_dep');
%     [rad_ds,dis_ds] = cart2pol(xy_coordinate(1,:),xy_coordinate(2,:));%�ֿ�������֮���ֱ������ϵת��Ϊ������ϵ
    loc = find(rad_ds<0);
    rad_ds(loc)     = rad_ds(loc)+2*pi;

    %----���� �ͻ�������֮�� �ĽǶ�----
    rad_sc = zeros(num_sat,num_cus);
    for i = 1:num_sat
        xy_coordinate   = bsxfun(@minus,coord_cus',coord_sat(i,:)');
        [rad_sc(i,:),~] = cart2pol(xy_coordinate(1,:),xy_coordinate(2,:));
    end
    loc         = find(rad_sc<0);
    rad_sc(loc) = rad_sc(loc)+2*pi; 
    
     %----���� ���ǿͻ��˴�֮�� �ľ���----
    dis_sc = pdist2([coord_sat;coord_cus],[coord_sat;coord_cus],'euclidean');
    dis_sc = dis_sc/10;
    %----���� �����ڽ� �Ĺ�ϵ----
    dis_cc      = dis_sc(num_sat+1:end,num_sat+1:end);
    lst         = 1:num_cus;
    dis_cc(lst+(lst-1)*num_cus) = -1;  %���Լ����Լ��ľ�����Ϊ-1��
    [~,neib_cc] = sort(dis_cc,2);
    neib_cc     = neib_cc+num_sat;
    
    dis_ss = dis_sc(1:num_sat,1:num_sat);
    lst         = 1:num_sat;
    dis_ss(lst+(lst-1)*num_sat) = -1;  %���Լ����Լ��ľ�����Ϊ-1��
    [~,neib_ss] = sort(dis_ss,2);
    
    [~,neib_sc] = sort(dis_sc(1:num_sat,num_sat+1:end),2);
    neib_sc     = neib_sc+num_sat;
    %----���� �ֿ����Ǳ˴�֮�� �ľ���----
    dis_ds  = pdist2([coord_dep;coord_sat],[coord_dep;coord_sat],'euclidean');
    dis_ds = dis_ds/10;
%     dis_ds  = [0,dis_ds;dis_ds',dis_sc(1:num_sat,1:num_sat)];
    
    
   %%----���ǺͿͻ��ķֲ�ͼ----
%     figure(1)
%     plot(satellites(:,1),satellites(:,2),'sb');
%     hold on;
%     plot(customers(:,1),customers(:,2),'*r');
%     hold off;
%     xlabel('x��');
%     ylabel('y��');
%     title('���ǺͿͻ���ķֲ�ͼ');
%     axis equal;
%     box on;
%     grid on;

%������������ṹ��vrp2e
vrp2e.coord_dep = coord_dep;
vrp2e.coord_sat = coord_sat;
vrp2e.coord_cus = coord_cus;
vrp2e.fleet = fleet;
vrp2e.demand = demand;
vrp2e.type = type;
vrp2e.dis_ds = dis_ds;
vrp2e.dis_sc = dis_sc;
vrp2e.rad_ds = rad_ds;
vrp2e.rad_sc = rad_sc;
vrp2e.neib_ss = neib_ss;
vrp2e.neib_cc = neib_cc;
vrp2e.neib_sc = neib_sc;
vrp2e.num_sat = num_sat;
vrp2e.num_cus = num_cus;
vrp2e.case_name = case_name;
vrp2e.E = 500;
vrp2e.V = 10;
%����Set4a�����⳵��Լ��
vrp2e.maxFleetPerSat = maxFleetPerSat;%ÿ���������ɵ�������������
end



