clc;clear;
seq   = [1,2,3,4,5,6,7,8, 9,10,11,12];
route = [1,5,9,7,2,8,6,3,10,11, 2,12,3,4];
demand= [0,0,0,0,1,2,3,4,5,6,7,8];
seq_c = [9,11,5];
s  = 2;
f  = 3;
ns = 4;
%1----������·��----
ns = 4;
len_route   = numel(route);                                 %���е�·������
loc_sat     = [find(route<=ns),len_route+1];                %������·���е�λ��
route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %ɾ���ճ�������·��
route

%----�ͻ�����Ϣ----
[~,pos_cus] = ismember(seq_c,route);      %����ͻ���·���е�λ��
% pos_cus     = find(route==seq_c);       %�����ͻ���·���е�λ��
pos_cus

%ָ���ͻ����ڵĳ���
[~,pos_cus] = ismember(seq_c,route);      %�ͻ���·���е�λ��
sign_fleet  = cumsum(route<=ns,2);        %���ǺͿͻ���Ӧ�ĳ���
fleet_cus   = sign_fleet(pos_cus);
fleet_cus

%���пͻ���Ӧ�ĳ����ţ�ǰ�����������)
[~,pos_cus] = find(route>ns);           %�ͻ���·���е�λ��
sign_fleet  = cumsum(route<=ns,2);      %���ǺͿͻ���Ӧ�ĳ���
fleet_cus   = sign_fleet(pos_cus);       %�ͻ����߼�λ�ö�Ӧ�ĳ�����
fleet_cus(route(pos_cus)-ns) = fleet_cus;   %���пͻ���Ӧ�ĳ�����
fleet_cus

%ָ���ͻ�seq_c��Ӧ������
[~,pos_cus] = ismember(seq_c,route);       %�ͻ���·���е�λ��
sign_fleet  = cumsum(route<=ns,2);           %���ǺͿͻ���Ӧ�ĳ���
fleet_cus   = sign_fleet(pos_cus);         %ָ���ͻ���Ӧ�ĳ�����
sat_fleet   = route(route<=ns);             %������Ӧ������
sat_cus     = sat_fleet(fleet_cus);        %�ͻ�seq_c��Ӧ������
sat_cus

%���пͻ���Ӧ������
[~,pos_cus] = find(route>ns);          %�ͻ���·���е�λ��
sign_fleet  = cumsum(route<=ns,2);     %���ǺͿͻ���Ӧ�ĳ���
fleet_cus   = sign_fleet(pos_cus);     %�ͻ����߼�λ�ö�Ӧ�ĳ�����
fleet_cus(route(pos_cus)-ns) = fleet_cus;   %���пͻ���Ӧ�ĳ�����
sat_fleet   = route(route<=ns);             %������Ӧ������
sat_cus     = sat_fleet(fleet_cus); 
sat_cus

%----���ǵ���Ϣ(����һ�����Ƕ�Ӧ�������)----
pos_sat    = find(route<=ns);             %������·���е�λ��
pos_sat

%����s��Ӧ�ĳ�������
sat_fleet    = route(route<=ns);          %������Ӧ������
sign_fleet_s = find(sat_fleet==s);        %����s��Ӧ�ĳ�������
sign_fleet_s

%����s������·��
sat_fleet     = route(route<=ns);                   %������Ӧ������
sign_fleet_s  = find(sat_fleet==s);                 %����s��Ӧ�ĳ�������
sign_fleet    = cumsum(route<=ns,2);                  %���ǺͿͻ���Ӧ�ĳ���
[logic_loc,~] = ismember(sign_fleet,sign_fleet_s);  %����s����Ӧ�ͻ�����Ϊ1
cus_sat       = route(logic_loc);                   %����s������·������s��
% cus_sat       = cus_sat(cus_sat>ns);                 %��ȡ·���еĿͻ��㣨����s��
cus_sat

%----·������������Ϣ-----
%·������������Ӧ������
pos_sat    = find(route<=ns);            %������·���е�λ��
sat_fleet  = route(pos_sat);             %·����Ӧ������
sat_fleet

%����·��������f���е����пͻ�
sign_fleet = cumsum(route<=ns,2);     %���ǺͿͻ���Ӧ�ĳ���
cus_fleet  = route((sign_fleet==f));  %��f��·���������ǣ�
cus_fleet

%�������·�����ܵ�����
num_fleet     = sum(route<=ns);          %·����Ŀ��ȥ�ճ���
sign_fleet    = cumsum(route<=ns,2);     %���ǺͿͻ���Ӧ�ĳ���
logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');  %��ȡ�������߼���ַ
cap_fleet     = zeros(1,num_fleet);
for i = 1:num_fleet
    cap_fleet(i) = sum(demand(route(logical_fleet(i,:))));
end
cap_fleet

%���н����߼��ĵ㣨�����ǺͿͻ�����Ӧ�ĳ�������
num_fleet     = sum(route<=ns);          %·����Ŀ��ȥ�ճ���
sign_fleet    = cumsum(route<=ns,2);     %���ǺͿͻ���Ӧ�ĳ���
logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');  %��ȡ�������߼���ַ
cap_fleet     = zeros(1,num_fleet);
for i = 1:num_fleet
    cap_fleet(i) = sum(demand(route(logical_fleet(i,:))));
end
cap_fleet

%·����ÿһ���Ӧ�ĳ�������
num_fleet     = sum(route<=ns);          %·����Ŀ��ȥ�ճ���
sign_fleet    = cumsum(route<=ns,2);     %���ǺͿͻ���Ӧ�ĳ���
logical_fleet = bsxfun(@eq,sign_fleet,(1:num_fleet)');  %��ȡ�������߼���ַ
cap_fleet     = zeros(1,num_fleet);
for i = 1:num_fleet
    cap_fleet(i) = sum(demand(route(logical_fleet(i,:))));
end
cap_sc = cap_fleet(sign_fleet);
cap_sc






