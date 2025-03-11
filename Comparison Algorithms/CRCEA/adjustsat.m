function [seq_c,seq_s,cap_s] = adjustsat(vrp2e,seq_c,seq_s,cap_s,sm_adjust)
    if sm_adjust==1 || sm_adjust==2
        [seq_c,seq_s,cap_s] = adjustsat1(vrp2e,seq_c,seq_s,cap_s,sm_adjust);
    else
        seq_c = adjustsat2(vrp2e,seq_c);
    end
end

%% --------------���ݵ�һ����Ϣ��������(�ڶ���ͻ�·��˳�򲻸ı�)-----------------------
function [seq_c,seq_s,cap_s] = adjustsat1(vrp2e,seq_c,seq_s,cap_s,sm_adjust)
    dis_ds = vrp2e.dis_ds;
    num_sat = vrp2e.num_sat;
    num_dep = length(dis_ds)-num_sat;
    [vrp2,vlt2] = msecond(vrp2e,seq_c);
    [vrp1,vlt1] = mfirst(vrp2e,seq_s,cap_s);
    %----��ѯ��һ����Ҫ���滻��������Ϣ----
    loc_d = [find(seq_s <= num_dep & seq_s ~= 0),find(seq_s, 1, 'last')+1];    %�������Ķ�Ӧ����� depot
    len_d = length(loc_d)-1;  
    for i = 1:len_d 
        if loc_d(i+1)-loc_d(i)>2    %���ڵ�һ���ÿһ����·����ÿһ��������ȥ��ÿ��route���ĸ����ǵ�������С���ͰѸ�����ŵ���һ��ͬһ·��������������
            sat_route = seq_s(loc_d(i)+1:loc_d(i+1)-1); %��ǰ·�������е�����
            cap_route = cap_s(loc_d(i)+1:loc_d(i+1)-1); %��ǰ·�������е�����
            if sm_adjust==1     %�˴�ѡ������С�����ǽ��为��Ŀͻ�������������
                mincap_sat = sat_route(find(cap_route==min(cap_route),1));
                loc_sat = find(seq_s==mincap_sat,1);
                for j = sat_route(sat_route~=mincap_sat)
                    new_route1 = seq_s;
                    new_cap1 = cap_s;
                    new_cap1(new_route1==j) = new_cap1(new_route1==j)+new_cap1(loc_sat);
                    new_route1(loc_sat) = [];
                    new_cap1(loc_sat) = [];
                    new_route2 = seq_c;
                    new_route2(new_route2 == mincap_sat - num_dep) = j - num_dep;
                    [new_vrp2 ,new_vlt2] = msecond(vrp2e,new_route2);
                    [new_vrp1,new_vlt1] = mfirst(vrp2e,new_route1,new_cap1);
                    if new_vrp2+new_vrp1<vrp2+vrp1 && new_vlt2+new_vlt1<=vlt2+vlt1
                        vrp2 = new_vrp2;
                        vrp1 = new_vrp1;
                        seq_c = new_route2;
                        seq_s = new_route1;
                        cap_s = new_cap1;
                    end
                end
            else    %�˴����ѡ�����ǽ��为��Ŀͻ���������ͬ·������
                rand_sat = sat_route(randi(length(sat_route)));
                loc_sat = find(seq_s==rand_sat,1);
                for j = sat_route(sat_route~=rand_sat)
                    new_route1 = seq_s;
                    new_cap1 = cap_s;
                    new_cap1(find(new_route1==j)) = new_cap1(find(new_route1==j))+new_cap1(loc_sat);
                    new_route1(loc_sat) = [];
                    new_cap1(loc_sat) = [];
                    new_route2 = seq_c;
                    new_route2(new_route2 == rand_sat - num_dep) = j - num_dep;
                    [new_vrp2 ,new_vlt2] = msecond(vrp2e,new_route2);
                    [new_vrp1,new_vlt1] = mfirst(vrp2e,new_route1,new_cap1);
                    if new_vrp2+new_vrp1<vrp2+vrp1 && new_vlt2+new_vlt1<=vlt2+vlt1
                        vrp2 = new_vrp2;
                        vrp1 = new_vrp1;
                        seq_c = new_route2;
                        seq_s = new_route1;
                        cap_s = new_cap1;
                    end
                end
            end
%         else    %�˴�����һ����������ӵ��òֿ�������������·����
%             single_sat = seq_s(loc_d(i)+1);  %�ҵ��õ�һ���Ǳ��
%             single_cap = cap_s(loc_d(i)+1);  %�ҵ��õ�һ���ǻ�����
%             rand_loc_d = find(seq_s == seq_s(loc_d(i)));    %������ͬ�ֿ���±�
%             if length(rand_loc_d)==1
%                 continue;
%             else
%                 rand_loc_d(rand_loc_d==find(seq_s==single_sat)-1) = [];%�ų���ǰ�±�
%                 min_newvrp1 = zeros(1,length(rand_loc_d));
%                 min_newroute1 = cell(1,length(rand_loc_d));
%                 min_newcap1 = cell(1,length(rand_loc_d));
%                 for j = length(rand_loc_d)
%                     if loc_d(find(loc_d == rand_loc_d(j))+1)-1 > length(seq_s)
%                         length(seq_s);
%                     end
%                     pre_seq_s = seq_s(rand_loc_d(j):loc_d(find(loc_d == rand_loc_d(j))+1)-1);   %�����òֿ����������·��ԭʼ����
%                     pre_cap_s = cap_s(rand_loc_d(j):loc_d(find(loc_d == rand_loc_d(j))+1)-1);   %�����òֿ����������·��ԭʼ������
%                     count_sat = zeros(length(pre_seq_s),length(pre_seq_s)+1);
%                     count_cap = zeros(length(pre_cap_s),length(pre_cap_s)+1);
%                     count_vrp1 = zeros(1,length(pre_seq_s));
%                     count_vlt1 = zeros(1,length(pre_cap_s));
%                     for k = 2:length(pre_seq_s) + 1
%                         count_sat(k-1,:) = [pre_seq_s(1:k-1), single_sat, pre_seq_s(k:end)];
%                         count_cap(k-1,:) = [pre_cap_s(1:k-1), single_cap, pre_cap_s(k:end)];
%                         Full_sat = [seq_s(1:loc_d(i)-1),count_sat(k-1,:),seq_s(find(seq_s==pre_seq_s(end))+1:end)];
%                         Full_cap = [cap_s(1:loc_d(i)-1),count_cap(k-1,:),cap_s(find(seq_s==pre_seq_s(end))+1:end)];
%                         [count_vrp1(k-1),count_vlt1(k-1)]=mfirst(vrp2e,Full_sat,Full_cap);
%                     end
%                     min_indes = find(min(count_vrp1(count_vlt1==0)));
%                     if isempty(min_indes)
%                         min_newvrp1(j) = 0;
%                         min_newroute1{j} = 0;
%                         min_newcap1{j} = 0;
%                     else
%                         min_newvrp1(j) = count_vrp1(min_indes);
%                         min_newroute1{j} = [seq_s(1:loc_d(i)-1),count_sat(min_indes),seq_s(find(seq_s==pre_seq_s(end))+1:end)];
%                         min_newcap1{j} = [cap_s(1:loc_d(i)-1),count_cap(min_indes),cap_s(find(seq_s==pre_seq_s(end))+1:end)];
%                     end
%                 end
%                 if vrp1 > min_newvrp1(min(find(min_newvrp1(:) > 0)))
%                     seq_s = min_newroute1{min(find(min_newvrp1(:) > 0))};
%                     cap_s = min_newcap1{min(find(min_newvrp1(:) > 0))};
%                 end
%             end
        end
    end
end

%% --------------���ݵڶ�����Ϣ��������(�ڶ���ͻ�·��˳�򲻸ı�)-----------------------
function seq_c = adjustsat2(vrp2e,seq_c)
    num_sat = vrp2e.num_sat;
    loc_s = [find(seq_c<=num_sat & seq_c~=0),find(seq_c, 1, 'last')+1];     %�ڶ���·�����������ڵ�λ��
    len_s = length(loc_s)-1;                            %��һ���ܵ�·����Ŀ
    for i = 1:len_s
        cost_new = zeros(1,num_sat);
        vlt_new = zeros(1,num_sat);
        len_r = loc_s(i+1)-loc_s(i)-1;                  %·���еĿͻ���Ŀ
        if len_r > 0  %1.�����ĸ����ǣ� 2.�����λ�����ĶԿͻ���֮�䣿
            seq_cus  = seq_c(loc_s(i)+1:loc_s(i+1)-1);     %��·�εĿͻ�����
            for s = 1:num_sat
                [cost_new(s),vlt_new(s)] = msecond(vrp2e,[s,seq_cus]);
            end
            %----������С�����ǲ���λ��----
            if isempty(cost_new(vlt_new == 0))
                seq_c(loc_s(i)) = seq_c(loc_s(i));
            else
                seq_c(loc_s(i)) = find(cost_new==min(cost_new(vlt_new == 0)),1);
            end
        end
    end
end




