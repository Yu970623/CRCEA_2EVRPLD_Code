%% -----------------------------������-----------------------------
%%  EHG���� stochastic ranking ����Լ������
%% Լ��Υ��ֵ�����ط�ʹ�ã�1.�ƻ��޸����½��Ƿ�����ɽ⣻2. select ��ǰpopulation����һ��population֮���ѡ��
clear all; close all; %clc; 
rng('shuffle');
Pf=0.65;
num_pop     = 10;        %��Ⱥ��С
max_run     = 20;         %�ظ�ʵ�����25
max_gen     = 1000;      %ÿ��ʵ������������
seq_dataset = [60,60,60,60];
for set = 1:4
    seq_data = seq_dataset(set);  %�ļ����ڵĲ�����������
    for seq = 1:seq_data
        %----��ȡ�ļ���ԭʼ���ݣ���վ�����ǺͿͻ������꣬������Ϣ���ͻ�����,��վ�����Ǽ���룬������ͻ�����룬�ͻ���ͻ�����룬��վ���������߽Ƕȣ�������ͻ����߽Ƕ�
        [vrp2e,coord_dep,coord_sat,coord_cus,fleet,demand,type,dis_ds,dis_sc,rad_ds,rad_sc,neib_ss,neib_cc,neib_sc,num_sat,num_cus] = extractdata(set,seq);
        case_name = vrp2e.case_name;
        %----������ά���йصĲ���----
        nrun1       = min(2*num_sat,2);    %ÿ�ε����е�һ������Ż�����
        nrun2       = min(2*num_cus,2);    %ÿ�ε����еڶ�������Ż�����
        %----�洢ÿ��ʵ������Ž��----opt_cap
        num_sc         = 2*(num_cus+fleet(2,2));  %�ͻ���+���˻�*2
        num_ds         = 2*(num_sat+fleet(1,2));%����*2+����*2
        opt_vrps    = zeros(1,max_run);      %�洢���������ظ�ʵ�����е�·������
        opt_vlts    = zeros(1,max_run);     %�洢���������ظ�ʵ�����е�Լ��Υ��
        opt_cuss    = cell(1,max_run);      %�洢���������ظ�ʵ�����еĿͻ�˳��
        opt_sats    = cell(1,max_run);      %�洢���������ظ�ʵ�����е�����˳��
        opt_caps    = cell(1,max_run);      %�洢���������ظ�ʵ�����е�������Ϣ
        vrps_covs  = cell(1,max_run);
        vltc_covs  = cell(1,max_run);
        cpu_time    = 0;
        %----�㷨ѭ����----
        parfor nrun = 1:max_run
            fprintf('EHG %s\n',case_name);
            penalty2    = 1;                     %�ڶ���ͷ�ֵ
            penalty1    = 1;                     %��һ��ͷ�ֵ
            tic;     %��ʱ��ʼ
            gen                   = 0;
            [seq_cuss,vrp2,vlt2]  = initcus(vrp2e,num_pop);
            [seq_sats,cap_sats,vrp1,vlt1] = initsat(vrp2e,seq_cuss);
            %----�洢ԭʼ���Ž�������Ⱥ����----
            str_sat  = seq_sats;
            str_cap  = cap_sats;
            str_cus  = seq_cuss;
            str_vrp1 = vrp1;
            str_vlt1 = vlt1;
            str_vrp2 = vrp2;
            str_vlt2 = vlt2;
            CV=vlt1+vlt2;
            obj=vrp1+vrp2;
            Feasible = find(CV<=0);
            if isempty(Feasible)
                Best = [];
            else
                [~,Best] = min(obj(Feasible));
            end
            opt_vrp_pre = obj(Feasible(Best));
            opt_vlt_pre = CV(Feasible(Best));
            opt_cus_pre =seq_cuss(Feasible(Best),:);
            opt_sat_pre = seq_sats(Feasible(Best),:);
            opt_cap_pre = cap_sats(Feasible(Best),:);

            %----������ʼ----
            coverge = [];
            vlt_converge = [];
            while gen < max_gen
                penalty1  = min(penalty1*(1+0.4*rand),1000);       %��һ��ͷ�ֵ
                penalty2  = min(penalty2*(1+0.4*rand),1000);       %�ڶ���ͷ�ֵ
                %----�����Ż���ʼ----
                for i = 1:num_pop
                 %% ----�Ż��ڶ���·��----
                    nr2           = 0;
                    while nr2 <= nrun2
                        sm = ceil(rand*4);
                        seq_cust = evosecond(vrp2e,seq_cuss(i,:),sm,opt_cus_pre);   %��ʱ�洢����
                        [vrpt2,vltt2] = msecond(vrp2e,seq_cust);             %��ʱ�洢vrpֵ
                        [seq_satt,cap_satt,vrpt1,vltt1] = initsat(vrp2e,seq_cust);
                        if ((vltt2==0)&&(vlt2(i)==0)) %||(rand<Pf)%if both is feasible or pf<0.55, compare their objecitves
                            if vrpt2+vrpt1 < vrp2(i)+vrp1(i) && vltt2==0 && vltt1==0% new solution with better objective  =�Ų��ܼ���Ϊ������Ҳ�����
                                seq_cuss(i, :) = [seq_cust, zeros(1, numel(seq_cuss(i, :)) - numel(seq_cust))];
                                seq_sats(i,:) = seq_satt;
                                cap_sats(i,:) = cap_satt;
                                vrp1(i) = vrpt1;
                                vlt1(i) = vltt1;
                                vrp2(i)     = vrpt2;
                                vlt2(i)     = vltt2;
                                nr2         = 0;
                            else
                                nr2 = nr2+1;
                            end
                        else
                            nr2 = nr2+1;
                        end
                        gen = gen+1;
                    end
                  %% ----��ʼ����һ��·�����Ż�----
                    [seq_sats(i,:),cap_sats(i,:),vrp1(i),vlt1(i)] = initsat(vrp2e,seq_cuss(i,:));
                    nr1                 = 0;
                    while nr1 <= nrun1
                        %��3��ģʽ��ѡ��һ�ֽ�����ģʽ
                        sm  = ceil(rand*2); 
                        seq_sat_pre = seq_sats(i,:);
                        [seq_satt,cap_satt] = evofirst(vrp2e,seq_sats(i,:),cap_sats(i,:),sm);
                        if isempty(setdiff(seq_sat_pre,seq_satt))
                            nr1 = nr1+1;
                        else
                            delate_sat = setdiff(seq_sat_pre,seq_satt);
                            delate_sat = delate_sat(delate_sat>size(rad_ds,1));
                            other_sat = seq_satt(seq_satt>size(rad_ds,1));
                            other_sat = unique(other_sat);
                            [vrpt1,vltt1] = mfirst(vrp2e,seq_satt,cap_satt);
                            all_combinations = generateCombinations(delate_sat-size(rad_ds,1), other_sat-size(rad_ds,1), seq_cuss(i,:));
                            cost_cus = zeros(1,size(all_combinations,2));
                            vlt_cus = zeros(1,size(all_combinations,2));
                            for k =1:size(all_combinations,2)
                                [cost_cus(k),vlt_cus(k)]=msecond(vrp2e,all_combinations{k});
                            end
                            [min_cost, min_idx] = min(cost_cus(vlt_cus<=0));
                            if isempty(min_idx)
                                nr1 = nr1+1;
                            else
                                best_combination = all_combinations{min_idx};
                               if ((vltt1==0)&&(vlt1(i)+vlt2(i)==0))
                                    if vrpt1+min_cost<vrp1(i)+vrp2(i)
                                        seq_cuss(i,:) = best_combination;
                                        seq_sats(i,:) = seq_satt;
                                        cap_sats(i,:) = cap_satt;
                                        vrp1(i) = vrpt1;
                                        vlt1(i) = vltt1;
                                        vrp2(i)     = min_cost;%
                                        vlt2(i)     = vlt_cus(min_idx);
                                        nr1             = 0;
                                    else
                                        nr1 = nr1+1;
                                    end
                                else 
                                    nr1 = nr1+1;
                               end
                            end
                        end
                        gen = gen+1;
                    end
                end
                
                %----��Ⱥ������·������----
                [seq_sats,cap_sats,seq_cuss,vrp1,vrp2,vlt1,vlt2] = select_SR([str_sat;seq_sats],[str_cap;cap_sats],[str_cus;seq_cuss],[str_vrp1,vrp1],[str_vrp2,vrp2],[str_vlt1,vlt1],[str_vlt2,vlt2]);
                %�´洢���Ž�
                CV=vlt1+vlt2;
                obj=vrp1+vrp2;
                Feasible = find(CV<=0);
                if isempty(Feasible)
                    Best = [];
                else
                    [~,Best] = min(obj(Feasible));
                end
                opt_vrp = obj(Feasible(Best));
                opt_vlt = CV(Feasible(Best));
                opt_cus =seq_cuss(Feasible(Best),:);
                opt_sat = seq_sats(Feasible(Best),:);
                opt_cap = cap_sats(Feasible(Best),:);
                if opt_vrp < opt_vrp_pre
                    opt_vrp_pre = opt_vrp;
                    opt_vlt_pre = opt_vlt;
                    opt_cus_pre = opt_cus;
                    opt_sat_pre = opt_sat;
                    opt_cap_pre = opt_cap;
                end
                coverge = [coverge,opt_vrp_pre];
                vlt_converge = [vlt_converge, opt_vlt];
                fprintf('The best value of 2EVRP Set%d_num%d: %6.2f  %6.2f\n',set,seq,opt_vrp_pre,opt_vlt_pre);
            end
            cpu_time            = cpu_time+toc;
            opt_vrps(nrun)      = opt_vrp_pre;
            opt_vlts(nrun)      = opt_vlt_pre;
            opt_cuss{nrun}      = opt_cus_pre;
            opt_sats{nrun}      = opt_sat_pre;
            opt_caps{nrun}      = opt_cap_pre;
            vrps_covs{nrun}     = coverge;
            vltc_covs{nrun}     = vlt_converge;
%             figroute(coord_dep,coord_sat,coord_cus,opt_sat,opt_cus);
        end
        cpu_time = cpu_time/max_run;
%         fprintf('Average Time Spend:  %6.2f\n\n',cpu_time);
        name = strcat('EHG_','Set',num2str(set),'_','num',num2str(seq));
        result(vrp2e,name,opt_vrps,opt_vlts,opt_cuss,opt_sats,opt_caps,vrps_covs,vltc_covs,cpu_time);
    end
end
