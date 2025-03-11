%% -----------------------------������-----------------------------
%% ��Ⱥ1��ȡԭʼ�ĳͷ�����,��Ⱥ2����Լ����penalty=0����������select�ж�[Population1 Off1 Off2]  [Population2 Off1 Off2]
clear;
clc;
clear all; close all;
rng('shuffle');
%----�������ú���Ⱥ��ʼ��----
num_pop     = 10;        
max_run     = 20;         
max_gen     = 1000;      
seq_dataset = [60,60,60,60];

%----Start----
for set = 1:4
    seq_data = seq_dataset(set);  %�ļ����ڵĲ�����������
    for seq = 1:seq_data
        %----��ȡ�ļ���ԭʼ���ݣ���վ�����ǺͿͻ������꣬������Ϣ���ͻ�����,��վ�����Ǽ���룬������ͻ�����룬�ͻ���ͻ�����룬��վ���������߽Ƕȣ�������ͻ����߽Ƕ�
        [vrp2e,coord_dep,coord_sat,coord_cus,fleet,demand,type,dis_ds,dis_sc,rad_ds,rad_sc,neib_ss,neib_cc,neib_sc,num_sat,num_cus] = extractdata(set,seq);
        
        %----������ά���йصĲ���----
        nrun1       = min(2*num_sat,5);    %ÿ�ε����е�һ������Ż�����
        nrun2       = min(2*num_cus,5);    %ÿ�ε����еڶ�������Ż�����
        %----�洢ÿ��ʵ������Ž��----opt_cap
        num_sc         = num_cus+2*fleet(2,2);  %�ͻ���+���˻�*2
        num_ds         = 2*num_sat+2*fleet(1,2);%����*2+����*2
        len_sat = 2*(fleet(1,2)+num_sat);
        opt_vrps    = zeros(1,max_run);%����Ŀ��ֵ
        opt_vlts    = zeros(1,max_run);%����Լ��Υ��ֵ
        opt_cuss    = cell(1,max_run);
        opt_sats    = cell(1,max_run);
        opt_caps    = cell(1,max_run);
        vrps_covs   = cell(1,max_run);
        vltc_covs   = cell(1,max_run);
        opt_vrps_pop2    = zeros(1,max_run);%����Ŀ��ֵ
        opt_vlts_pop2    = zeros(1,max_run);%����Լ��Υ��ֵ
        opt_cuss_pop2    = cell(1,max_run);
        opt_sats_pop2    = cell(1,max_run);
        opt_caps_pop2    = cell(1,max_run);
        vrps_covs_pop2   = cell(1,max_run);
        vltc_covs_pop2   = cell(1,max_run);
        cpu_time  = 0;
        epsilon0=1e30;
        cp=(-log(epsilon0)-6)/log(1-0.5);
        alpha=0.6;
        %----�㷨ѭ����----
        parfor nrun = 1:max_run   %��������
            score_pop2       = ones(1,4);
            penalty2    = 1;                     %�ڶ���ͷ�ֵ
            penalty1    = 1;                     %��һ��ͷ�ֵ
            penalty2_pop2    = 1;                %Population2 ����Լ�� �ڶ���ͷ�ֵ%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            penalty1_pop2    = 1;                %Population2 ����Լ�� ��һ��ͷ�ֵ%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            tic;     %��ʱ��ʼ
            gen                   = 0;
            [seq_cuss,vrp2,vlt2]  = initcus(vrp2e,num_pop);%��ʼ���ڶ���
            [seq_sats,cap_sats,vrp1,vlt1] = initsat(vrp2e,seq_cuss);%��ʼ����һ��
            [seq_cuss_pop2,vrp2_pop2,vlt2_pop2]  = initcus(vrp2e,num_pop);%��ʼ��Population2�ڶ���
            [seq_sats_pop2,cap_sats_pop2,vrp1_pop2,vlt1_pop2] = initsat(vrp2e,seq_cuss_pop2);%��ʼ��Population2��һ��

            %----�洢���ŵ�·�����������Ϣ---- 
            [opt_vrp,loc_opt]   = min((vrp1+vrp2).*((vlt1+vlt2)*10000+1));%f = f1+10000*CV
            opt_vlt             = vlt1(loc_opt)+vlt2(loc_opt);%optimal's CV  
            opt_cus             = seq_cuss(loc_opt,:);
            opt_sat             = seq_sats(loc_opt,:);
            opt_cap             = cap_sats(loc_opt,:);
            
            [opt_vrp_pop2,loc_opt_pop2]   = min((vrp1_pop2+vrp2_pop2).*((vlt1_pop2+vlt2_pop2)*0+1));%f = f1+10000*CV
            opt_vlt_pop2             = vlt1_pop2(loc_opt_pop2)+vlt2_pop2(loc_opt_pop2);%optimal's CV  
            opt_cus_pop2             = seq_cuss_pop2(loc_opt_pop2,:);
            opt_sat_pop2             = seq_sats_pop2(loc_opt_pop2,:);
            opt_cap_pop2             = cap_sats_pop2(loc_opt_pop2,:);
            fprintf('The best initial solution value of 2EVRP Set%d_num%d: %6.2f  %6.2f\n',set,seq,opt_vrp,opt_vlt);%����·��cost����Լ��Υ��ֵ
            %----�洢ԭʼ���и��壬��Ϊ��Ӣ�Ᵽ�����ں�����Ⱥ����----
            Elite_sat  = seq_sats;
            Elite_cap  = cap_sats;
            Elite_cus  = seq_cuss;
            Elite_vrp1 = vrp1;
            Elite_vlt1 = vlt1;
            Elite_vrp2 = vrp2;
            Elite_vlt2 = vlt2;
            Elite_sat_pop2  = seq_sats_pop2;
            Elite_cap_pop2  = cap_sats_pop2;
            Elite_cus_pop2  = seq_cuss_pop2;
            Elite_vrp1_pop2 = vrp1_pop2;
            Elite_vlt1_pop2 = vlt1_pop2;
            Elite_vrp2_pop2 = vrp2_pop2;
            Elite_vlt2_pop2 = vlt2_pop2;
            coverge = [];
            vlt_converge = [];
            coverge2 = [];
            vlt_converge2 = [];
            sat_offl = ones(1,num_sat);
            sat_offl_pop2 = ones(1,num_sat);
            %----������ʼ----
            while gen < max_gen
                penalty1  = min(penalty1*(1+0.4*rand),1000);       %��һ��ͷ�ֵ  each generation��ʼpenaltyΪ1-2֮���һ��ֵ����ÿһ���Ľ���each echelonʱ��������
                penalty2  = min(penalty2*(1+0.4*rand),1000);       %�ڶ���ͷ�ֵ
                penalty1_pop2  = min(penalty1_pop2*(1+0.4*rand),1000); 
                penalty2_pop2  = min(penalty2_pop2*(1+0.4*rand),1000); 
                %%epsilon change
                if gen/max_gen<=alpha  %�������Լ�����̽׶�
                    epsilon = epsilon0;
                else
                    epsilon = epsilon0.*(1-(gen-max_gen*alpha)/(max_gen-max_gen*alpha)).^cp;
                end
                %----�����Ż���ʼ----
                for i = 1:num_pop %������Ⱥ��ÿһ���⣨��������Ⱥ��
                    
                 %% ----�Ż��ڶ���·��----
                    nr2           = 0;
                    pen2          = penalty2;
                    sm_adjust    = 3;%ֻʹ�ú�����������
                    seq_cuss(i,:)      = adjustsat(vrp2e,seq_cuss(i,:),seq_sats(i,:),cap_sats(i,:),sm_adjust);%����satellite
                    [vrp2(i) ,vlt2(i)] = msecond(vrp2e,seq_cuss(i,:));%����ڶ����vrpֵ
                    while nr2 <= nrun2 %�ڶ������д���С����ֵ
                        pen2           = pen2*(1+0.4*rand);%�ͷ�ֵ��һֱ��ָ����ʽ���� 1.0-1.4 ��
                        %��4��ģʽ��ѡ��һ�ֽ�����ģʽ
                        if mod(gen,50)==0
                            sm1 = ceil(rand*3);  %�׶���ִ��3��ȫ������
                            sm2 = 0;
                        else
                            sm2 = ceil(rand*5); %5�־ֲ�����
                            sm1 = 0;
                        end
                        sm3 = ceil(rand*4); %3���޸�
                        epsilonpop1 = 0;
                        [seq_cust,sat_offl] = evosecond(vrp2e,seq_cuss(i,:),sm1,sm2,sm3,sat_offl,Elite_cus_pop2(i,:),epsilonpop1);
                        
                        [vrpt2,vltt2]   = msecond(vrp2e,seq_cust);             %���㷨����·�����⣩��Ŀ�꺯��ֵ��Լ��Υ�� t2
                        [seq_satt,cap_satt,vrpt1,vltt1] = initsat(vrp2e,seq_cust);
                        if (vrpt2+vrpt1)*((vltt1+vltt2)*pen2+1) < (vrp2(i)+vrp1(i))*((vlt1(i)+vlt2(i))*pen2+1)
                            seq_cuss(i,:) = [seq_cust, zeros(1, numel(seq_cuss(i, :)) - numel(seq_cust))];
                            seq_sats(i,:) = seq_satt;
                            cap_sats(i,:) = cap_satt;
                            vrp1(i) = vrpt1;
                            vlt1(i) = vltt1;
                            vrp2(i)     = vrpt2;%
                            vlt2(i)     = vltt2;
                            nr2         = 0;%��մ���������
                        else
                            nr2 = nr2+1;
                        end
                        gen       = gen+1;
                    end
                  %% ----��ʼ����һ��·�����Ż�----
                  [seq_sats(i,:),cap_sats(i,:),vrp1(i),vlt1(i)] = initsat(vrp2e,seq_cuss(i,:));%��ʼ��satellite��depot���·�������Ϊsatellite·����satellite�ݣ�Ŀ��ֵ1��Լ��Υ��1
%                   sm_adjust    = randi([1, 2]);%ֻʹ��ǰ����������
%                   seq_cuss(i,:) = adjustsat(vrp2e,seq_cuss(i,:),seq_sats(i,:),cap_sats(i,:),sm_adjust);%����satellite
%                   [vrp2(i) ,vlt2(i)] = msecond(vrp2e,seq_cuss(i,:));%����ڶ����vrpֵ
%                   [seq_sats(i,:),cap_sats(i,:),vrp1(i),vlt1(i)] = initsat(vrp2e,seq_cuss(i,:));
                    nr1                 = 0;
                    pen1                = penalty1;
                    while nr1 <= nrun1 
                        pen1                = pen1*(1+0.4*rand);%�ͷ�ֵ��һֱ��ָ����ʽ����ÿ�γ�1.0-1.4 ��
                        %��2��ģʽ��ѡ��һ�ֽ�����ģʽ
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
                            cost_sat = zeros(1,size(all_combinations,2));
                            vlt_sat = zeros(1,size(all_combinations,2));
                            seq_s = zeros(size(all_combinations,2),len_sat);
                            cap_s = zeros(size(all_combinations,2),len_sat);
                            for k =1:size(all_combinations,2)
                                [cost_cus(k),vlt_cus(k)]=msecond(vrp2e,all_combinations{k});
                                [seq_s(k,:),cap_s(k,:),cost_sat(k),vlt_sat(k)] = initsat(vrp2e,all_combinations{k});
                            end
                            [min_cost, min_idx] = min(cost_sat + cost_cus + Inf*(vlt_sat + vlt_cus > 0));
                            if isempty(min_idx)
                                nr1 = nr1+1;
                            else
                                best_combination = all_combinations{min_idx};
                                if min_cost < (vrp2(i)+vrp1(i))*((vlt1(i)+vlt2(i))*pen2+1)
                                    seq_cuss(i,:) = best_combination;
                                    seq_sats(i,:) = seq_s(min_idx);
                                    cap_sats(i,:) = cap_s(min_idx);
                                    vrp1(i) = cost_sat(min_idx);
                                    vlt1(i) = vlt_sat(min_idx);
                                    vrp2(i)     = cost_cus(min_idx);%
                                    vlt2(i)     = vlt_cus(min_idx);
                                    nr1         = 0;
                                else
                                    nr1 = nr1+1;
                                end
                            end
                        end
                    end
                end
                %----Population2������ʼ----
                for i = 1:num_pop %������Ⱥ��ÿһ���⣨��������Ⱥ��
                    %----������������----
                    
                 %% ----�Ż��ڶ���·��----
                    nr2_pop2           = 0;
                    pen2_pop2          = penalty2_pop2;
                    sm_adjust_pop2     = 3;%ֻʹ�ú�����������
                    seq_cuss_pop2(i,:) = adjustsat(vrp2e,seq_cuss_pop2(i,:),seq_sats_pop2(i,:),cap_sats_pop2(i,:),sm_adjust_pop2);%����satellite
                    [vrp2_pop2(i) ,vlt2_pop2(i)] = msecond(vrp2e,seq_cuss_pop2(i,:));%����ڶ����vrpֵ
                    while nr2_pop2 <= nrun2 %�ڶ������д���С����ֵ
                        pen2_pop2 = pen2_pop2*(1+0.4*rand);%�ͷ�ֵ��һֱ��ָ����ʽ���� 1.0-1.4 ��
                        %��4��ģʽ��ѡ��һ�ֽ�����ģʽ
                        if mod(gen,50)==0
                            sm1 = ceil(rand*3);  %�׶���ִ��3��ȫ������
                            sm2 = 0;
                        else
                            sm2 = ceil(rand*5); %5�־ֲ�����
                            sm1 = 0;
                        end
                        sm3 = ceil(rand*4); %3���޸�
                        [seq_cust_pop2,sat_offl_pop2] = evosecond(vrp2e,seq_cuss(i,:),sm1,sm2,sm3,sat_offl_pop2,Elite_cus(i,:),epsilon);
                        [vrpt2_pop2,vltt2_pop2] = msecond(vrp2e,seq_cust_pop2);             %�������·�����⣩��Ŀ�꺯��ֵ��Լ��Υ�� t2
                        [seq_satt_pop2,cap_satt_pop2,vrpt1_pop2,vltt1_pop2] = initsat(vrp2e,seq_cust_pop2);
                       %% epsilon 
                       if nr2_pop2/nrun2<=alpha
                           epsilon=epsilon0;
                       else
                           epsilon = epsilon0.*(1-(nr2_pop2-alpha*nrun2)/(nrun2-nrun2*alpha)).^cp;
                       end  
                       cvt2_pop2=vltt2_pop2;
                       cvt1_pop2=vltt1_pop2;
                        if vltt2_pop2<=epsilon && vltt1_pop2<=epsilon
                            cvt2_pop2 = 0; 
                            cvt1_pop2 = 0;
                        end
                        cv2_pop2 = vlt2_pop2;
                        cv1_pop2 = vlt1_pop2;
                        if vlt2_pop2(i)<=epsilon && vlt1_pop2(i)<=epsilon
                            cv2_pop2(i)=0;
                            cv1_pop2(i)=0;
                        end
                        if (vrpt1_pop2+vrpt2_pop2)*(1+pen2_pop2*(cvt1_pop2+cvt2_pop2)) < (vrp1_pop2(i)+vrp2_pop2(i))*(1+pen2_pop2*(cv1_pop2(i)+cv2_pop2(i)))   %���½���ţ����½��滻��ǰ��i������ս�������������
                            seq_cuss_pop2(i,:) = [seq_cust_pop2, zeros(1, numel(seq_cuss_pop2(i, :)) - numel(seq_cust_pop2))];
                            seq_sats_pop2(i,:) = seq_satt_pop2;
                            cap_sats_pop2(i,:) = cap_satt_pop2;
                            vrp1_pop2(i)     = vrpt1_pop2;
                            vlt1_pop2(i)     = vltt1_pop2;
                            vrp2_pop2(i)     = vrpt2_pop2;
                            vlt2_pop2(i)     = vltt2_pop2;
                            nr2_pop2         = 0;
                        else
                            nr2_pop2 = nr2_pop2+1;
                        end
                        gen = gen+1;
                    end
                  %% ----��ʼ����һ��·�����Ż�----
                    [seq_sats_pop2(i,:),cap_sats_pop2(i,:),vrp1_pop2(i),vlt1_pop2(i)] = initsat(vrp2e,seq_cuss(i,:));%��ʼ��satellite��depot���·�������Ϊsatellite·����satellite�ݣ�Ŀ��ֵ1��Լ��Υ��1                    
                    nr1_pop2                 = 0;
                    pen1_pop2                = penalty1_pop2;
                    while nr1_pop2 <= nrun1 
                        pen1_pop2                = pen1_pop2*(1+0.4*rand); %�ͷ�ֵ��һֱ��ָ����ʽ����ÿ�γ�1.0-1.4 ��
                        %��2��ģʽ��ѡ��һ�ֽ�����ģʽ
                        sm  = ceil(rand*2);
                        seq_sat_pre = seq_sats_pop2(i,:);
                        [seq_satt_pop2,cap_satt_pop2] = evofirst(vrp2e,seq_sats_pop2(i,:),cap_sats_pop2(i,:),sm);
                      %% epsilon
                       if nr1_pop2/nrun1<=alpha
                           epsilon=epsilon0;
                       else
                           epsilon = epsilon0.*(1-(nr1_pop2-alpha*nrun1)/(nrun1-alpha*nrun1)).^cp;
                       end
                        if isempty(setdiff(seq_sat_pre,seq_satt_pop2))
                            nr1_pop2 = nr1_pop2+1;
                        else
                            delate_sat = setdiff(seq_sat_pre,seq_satt_pop2);
                            delate_sat = delate_sat(delate_sat>size(rad_ds,1));
                            other_sat = seq_satt_pop2(seq_satt_pop2>size(rad_ds,1));
                            other_sat = unique(other_sat);
                            [vrpt1_pop2,vltt1_pop2] = mfirst(vrp2e,seq_satt_pop2,cap_satt_pop2);
                            all_combinations = generateCombinations(delate_sat-size(rad_ds,1), other_sat-size(rad_ds,1), seq_cuss_pop2(i,:));
                            
                            if vltt1_pop2<=epsilon && vlt1_pop2(i)<=epsilon  %����1��Լ������
                                vltt1_pop22 = 0;
                                vlt1_pop22 = 0;
                            else
                                vltt1_pop22 = vltt1_pop2;
                                vlt1_pop22 = vlt1_pop2(i);
                            end
                            cost_cus = zeros(1,size(all_combinations,2));
                            vlt_cus = zeros(1,size(all_combinations,2));
                            cost_sat = zeros(1,size(all_combinations,2));
                            vlt_sat = zeros(1,size(all_combinations,2));
                            seq_s = zeros(size(all_combinations,2),len_sat);
                            cap_s = zeros(size(all_combinations,2),len_sat);
                            for k =1:size(all_combinations,2)
                                [cost_cus(k),vlt_cus(k)]=msecond(vrp2e,all_combinations{k});
                                [seq_s(k,:),cap_s(k,:),cost_sat(k),vlt_sat(k)] = initsat(vrp2e,all_combinations{k});
                            end
                            [min_cost, min_idx] = min(cost_sat + cost_cus + Inf*(vlt_sat + vlt_cus >= epsilon));
                            if isempty(min_idx)
                                nr1_pop2 = nr1_pop2+1;
                            else
                                best_combination = all_combinations{min_idx};
                                vltt2_pop22=0;
                                if vlt2_pop2(i)<=epsilon
                                    vlt2_pop22=0;
                                else
                                    vlt2_pop22 = vlt2_pop2(i);
                                end
                                if min_cost < (vrp2_pop2(i)+vrp1_pop2(i))*((vlt1_pop22+vlt2_pop22)*pen1_pop2+1)
                                    seq_cuss_pop2(i,:) = best_combination;
                                    seq_sats_pop2(i,:) = seq_s(min_idx);
                                    cap_sats_pop2(i,:) = cap_s(min_idx);
                                    vrp1_pop2(i) = cost_sat(min_idx);
                                    vlt1_pop2(i) = vlt_sat(min_idx);
                                    vrp2_pop2(i)     = cost_cus(min_idx);%
                                    vlt2_pop2(i)     = vlt_cus(min_idx);
                                    nr1_pop2             = 0;
                                else
                                    nr1_pop2 = nr1_pop2+1;
                                end
                            end
                        end
                    end
                    %���ÿһ������
                end
                %% ����Ⱥ����
                %�Գ�ʼ����Ⱥ������Ⱥ�ĺϲ���Ⱥ ����ѡ��
                [seq_sats,cap_sats,seq_cuss,vrp1,vrp2,vlt1,vlt2] = select([Elite_sat;seq_sats;seq_sats_pop2],[Elite_cap;cap_sats;cap_sats_pop2],[Elite_cus;seq_cuss;seq_cuss_pop2],[Elite_vrp1,vrp1,vrp1_pop2],[Elite_vrp2,vrp2,vrp2_pop2],[Elite_vlt1,vlt1,vlt1_pop2],[Elite_vlt2,vlt2,vlt2_pop2],num_pop);
                if (vrp1(1)+vrp2(1))*((vlt1(1)+vlt2(1))*10000+1)<opt_vrp+opt_vlt*10000 %��ǰ��ý�<��ʷ���Ž�
                    opt_vrp = vrp1(1)+vrp2(1);%���Ϊ1�Ľ������ŵģ�select�н��������򣬿�����Լ���ͷ�
                    opt_vlt = vlt1(1)+vlt2(1);
                    opt_cus = seq_cuss(1,:);
                    opt_sat = seq_sats(1,:);
                    opt_cap = cap_sats(1,:);
%                     score(sm_adjust) = score(sm_adjust)+1;
                end
                
                %% ������Ⱥ����
                [seq_sats_pop2,cap_sats_pop2,seq_cuss_pop2,vrp1_pop2,vrp2_pop2,vlt1_pop2,vlt2_pop2] = select2([Elite_sat_pop2;seq_sats_pop2;seq_sats],[Elite_cap_pop2;cap_sats_pop2;cap_sats],[Elite_cus_pop2;seq_cuss_pop2;seq_cuss],[Elite_vrp1_pop2,vrp1_pop2,vrp1],[Elite_vrp2_pop2,vrp2_pop2,vrp2],[Elite_vlt1_pop2,vlt1_pop2,vlt1],[Elite_vlt2_pop2,vlt2_pop2,vlt2],num_pop,epsilon);       
                if gen/max_gen<=alpha%ֻ�Ƚ�Ŀ�꺯��
                    if (vrp1_pop2(1)+vrp2_pop2(1))<opt_vrp_pop2
                        opt_vrp_pop2 = vrp1_pop2(1)+vrp2_pop2(1);
                        opt_vlt_pop2 = vlt1_pop2(1)+vlt2_pop2(1);
                        opt_cus_pop2 = seq_cuss_pop2(1,:);
                        opt_sat_pop2 = seq_sats_pop2(1,:);
                        opt_cap_pop2 = cap_sats_pop2(1,:);
%                         score_pop2(sm_adjust_pop2) = score_pop2(sm_adjust_pop2)+1;
                    end
                else
                    if ((vrp1_pop2(1)+vrp2_pop2(1))<opt_vrp_pop2) || ((vlt1_pop2(1)+vlt2_pop2(1))<opt_vlt_pop2)
                        opt_vrp_pop2 = vrp1_pop2(1)+vrp2_pop2(1);%���Ϊ1�Ľ������ŵģ�select�н��������򣬿�����Լ���ͷ�
                        opt_vlt_pop2 = vlt1_pop2(1)+vlt2_pop2(1);
                        opt_cus_pop2 = seq_cuss_pop2(1,:);
                        opt_sat_pop2 = seq_sats_pop2(1,:);
                        opt_cap_pop2 = cap_sats_pop2(1,:);%capacity
%                         score_pop2(sm_adjust_pop2) = score_pop2(sm_adjust_pop2)+1;
                    end
                end
                %----�洢ԭʼ��������Ⱥ����----
                Elite_sat  = seq_sats;%�Ժϲ���Ⱥѡ��� ��Ӣ��Ⱥ ���д洢
                Elite_cap  = cap_sats;
                Elite_cus  = seq_cuss;
                Elite_vrp1 = vrp1;
                Elite_vlt1 = vlt1;
                Elite_vrp2 = vrp2;
                Elite_vlt2 = vlt2;
                %----�洢ԭʼ��������Ⱥ2����----
                Elite_sat_pop2  = seq_sats_pop2;%�Ժϲ���Ⱥѡ��� ��Ӣ��Ⱥ ���д洢
                Elite_cap_pop2  = cap_sats_pop2;
                Elite_cus_pop2  = seq_cuss_pop2;
                Elite_vrp1_pop2 = vrp1_pop2;
                Elite_vlt1_pop2 = vlt1_pop2;
                Elite_vrp2_pop2 = vrp2_pop2;
                Elite_vlt2_pop2 = vlt2_pop2;
                coverge = [coverge,opt_vrp];
                vlt_converge = [vlt_converge, opt_vlt];
                coverge2 = [coverge2,opt_vrp_pop2];
                vlt_converge2 = [vlt_converge2, opt_vlt_pop2];
                fprintf('The best value of 2EVRP Set%d_num%d: %6.2f  %6.2f\n',set,seq,opt_vrp,opt_vlt);
            end
            
            %% �洢���
            cpu_time             = cpu_time+toc;
            opt_vrps(nrun)       = opt_vrp;%for each run ����·������ֵ
            opt_vlts(nrun)       = opt_vlt;%Լ��Υ��
            opt_cuss{nrun} = opt_cus;%���ſͻ�·��
            opt_sats{nrun} = opt_sat;%��������·��
            len                  = numel(opt_cap);%numel��Ԫ������
            opt_caps{nrun} = opt_cap;
            vrps_covs{nrun} = coverge;
            vltc_covs{nrun} = vlt_converge;
             %�洢���2
            opt_vrps_pop2(nrun)       = opt_vrp_pop2;%for each run ����·������ֵ
            opt_vlts_pop2(nrun)       = opt_vlt_pop2;%Լ��Υ��
            opt_cuss_pop2{nrun} = opt_cus_pop2;%���ſͻ�·��
            opt_sats_pop2{nrun} = opt_sat_pop2;%��������·��
            len_pop2            = numel(opt_cap_pop2);
            opt_caps_pop2{nrun} = opt_cap_pop2;
            vrps_covs_pop2{nrun} = coverge2;
            vltc_covs_pop2{nrun} = vlt_converge2;
%              figroute(coord_dep,coord_sat,coord_cus,opt_sat,opt_cus);
%              figroute(coord_dep,coord_sat,coord_cus,opt_sat_pop2,opt_cus_pop2);
        end
        cpu_time = cpu_time/max_run/5;%������е�ƽ��ʱ��
        fprintf('Average Time Spend:  %6.2f\n\n',cpu_time);
        name = strcat('CRCEA_','Set',num2str(set),'_','num',num2str(seq));
        result(vrp2e,name,opt_vrps,opt_vlts,opt_cuss,opt_sats,opt_caps,vrps_covs,vltc_covs,cpu_time);
        name_pop2 = strcat('CRCEA__P2_','Set',num2str(set),'_','num',num2str(seq),'_pop2');
        result(vrp2e,name_pop2,opt_vrps_pop2,opt_vlts_pop2,opt_cuss_pop2,opt_sats_pop2,opt_caps_pop2,vrps_covs_pop2,vltc_covs_pop2,cpu_time);
    end
end
% clear;