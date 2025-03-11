%% -----------------------------主程序-----------------------------
%% 种群1采取原始的惩罚函数,种群2忽视约束（penalty=0）。而且在select中对[Population1 Off1 Off2]  [Population2 Off1 Off2]
clear;
clc;
clear all; close all;
rng('shuffle');
%----参数设置和种群初始化----
num_pop     = 10;        
max_run     = 20;         
max_gen     = 1000;      
seq_dataset = [60,60,60,60];

%----Start----
for set = 1:4
    seq_data = seq_dataset(set);  %文件夹内的测试样本总数
    for seq = 1:seq_data
        %----提取文件的原始数据：总站、卫星和客户的坐标，车辆信息，客户需求,总站与卫星间距离，卫星与客户间距离，客户与客户间距离，总站与卫星连线角度，卫星与客户连线角度
        [vrp2e,coord_dep,coord_sat,coord_cus,fleet,demand,type,dis_ds,dis_sc,rad_ds,rad_sc,neib_ss,neib_cc,neib_sc,num_sat,num_cus] = extractdata(set,seq);
        
        %----设置与维数有关的参数----
        nrun1       = min(2*num_sat,5);    %每次迭代中第一层最多优化次数
        nrun2       = min(2*num_cus,5);    %每次迭代中第二层最多优化次数
        %----存储每次实验的最优结果----opt_cap
        num_sc         = num_cus+2*fleet(2,2);  %客户数+无人机*2
        num_ds         = 2*num_sat+2*fleet(1,2);%卫星*2+车辆*2
        len_sat = 2*(fleet(1,2)+num_sat);
        opt_vrps    = zeros(1,max_run);%最优目标值
        opt_vlts    = zeros(1,max_run);%最优约束违反值
        opt_cuss    = cell(1,max_run);
        opt_sats    = cell(1,max_run);
        opt_caps    = cell(1,max_run);
        vrps_covs   = cell(1,max_run);
        vltc_covs   = cell(1,max_run);
        opt_vrps_pop2    = zeros(1,max_run);%最优目标值
        opt_vlts_pop2    = zeros(1,max_run);%最优约束违反值
        opt_cuss_pop2    = cell(1,max_run);
        opt_sats_pop2    = cell(1,max_run);
        opt_caps_pop2    = cell(1,max_run);
        vrps_covs_pop2   = cell(1,max_run);
        vltc_covs_pop2   = cell(1,max_run);
        cpu_time  = 0;
        epsilon0=1e30;
        cp=(-log(epsilon0)-6)/log(1-0.5);
        alpha=0.6;
        %----算法循环体----
        parfor nrun = 1:max_run   %独立运行
            score_pop2       = ones(1,4);
            penalty2    = 1;                     %第二层惩罚值
            penalty1    = 1;                     %第一层惩罚值
            penalty2_pop2    = 1;                %Population2 忽视约束 第二层惩罚值%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            penalty1_pop2    = 1;                %Population2 忽视约束 第一层惩罚值%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            tic;     %计时开始
            gen                   = 0;
            [seq_cuss,vrp2,vlt2]  = initcus(vrp2e,num_pop);%初始化第二层
            [seq_sats,cap_sats,vrp1,vlt1] = initsat(vrp2e,seq_cuss);%初始化第一层
            [seq_cuss_pop2,vrp2_pop2,vlt2_pop2]  = initcus(vrp2e,num_pop);%初始化Population2第二层
            [seq_sats_pop2,cap_sats_pop2,vrp1_pop2,vlt1_pop2] = initsat(vrp2e,seq_cuss_pop2);%初始化Population2第一层

            %----存储最优的路径及其相关信息---- 
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
            fprintf('The best initial solution value of 2EVRP Set%d_num%d: %6.2f  %6.2f\n',set,seq,opt_vrp,opt_vlt);%最优路径cost，和约束违反值
            %----存储原始所有个体，作为精英解保留用于后续种群更新----
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
            %----进化开始----
            while gen < max_gen
                penalty1  = min(penalty1*(1+0.4*rand),1000);       %第一层惩罚值  each generation初始penalty为1-2之间的一个值，在每一代的进化each echelon时不断增大
                penalty2  = min(penalty2*(1+0.4*rand),1000);       %第二层惩罚值
                penalty1_pop2  = min(penalty1_pop2*(1+0.4*rand),1000); 
                penalty2_pop2  = min(penalty2_pop2*(1+0.4*rand),1000); 
                %%epsilon change
                if gen/max_gen<=alpha  %保持最大约束容忍阶段
                    epsilon = epsilon0;
                else
                    epsilon = epsilon0.*(1-(gen-max_gen*alpha)/(max_gen-max_gen*alpha)).^cp;
                end
                %----迭代优化开始----
                for i = 1:num_pop %对于种群中每一个解（产生新种群）
                    
                 %% ----优化第二层路径----
                    nr2           = 0;
                    pen2          = penalty2;
                    sm_adjust    = 3;%只使用后面两个算子
                    seq_cuss(i,:)      = adjustsat(vrp2e,seq_cuss(i,:),seq_sats(i,:),cap_sats(i,:),sm_adjust);%调整satellite
                    [vrp2(i) ,vlt2(i)] = msecond(vrp2e,seq_cuss(i,:));%计算第二层的vrp值
                    while nr2 <= nrun2 %第二层运行代数小于阈值
                        pen2           = pen2*(1+0.4*rand);%惩罚值是一直以指数形式增大 1.0-1.4 ？
                        %从4中模式中选择一种进化的模式
                        if mod(gen,50)==0
                            sm1 = ceil(rand*3);  %阶段性执行3种全局搜索
                            sm2 = 0;
                        else
                            sm2 = ceil(rand*5); %5种局部搜索
                            sm1 = 0;
                        end
                        sm3 = ceil(rand*4); %3种修复
                        epsilonpop1 = 0;
                        [seq_cust,sat_offl] = evosecond(vrp2e,seq_cuss(i,:),sm1,sm2,sm3,sat_offl,Elite_cus_pop2(i,:),epsilonpop1);
                        
                        [vrpt2,vltt2]   = msecond(vrp2e,seq_cust);             %计算法出新路径（解）的目标函数值和约束违反 t2
                        [seq_satt,cap_satt,vrpt1,vltt1] = initsat(vrp2e,seq_cust);
                        if (vrpt2+vrpt1)*((vltt1+vltt2)*pen2+1) < (vrp2(i)+vrp1(i))*((vlt1(i)+vlt2(i))*pen2+1)
                            seq_cuss(i,:) = [seq_cust, zeros(1, numel(seq_cuss(i, :)) - numel(seq_cust))];
                            seq_sats(i,:) = seq_satt;
                            cap_sats(i,:) = cap_satt;
                            vrp1(i) = vrpt1;
                            vlt1(i) = vltt1;
                            vrp2(i)     = vrpt2;%
                            vlt2(i)     = vltt2;
                            nr2         = 0;%清空代数计数器
                        else
                            nr2 = nr2+1;
                        end
                        gen       = gen+1;
                    end
                  %% ----初始化第一层路径并优化----
                  [seq_sats(i,:),cap_sats(i,:),vrp1(i),vlt1(i)] = initsat(vrp2e,seq_cuss(i,:));%初始化satellite和depot间的路径，输出为satellite路径和satellite容，目标值1，约束违反1
%                   sm_adjust    = randi([1, 2]);%只使用前面两个算子
%                   seq_cuss(i,:) = adjustsat(vrp2e,seq_cuss(i,:),seq_sats(i,:),cap_sats(i,:),sm_adjust);%调整satellite
%                   [vrp2(i) ,vlt2(i)] = msecond(vrp2e,seq_cuss(i,:));%计算第二层的vrp值
%                   [seq_sats(i,:),cap_sats(i,:),vrp1(i),vlt1(i)] = initsat(vrp2e,seq_cuss(i,:));
                    nr1                 = 0;
                    pen1                = penalty1;
                    while nr1 <= nrun1 
                        pen1                = pen1*(1+0.4*rand);%惩罚值是一直以指数形式增大每次乘1.0-1.4 ？
                        %从2中模式中选择一种进化的模式
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
                %----Population2进化开始----
                for i = 1:num_pop %对于种群中每一个解（产生新种群）
                    %----进化代数更新----
                    
                 %% ----优化第二层路径----
                    nr2_pop2           = 0;
                    pen2_pop2          = penalty2_pop2;
                    sm_adjust_pop2     = 3;%只使用后面两个算子
                    seq_cuss_pop2(i,:) = adjustsat(vrp2e,seq_cuss_pop2(i,:),seq_sats_pop2(i,:),cap_sats_pop2(i,:),sm_adjust_pop2);%调整satellite
                    [vrp2_pop2(i) ,vlt2_pop2(i)] = msecond(vrp2e,seq_cuss_pop2(i,:));%计算第二层的vrp值
                    while nr2_pop2 <= nrun2 %第二层运行代数小于阈值
                        pen2_pop2 = pen2_pop2*(1+0.4*rand);%惩罚值是一直以指数形式增大 1.0-1.4 ？
                        %从4中模式中选择一种进化的模式
                        if mod(gen,50)==0
                            sm1 = ceil(rand*3);  %阶段性执行3种全局搜索
                            sm2 = 0;
                        else
                            sm2 = ceil(rand*5); %5种局部搜索
                            sm1 = 0;
                        end
                        sm3 = ceil(rand*4); %3种修复
                        [seq_cust_pop2,sat_offl_pop2] = evosecond(vrp2e,seq_cuss(i,:),sm1,sm2,sm3,sat_offl_pop2,Elite_cus(i,:),epsilon);
                        [vrpt2_pop2,vltt2_pop2] = msecond(vrp2e,seq_cust_pop2);             %计算出新路径（解）的目标函数值和约束违反 t2
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
                        if (vrpt1_pop2+vrpt2_pop2)*(1+pen2_pop2*(cvt1_pop2+cvt2_pop2)) < (vrp1_pop2(i)+vrp2_pop2(i))*(1+pen2_pop2*(cv1_pop2(i)+cv2_pop2(i)))   %若新解更优，用新解替换当前解i，且清空进化次数计数器
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
                  %% ----初始化第一层路径并优化----
                    [seq_sats_pop2(i,:),cap_sats_pop2(i,:),vrp1_pop2(i),vlt1_pop2(i)] = initsat(vrp2e,seq_cuss(i,:));%初始化satellite和depot间的路径，输出为satellite路径和satellite容，目标值1，约束违反1                    
                    nr1_pop2                 = 0;
                    pen1_pop2                = penalty1_pop2;
                    while nr1_pop2 <= nrun1 
                        pen1_pop2                = pen1_pop2*(1+0.4*rand); %惩罚值是一直以指数形式增大每次乘1.0-1.4 ？
                        %从2中模式中选择一种进化的模式
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
                            
                            if vltt1_pop2<=epsilon && vlt1_pop2(i)<=epsilon  %新老1级约束容忍
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
                    %针对每一个个体
                end
                %% 主种群择优
                %对初始化种群和新种群的合并种群 进行选择
                [seq_sats,cap_sats,seq_cuss,vrp1,vrp2,vlt1,vlt2] = select([Elite_sat;seq_sats;seq_sats_pop2],[Elite_cap;cap_sats;cap_sats_pop2],[Elite_cus;seq_cuss;seq_cuss_pop2],[Elite_vrp1,vrp1,vrp1_pop2],[Elite_vrp2,vrp2,vrp2_pop2],[Elite_vlt1,vlt1,vlt1_pop2],[Elite_vlt2,vlt2,vlt2_pop2],num_pop);
                if (vrp1(1)+vrp2(1))*((vlt1(1)+vlt2(1))*10000+1)<opt_vrp+opt_vlt*10000 %当前最好解<历史最优解
                    opt_vrp = vrp1(1)+vrp2(1);%序号为1的解是最优的，select中进行了排序，考虑了约束惩罚
                    opt_vlt = vlt1(1)+vlt2(1);
                    opt_cus = seq_cuss(1,:);
                    opt_sat = seq_sats(1,:);
                    opt_cap = cap_sats(1,:);
%                     score(sm_adjust) = score(sm_adjust)+1;
                end
                
                %% 辅助种群择优
                [seq_sats_pop2,cap_sats_pop2,seq_cuss_pop2,vrp1_pop2,vrp2_pop2,vlt1_pop2,vlt2_pop2] = select2([Elite_sat_pop2;seq_sats_pop2;seq_sats],[Elite_cap_pop2;cap_sats_pop2;cap_sats],[Elite_cus_pop2;seq_cuss_pop2;seq_cuss],[Elite_vrp1_pop2,vrp1_pop2,vrp1],[Elite_vrp2_pop2,vrp2_pop2,vrp2],[Elite_vlt1_pop2,vlt1_pop2,vlt1],[Elite_vlt2_pop2,vlt2_pop2,vlt2],num_pop,epsilon);       
                if gen/max_gen<=alpha%只比较目标函数
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
                        opt_vrp_pop2 = vrp1_pop2(1)+vrp2_pop2(1);%序号为1的解是最优的，select中进行了排序，考虑了约束惩罚
                        opt_vlt_pop2 = vlt1_pop2(1)+vlt2_pop2(1);
                        opt_cus_pop2 = seq_cuss_pop2(1,:);
                        opt_sat_pop2 = seq_sats_pop2(1,:);
                        opt_cap_pop2 = cap_sats_pop2(1,:);%capacity
%                         score_pop2(sm_adjust_pop2) = score_pop2(sm_adjust_pop2)+1;
                    end
                end
                %----存储原始解用于种群更新----
                Elite_sat  = seq_sats;%对合并种群选择的 精英种群 进行存储
                Elite_cap  = cap_sats;
                Elite_cus  = seq_cuss;
                Elite_vrp1 = vrp1;
                Elite_vlt1 = vlt1;
                Elite_vrp2 = vrp2;
                Elite_vlt2 = vlt2;
                %----存储原始解用于种群2更新----
                Elite_sat_pop2  = seq_sats_pop2;%对合并种群选择的 精英种群 进行存储
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
            
            %% 存储结果
            cpu_time             = cpu_time+toc;
            opt_vrps(nrun)       = opt_vrp;%for each run 最优路径代价值
            opt_vlts(nrun)       = opt_vlt;%约束违反
            opt_cuss{nrun} = opt_cus;%最优客户路径
            opt_sats{nrun} = opt_sat;%最优卫星路径
            len                  = numel(opt_cap);%numel：元素数量
            opt_caps{nrun} = opt_cap;
            vrps_covs{nrun} = coverge;
            vltc_covs{nrun} = vlt_converge;
             %存储结果2
            opt_vrps_pop2(nrun)       = opt_vrp_pop2;%for each run 最优路径代价值
            opt_vlts_pop2(nrun)       = opt_vlt_pop2;%约束违反
            opt_cuss_pop2{nrun} = opt_cus_pop2;%最优客户路径
            opt_sats_pop2{nrun} = opt_sat_pop2;%最优卫星路径
            len_pop2            = numel(opt_cap_pop2);
            opt_caps_pop2{nrun} = opt_cap_pop2;
            vrps_covs_pop2{nrun} = coverge2;
            vltc_covs_pop2{nrun} = vlt_converge2;
%              figroute(coord_dep,coord_sat,coord_cus,opt_sat,opt_cus);
%              figroute(coord_dep,coord_sat,coord_cus,opt_sat_pop2,opt_cus_pop2);
        end
        cpu_time = cpu_time/max_run/5;%多次运行的平均时间
        fprintf('Average Time Spend:  %6.2f\n\n',cpu_time);
        name = strcat('CRCEA_','Set',num2str(set),'_','num',num2str(seq));
        result(vrp2e,name,opt_vrps,opt_vlts,opt_cuss,opt_sats,opt_caps,vrps_covs,vltc_covs,cpu_time);
        name_pop2 = strcat('CRCEA__P2_','Set',num2str(set),'_','num',num2str(seq),'_pop2');
        result(vrp2e,name_pop2,opt_vrps_pop2,opt_vlts_pop2,opt_cuss_pop2,opt_sats_pop2,opt_caps_pop2,vrps_covs_pop2,vltc_covs_pop2,cpu_time);
    end
end
% clear;