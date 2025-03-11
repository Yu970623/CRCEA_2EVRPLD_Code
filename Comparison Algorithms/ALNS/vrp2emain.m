%% -----------------------------主程序-----------------------------
clear; close; clc; 
%seed = 107; rng(seed,'twister');format bank; %rng(sum(100*clock)); %format long;
rng('shuffle');
%----参数设置和种群初始化----
max_run     = 20;            %重复实验次数
max_gen     = 1000;          %每次实验最多迭代次数500k，5000k
nreg        = 100;           %每次迭代中局部优化次数
pen_0      = 5+rand*9995;   %容量的惩罚值   %文献范围[5,10000]
delta       = 1.1;           %惩罚值的调节系数
theta       = 0.02;          %与最优解的比例系数
seq_dataset = [60,60,60,60]; %5个测试集的测试样例

%----Start----
for num_folder = 1:4
    seq_data = seq_dataset(num_folder);  %文件夹内的测试样本总数%%%%%%%%%%
    for seq = 1:seq_data%%%%%%%%%%%%%%%
       %----提取文件的原始数据：总站、卫星和客户的坐标，车辆信息，客户需求,总站与卫星间距离，卫星与客户间距离，客户与客户间距离，总站与卫星连线角度，卫星与客户连线角度
        [vrp2e,coord_dep,coord_sat,coord_cus,fleet,demand,type,dis_ds,dis_sc,rad_ds,rad_sc,neib_ss,neib_cc,neib_sc,num_sat,num_cus] = extractdata(num_folder,seq);
        case_name = vrp2e.case_name;
        %----设置与维数有关的参数----
%         max_gen     = nc*10;
        %----存储每次实验的最优结果----
        num_sc         = 2*(num_cus+fleet(2,2));
        num_ds         = 2*(fleet(1,2)+num_sat);
        opt_vrps    = zeros(1,max_run);
        opt_vlts    = zeros(1,max_run);
        opt_cuss    = cell(1,max_run);
        opt_sats    = cell(1,max_run);
        opt_caps    = cell(1,max_run);
        vrps_covs  = cell(1,max_run);
        vltc_covs  = cell(1,max_run);
        cpu_time    = 0;
        %----算法循环体----
        parfor nrun = 1:max_run
            fprintf('ALNS on %s\n',case_name);
            score = ones(1,12);    %含3中全局破坏（1-3）+5种局部破坏（4-8）+4种修复（9-12）共12种模式（不包含局部搜索）
            penal = pen_0;
            tic;     %计时开始
            gen                 = 0;
           %% ----初始解----
            [seq_cus,seq_sat,cap_sat] = initial(vrp2e);
            [fe_cost2,fc_cost2,fvlt2]     = msecond(vrp2e,seq_cus);
            [fe_cost1,fc_cost1,fvlt1]     = mfirst(vrp2e,seq_sat,cap_sat);
            fvrp1 = fe_cost1+fe_cost2;
            fvrp2 = fc_cost1+fc_cost2;
            fvrp    = fvrp1+fvrp2;
            fvlt    = fvlt1+fvlt2;
            %----更新惩罚值----
            if fvlt1+fvlt2>0
                penal  = max(min(penal./1.1,10000),5.0);
            else
                penal  = max(min(penal*1.1,10000),5.0);
            end
            sat_offl = ones(1,num_sat);   %当前待优化路径对应的卫星开闭状态,1为开，0为关
            s_offl   = ones(1,num_sat);   %当前关闭卫星的状态：1为开，0为关
            s_offs   = [];                %删除客户对应的卫星
            %----存储最优的路径及其相关信息----
            opt_vrp  = fvrp;
            opt_vlt = fvlt;
            opt_cus  = seq_cus;
            opt_sat  = seq_sat;
            opt_cap  = cap_sat;
            %% ----进化开始----
            ng = 0;             %评价局部搜索的次数
            coverge = [];
            vlt_converge= [];
            while gen < max_gen
                %----进化代数更新----
                gen       = gen+1;
%                 fprintf('gen = %d\n',gen);
              %% ----第二层路径优化----
                %----选取破坏模式----
                if ng == nreg
                    sm1    = roulette(score(1:3),1);                                          %破坏方式1：含3种情况（1-3）
                    [seq_cs,seq_c,sat_offl,s_offs] = destroyl(vrp2e,seq_cus,sat_offl,sm1);       %全局破坏
                else
                    sm1    = roulette(score(4:8),1)+3;                                     %破坏方式2：含5种情况（4-8）
                    s_offl = sat_offl;
                    [seq_cs,seq_c,s_offs,s_offl]   = destroys(vrp2e,seq_cus,s_offl,sm1);           %局部破坏
                end
                %----选取修复模式----
                sm2 = roulette(score(9:12),1);         %----修复方式：含4种情况（9-12）--
                %----修复时需要关闭的卫星
                off = 1:num_sat;
                if sm2 == 3  %非原路径修复方式
                    s_off = [repmat(off(s_offl==0),[numel(seq_c),1]),s_offs'];
                elseif sm1 == 7 && numel(s_offs)>0 %单条路径删除：原路径不可重现
                    s_off = [off(s_offl==0),s_offs(1)]; 
                else
                    s_off = off(s_offl==0);
                end
                %----进行修复----
                seq_cs = repair2(vrp2e,seq_cs,seq_c,s_off,sm2);
                sm2 = sm2+8;      %修复方式对应的序号：+破坏的8种情况
                %----路径度量----
                [e_cost2,c_cost2,vlt2] = msecond(vrp2e,seq_cs);
              %% ----第一层路径优化----
                [seq_ss,cap_ss,e_cost1,c_cost1,vlt1] = repair1(vrp2e,seq_cs); 
                %----更新惩罚值----
                if vlt1+vlt2>0
                    penal = max(min(penal./1.1,10000),5.0);
                else
                    penal = max(min(penal*1.1,10000),5.0);
                end

              %% ----优化选择----
                if ng == nreg
                    seq_cs = local2(vrp2e,seq_cs);
                    [e_cost2,c_cost2,vlt2] = msecond(vrp2e,seq_cs);
                    seq_cus  = seq_cs;
                    seq_sat  = seq_ss;
                    cap_sat  = cap_ss;
                    fvrp1 = e_cost1+e_cost2;
                    fvrp2 = c_cost1+c_cost2;
                    fvlt1 = vlt1;
                    fvlt2 = vlt2;
                    sat_offl = s_offl;
                    ng = 0;
                elseif e_cost1+e_cost2+c_cost1+c_cost2+penal*(vlt1+vlt2) < (1+theta)*(opt_vrp+penal*opt_vlt)
                    seq_cs   = local2(vrp2e,seq_cs);
                    [e_cost2,c_cost2,vlt2] = msecond(vrp2e,seq_cs);
                end
                if e_cost1+e_cost2+c_cost1+c_cost2+penal*(vlt1+vlt2) < fvrp1+fvrp2+penal*(fvlt1+fvlt2)
                    seq_cus  = seq_cs;
                    seq_sat  = seq_ss;
                    cap_sat  = cap_ss;
                    fvrp1 = e_cost1+e_cost2;
                    fvrp2 = c_cost1+c_cost2;
                    fvlt1 = vlt1;
                    fvlt2 = vlt2;
                    sat_offl = s_offl;
                    ng       = 0;
                else
                    ng       = ng+1;
                end
                if fvrp1+fvrp2+penal*(fvlt1+fvlt2)< opt_vrp+penal*opt_vlt
                    opt_vrp    = fvrp1+fvrp2;
                    opt_vlt    = fvlt1+fvlt2;
                    opt_cus    = seq_cus;
                    opt_sat    = seq_sat;
                    opt_cap    = cap_sat;
                    score(sm1) = score(sm1)+1;
                    score(sm2) = score(sm2)+1;
                end
                coverge = [coverge,opt_vrp];
                vlt_converge = [vlt_converge, opt_vlt];
                fprintf('The best value of 2EVRP Set%d_num%d: %6.2f  %6.2f\n',num_folder,seq,opt_vrp,opt_vlt);
            end
%             fprintf('The best value of 2EVRP Set%d_num%d: %6.2f  %6.2f\n',num_folder+1,seq,opt_vrp,opt_vltc+opt_vltv);
            cpu_time             = cpu_time+toc;
            opt_vrps(nrun)       = opt_vrp;
            opt_vlts(nrun)       = opt_vlt;
            opt_cuss{nrun} = opt_cus;
            opt_sats{nrun} = opt_sat;
            opt_caps{nrun} = opt_cap;
            vrps_covs{nrun} = coverge;
            vltc_covs{nrun} = vlt_converge;
%             figroute(coord_dep,coord_sat,coord_cus,opt_sat,opt_cus,max_gen);
        end
        cpu_time = cpu_time/max_run;
%         fprintf('Average Time Spend:  %6.2f\n\n',cpu_time);
        name = strcat('ALNS_','Set',num2str(num_folder),'_','num',num2str(seq));
        result(vrp2e,name,opt_vrps,opt_vlts,opt_cuss,opt_sats,opt_caps,vrps_covs,vltc_covs,cpu_time);
    end
end
% clear;