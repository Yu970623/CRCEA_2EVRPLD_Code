%% -----------------------------主程序-----------------------------
clear; close; clc;  
%seed = 107; rng(seed,'twister');format bank; %rng(sum(100*clock)); %format long;%randn('state',sum(100*clock)); rand('state',sum(100*clock)); format bank; %format long; %
rng('shuffle');
%----参数设置和种群初始化----
max_run     = 20;             %重复实验次数
max_gen     = 1000;           %每次实验最多迭代次数500k，5000k
gmax        = 100;            %全局破坏间隔
imax        = 100;            %局部破坏间隔
% score       = ones(1,12);    %含3中全局破坏（1-3）+5种局部破坏（4-8）+4种修复（9-12）共12种模式（不包含局部搜索）

probability = [0.35,0.39,0.32,0.31,0.29,0.33,0.26;
    0.18,0.20,0.12,0.07,0.80,0.52,0.78;
    0.09,0.07,0.14,0.16,0.14,0.21,0.19;
    0.33,0.73,0.19,0.09,0.32,0.21,0.41;
    0.06,0.01,0.29,0.24,0.14,0.28,0.26];
seq_dataset = [60,60,60,60]; %4个测试集的测试样例
%----Start----
for num_folder = 1:4 
    seq_data = seq_dataset(num_folder);  %文件夹内的测试样本总数
    ps = probability(:,num_folder);
    for seq = 1:seq_data%%%%%%%%%%%%%%
         %----提取文件的原始数据：总站、卫星和客户的坐标，车辆信息，客户需求,需求属性，总站与卫星间距离，卫星与客户间距离，客户与客户间距离，总站与卫星连线角度，卫星与客户连线角度
        [vrp2e,coord_dep,coord_sat,coord_cus,fleet,demand,type,dis_ds,dis_sc,rad_ds,rad_sc,neib_ss,neib_cc,neib_sc,num_sat,num_cus] = extractdata(num_folder,seq);
        case_name = vrp2e.case_name;
        %----设置与维数有关的参数----
%         max_gen     = nc*10;
        %----存储每次实验的最优结果----
        num_sc         = 2*(num_cus+fleet(2,2));  %客户数+无人机*2
        num_ds         = 2*(num_sat+fleet(1,2));%卫星*2+车辆*2
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
            fprintf('LNS-2E %s\n',case_name);
            tic;     %计时开始
            gen      = 0;
            s_off    = [];
            [seq_cus,seq_sat,cap_sat,fvrp1,fvrp2,fvlt] = initial(vrp2e,s_off);
            %----存储最优的路径及其相关信息----
            fvrp = fvrp1+fvrp2;
            opt_vrp             = fvrp;
%             opt_ccost             = fvrp2;
            opt_vlt             = fvlt;
            opt_cus             = seq_cus;
            opt_sat             = seq_sat;
            opt_cap             = cap_sat;
%             fprintf('The best initial solution value of 2EVRP Set%d_num%d: %6.2f\n',num_folder+1,seq,opt_vrp);
%             figroute(coord_dep,coord_sat,coord_cus,opt_sat,opt_cus,max_gen);

            coverge = [];
            vlt_converge= [];
            %----进化开始----
            while gen < max_gen   
                iter      = 0;    %记录局部搜索的次数
                gter      = 0;    %记录全局搜索的次数
              %% ----第二层路径优化----
                while iter<=imax
                    iter = iter+1;
                    %----第二层破坏----
                    [seq_cs,seq_c,s_off,sm] = destroy(vrp2e,seq_cus,ps,s_off,gter,gmax);
                    %----第二层修复、局部搜索和解度量----
                    seq_cs      = repair2(vrp2e,seq_cs,seq_c,s_off);
                    [e_cost2,c_cost2,vlt2] = msecond(vrp2e,seq_cs);
                    %----第一层修复、局部搜索和解度量----
                    [seq_ss,cap_ss] = repair1(vrp2e,seq_cs);
                    [e_cost1,c_cost1,vlt1] = mfirst(vrp2e,seq_ss,cap_ss);
                    fvrp1 = e_cost1+e_cost2;
                    fvrp2 = c_cost1+c_cost2;
                    %----判别是否进行卫星关闭和开放
                    if sm == 1
                        gter = 0;
                    end
                    %----新解更新
                    if fvrp1+fvrp2 < fvrp && vlt1+vlt2<= fvlt
                        seq_cus = seq_cs;
                        seq_sat = seq_ss;
                        cap_sat = cap_ss;
                        fvrp    = fvrp1+fvrp2;
                        fvlt    = vlt1+vlt2;
                        iter    = 0;
%                         fprintf('The value of 2EVRP Set%d_num%d: %6.2f\n',num_folder,seq,fvrp);
                    end
                    gter = gter+1;
%                     fprintf('gen = %d\n',gen);
                end
                gen  = gen+1;
                %----全局最优解更新
                if fvrp < opt_vrp && fvlt<=opt_vlt
                    opt_vrp = fvrp;
                    opt_vlt = fvlt;
                    opt_cus = seq_cus;
                    opt_sat = seq_sat;
                    opt_cap = cap_sat;
               else
                    s_off   = [];
                    [seq_cus,seq_sat,cap_sat,fvrp1,fvrp2,fvlt] = initial(vrp2e,s_off);
                end
                fprintf('The best value of 2EVRP Set%d_num%d: %6.2f\n',num_folder,seq,opt_vrp);
                coverge = [coverge,opt_vrp];
                vlt_converge = [vlt_converge, opt_vlt];
            end
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
        fprintf('Average Time Spend:  %6.2f\n\n',cpu_time);
        name = strcat('LNS_','Set',num2str(num_folder),'_','num',num2str(seq));
        result(vrp2e,name,opt_vrps,opt_vlts,opt_cuss,opt_sats,opt_caps,vrps_covs,vltc_covs,cpu_time);
    end
end
% clear;