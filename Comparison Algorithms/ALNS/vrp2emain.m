%% -----------------------------������-----------------------------
clear; close; clc; 
%seed = 107; rng(seed,'twister');format bank; %rng(sum(100*clock)); %format long;
rng('shuffle');
%----�������ú���Ⱥ��ʼ��----
max_run     = 20;            %�ظ�ʵ�����
max_gen     = 1000;          %ÿ��ʵ������������500k��5000k
nreg        = 100;           %ÿ�ε����оֲ��Ż�����
pen_0      = 5+rand*9995;   %�����ĳͷ�ֵ   %���׷�Χ[5,10000]
delta       = 1.1;           %�ͷ�ֵ�ĵ���ϵ��
theta       = 0.02;          %�����Ž�ı���ϵ��
seq_dataset = [60,60,60,60]; %5�����Լ��Ĳ�������

%----Start----
for num_folder = 1:4
    seq_data = seq_dataset(num_folder);  %�ļ����ڵĲ�����������%%%%%%%%%%
    for seq = 1:seq_data%%%%%%%%%%%%%%%
       %----��ȡ�ļ���ԭʼ���ݣ���վ�����ǺͿͻ������꣬������Ϣ���ͻ�����,��վ�����Ǽ���룬������ͻ�����룬�ͻ���ͻ�����룬��վ���������߽Ƕȣ�������ͻ����߽Ƕ�
        [vrp2e,coord_dep,coord_sat,coord_cus,fleet,demand,type,dis_ds,dis_sc,rad_ds,rad_sc,neib_ss,neib_cc,neib_sc,num_sat,num_cus] = extractdata(num_folder,seq);
        case_name = vrp2e.case_name;
        %----������ά���йصĲ���----
%         max_gen     = nc*10;
        %----�洢ÿ��ʵ������Ž��----
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
        %----�㷨ѭ����----
        parfor nrun = 1:max_run
            fprintf('ALNS on %s\n',case_name);
            score = ones(1,12);    %��3��ȫ���ƻ���1-3��+5�־ֲ��ƻ���4-8��+4���޸���9-12����12��ģʽ���������ֲ�������
            penal = pen_0;
            tic;     %��ʱ��ʼ
            gen                 = 0;
           %% ----��ʼ��----
            [seq_cus,seq_sat,cap_sat] = initial(vrp2e);
            [fe_cost2,fc_cost2,fvlt2]     = msecond(vrp2e,seq_cus);
            [fe_cost1,fc_cost1,fvlt1]     = mfirst(vrp2e,seq_sat,cap_sat);
            fvrp1 = fe_cost1+fe_cost2;
            fvrp2 = fc_cost1+fc_cost2;
            fvrp    = fvrp1+fvrp2;
            fvlt    = fvlt1+fvlt2;
            %----���³ͷ�ֵ----
            if fvlt1+fvlt2>0
                penal  = max(min(penal./1.1,10000),5.0);
            else
                penal  = max(min(penal*1.1,10000),5.0);
            end
            sat_offl = ones(1,num_sat);   %��ǰ���Ż�·����Ӧ�����ǿ���״̬,1Ϊ����0Ϊ��
            s_offl   = ones(1,num_sat);   %��ǰ�ر����ǵ�״̬��1Ϊ����0Ϊ��
            s_offs   = [];                %ɾ���ͻ���Ӧ������
            %----�洢���ŵ�·�����������Ϣ----
            opt_vrp  = fvrp;
            opt_vlt = fvlt;
            opt_cus  = seq_cus;
            opt_sat  = seq_sat;
            opt_cap  = cap_sat;
            %% ----������ʼ----
            ng = 0;             %���۾ֲ������Ĵ���
            coverge = [];
            vlt_converge= [];
            while gen < max_gen
                %----������������----
                gen       = gen+1;
%                 fprintf('gen = %d\n',gen);
              %% ----�ڶ���·���Ż�----
                %----ѡȡ�ƻ�ģʽ----
                if ng == nreg
                    sm1    = roulette(score(1:3),1);                                          %�ƻ���ʽ1����3�������1-3��
                    [seq_cs,seq_c,sat_offl,s_offs] = destroyl(vrp2e,seq_cus,sat_offl,sm1);       %ȫ���ƻ�
                else
                    sm1    = roulette(score(4:8),1)+3;                                     %�ƻ���ʽ2����5�������4-8��
                    s_offl = sat_offl;
                    [seq_cs,seq_c,s_offs,s_offl]   = destroys(vrp2e,seq_cus,s_offl,sm1);           %�ֲ��ƻ�
                end
                %----ѡȡ�޸�ģʽ----
                sm2 = roulette(score(9:12),1);         %----�޸���ʽ����4�������9-12��--
                %----�޸�ʱ��Ҫ�رյ�����
                off = 1:num_sat;
                if sm2 == 3  %��ԭ·���޸���ʽ
                    s_off = [repmat(off(s_offl==0),[numel(seq_c),1]),s_offs'];
                elseif sm1 == 7 && numel(s_offs)>0 %����·��ɾ����ԭ·����������
                    s_off = [off(s_offl==0),s_offs(1)]; 
                else
                    s_off = off(s_offl==0);
                end
                %----�����޸�----
                seq_cs = repair2(vrp2e,seq_cs,seq_c,s_off,sm2);
                sm2 = sm2+8;      %�޸���ʽ��Ӧ����ţ�+�ƻ���8�����
                %----·������----
                [e_cost2,c_cost2,vlt2] = msecond(vrp2e,seq_cs);
              %% ----��һ��·���Ż�----
                [seq_ss,cap_ss,e_cost1,c_cost1,vlt1] = repair1(vrp2e,seq_cs); 
                %----���³ͷ�ֵ----
                if vlt1+vlt2>0
                    penal = max(min(penal./1.1,10000),5.0);
                else
                    penal = max(min(penal*1.1,10000),5.0);
                end

              %% ----�Ż�ѡ��----
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