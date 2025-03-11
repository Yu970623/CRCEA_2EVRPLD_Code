%% ---------------------计算双层路径的函数值---------------------------
function fvrp = metric(seq_cs,seq_ss,cap_ss)
    num_pop = size(seq_ss,1);
    fvrp = zeros(1,num_pop);
    for i = 1:num_pop
        [e_cost1,c_cost1,fvlt1] = mfirst(vrp2e,seq_ss(i,:),cap_ss(i,:));
        [e_cost2,c_cost2,fvlt2] = msecond(vrp2e,seq_cs(i,:));
        fvrp1 = e_cost1+e_cost2;
        fvrp2 = c_cost1+c_cost2;
        fvrp(i)       = fvrp1+fvrp2+fvlt1+fvlt2;
    end
end
%----第一层车辆度量----
function [e_cost1,c_cost1,fvlt1] = mfirst(vrp2e,seq_s,cap_s)
    global dis_ds; global dis_ss; global fleet;
    pen_c = 10000;
    pen_v = 10000;
    nds   = numel(seq_s);
    fvrp1 = 0;
    fvlt1 = 0;
    nfs   = 0;      %统计使用的车辆数目
    cap_v = 0;      %统计车辆的装载容量
    for i = 2:nds
        if seq_s(i)>0
            cap_v        = cap_v+cap_s(i);
            if seq_s(i-1)==0
                fvrp1 = fvrp1+dis_ds(seq_s(i));
            else
                fvrp1 = fvrp1+dis_ss(seq_s(i-1),seq_s(i));
                nfs      = nfs+1;
            end
        else
            if seq_s(i-1)>0
                fvrp1   = fvrp1+dis_ds(seq_s(i-1));
                fvlt1  = fvlt1+pen_c*max(0,cap_v-fleet(1,1));
                cap_v       = 0;
            end
        end  
    end
    fvlt1 = fvlt1+pen_v*(max(0,nfs-fleet(1,2)));
end
%----第二层车辆度量----
function [e_cost2,c_cost2,fvlt2] = msecond(vrp2e,seq_c)
    global dis_sc; global dis_cc; global demand; global fleet; 
    pen_c = 10000;
    pen_v = 10000;
    nsc   = numel(seq_c);
    fvrp2 = 0;  
    fvlt2 = 0;
    nfs   = 0;    %统计使用的车辆数目
    cap_v = 0;    %统计车辆的装载容量
    s     =  -seq_c(1);
    for i = 2:nsc
        if seq_c(i)>0
            cap_v     = cap_v+demand(seq_c(i));
            if seq_c(i-1) > 0
                fvrp2 = fvrp2+dis_cc(seq_c(i-1),seq_c(i));
            else
                fvrp2 = fvrp2+dis_sc(s,seq_c(i));
                nfs   = nfs+1;
            end
        else
            if seq_c(i-1)>0
                fvrp2 = fvrp2+dis_sc(s,seq_c(i-1));
                fvlt2 = fvlt2+pen_c*(max(0,cap_v-fleet(2,1)));
                cap_v = 0;
            end
            s = -seq_c(i);
        end
    end
    if seq_c(nsc) > 0
        fvrp2 = fvrp2+dis_sc(s,seq_c(nsc));
        fvlt2 = fvlt2+pen_c*(max(0,cap_v-fleet(2,1)));
    end
    fvlt2     = fvlt2+pen_v*(max(0,nfs-fleet(2,2)));
end