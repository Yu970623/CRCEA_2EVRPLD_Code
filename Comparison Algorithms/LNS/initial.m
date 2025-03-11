%% --------------------------初始化--------------------------
%初始化方式：
%step1: 对第一层客户随机排序后修复和局部搜索
%step2: 对第二层路径进行修复和局部搜索
function [seq_cus,seq_sat,cap_sat,fvrp1,fvrp2,fvlt] = initial(vrp2e,s_off)
    num_cus = vrp2e.num_cus;
    num_sat = vrp2e.num_sat;
    
    seq_cus = [];  %客户访问顺序
    seq_c   = num_sat+(1:num_cus); %客户访问顺序
     %----第二层修复和局部搜索----
     seq_cus = repair2(vrp2e,seq_cus,seq_c,s_off);
     %----第一层修复和局部搜索----
     [seq_sat,cap_sat] = repair1(vrp2e,seq_cus);
     %----新解的适应值度量----
     [e_cost2,c_cost2,fvlt2] = msecond(vrp2e,seq_cus);
     [e_cost1,c_cost1,fvlt1] = mfirst(vrp2e,seq_sat,cap_sat);
     fvrp1 = e_cost1+e_cost2;
     fvrp2 = c_cost1+c_cost2;
     fvlt = fvlt1+fvlt2;
end