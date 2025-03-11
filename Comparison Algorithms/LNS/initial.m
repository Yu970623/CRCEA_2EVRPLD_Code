%% --------------------------��ʼ��--------------------------
%��ʼ����ʽ��
%step1: �Ե�һ��ͻ����������޸��;ֲ�����
%step2: �Եڶ���·�������޸��;ֲ�����
function [seq_cus,seq_sat,cap_sat,fvrp1,fvrp2,fvlt] = initial(vrp2e,s_off)
    num_cus = vrp2e.num_cus;
    num_sat = vrp2e.num_sat;
    
    seq_cus = [];  %�ͻ�����˳��
    seq_c   = num_sat+(1:num_cus); %�ͻ�����˳��
     %----�ڶ����޸��;ֲ�����----
     seq_cus = repair2(vrp2e,seq_cus,seq_c,s_off);
     %----��һ���޸��;ֲ�����----
     [seq_sat,cap_sat] = repair1(vrp2e,seq_cus);
     %----�½����Ӧֵ����----
     [e_cost2,c_cost2,fvlt2] = msecond(vrp2e,seq_cus);
     [e_cost1,c_cost1,fvlt1] = mfirst(vrp2e,seq_sat,cap_sat);
     fvrp1 = e_cost1+e_cost2;
     fvrp2 = c_cost1+c_cost2;
     fvlt = fvlt1+fvlt2;
end