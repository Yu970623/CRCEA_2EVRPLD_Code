algorithm_name = {'LNS','ALNS','EHG','TPEA'};
Data = cell(4,60);
for i = 1:60
    for j = 1:4
        filename = ['F:\2E_Data\',algorithm_name{j},'\vrp_data\',algorithm_name{j},'_Set1_num',num2str(i),'.mat'];
        load(filename)
        Data{j,i} = opt_vrps(1:20);
    end
end
results = zeros(20,4);
for i = 1:4
    for j = 8
        results(:,i) = Data{i,j}';
    end
end
Names = {'LNS','ALNS','EHG','CRCEA'};
boxplot(results,Names)
% ylim([0,0.15]);
