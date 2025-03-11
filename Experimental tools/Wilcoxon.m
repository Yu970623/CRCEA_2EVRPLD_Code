algorithm_name = {'LNS','ALNS','EHG','CRCEA'};
% algorithm_name = {'CRCEA2','CRCEA3','CRCEA4','CRCEA'};
Data = cell(4,60);
for k = 1:4 
    for i = 4:4:60
        for j = 1:4
            filename = ['F:\2E_Data\',algorithm_name{j},'\vrp_data\',algorithm_name{j},'_Set',num2str(k),'_num',num2str(i),'.mat'];
            load(filename)
            Data{j,(k-1)*15+ceil(i/4)} = opt_vrps(1:20);
        end
    end
end

% 计算每个问题上各算法的均值、标准差及比较结果
results = cell(61, 5);  % 61行，5列（包括表头）

% 表头：问题名称与算法名
results{1, 1} = 'Problem';
results{1, 2} = algorithm_name{1};
results{1, 3} = algorithm_name{2};
results{1, 4} = algorithm_name{3};
results{1, 5} = algorithm_name{4};

epsilon = 1e-2;  % 设置容忍度（可根据需要调整）
% 初始化统计结果
stats = zeros(4, 3);  % 每个算法的统计结果，4行（算法数），3列（领先、弱于、近似）
% 逐问题处理
for i = 1:60
    results{i+1, 1} = ['Instance', num2str(i)];  % 问题名称
    % 计算均值与标准差，并进行Wilcoxon检验
    comp_LNS = compare_with_CRCEA(Data{1,i}, Data{4,i}, epsilon);
    comp_ALNS = compare_with_CRCEA(Data{2,i}, Data{4,i}, epsilon);
    comp_EHG = compare_with_CRCEA(Data{3,i}, Data{4,i}, epsilon);

    % 存储计算结果
    results{i+1, 2} = sprintf('%.4e (%.4e) %s', mean(Data{1,i}), std(Data{1,i}), comp_LNS);
    results{i+1, 3} = sprintf('%.4e (%.4e) %s', mean(Data{2,i}), std(Data{2,i}), comp_ALNS);
    results{i+1, 4} = sprintf('%.4e (%.4e) %s', mean(Data{3,i}), std(Data{3,i}), comp_EHG);
    results{i+1, 5} = sprintf('%.4e (%.4e) ', mean(Data{4,i}), std(Data{4,i}));  % CRCEA与自己比较，始终为"="
    % 统计每个算法与CRCEA的比较结果
    if strcmp(comp_LNS, '+')
        stats(1, 1) = stats(1, 1) + 1;  % LNS领先
    elseif strcmp(comp_LNS, '-')
        stats(1, 2) = stats(1, 2) + 1;  % LNS弱于
    else
        stats(1, 3) = stats(1, 3) + 1;  % LNS近似
    end
    if strcmp(comp_ALNS, '+')
        stats(2, 1) = stats(2, 1) + 1;  % ALNS领先
    elseif strcmp(comp_ALNS, '-')
        stats(2, 2) = stats(2, 2) + 1;  % ALNS弱于
    else
        stats(2, 3) = stats(2, 3) + 1;  % ALNS近似
    end
    if strcmp(comp_EHG, '+')
        stats(3, 1) = stats(3, 1) + 1;  % EHG领先
    elseif strcmp(comp_EHG, '-')
        stats(3, 2) = stats(3, 2) + 1;  % EHG弱于
    else
        stats(3, 3) = stats(3, 3) + 1;  % EHG近似
    end
end
% 添加统计结果行到表格
results{62, 1} = '+,-,=';  % 第一列：'+,-,='
results{62, 2} = sprintf('%d/%d/%d', stats(1, 1), stats(1, 2), stats(1, 3));  % LNS统计结果
results{62, 3} = sprintf('%d/%d/%d', stats(2, 1), stats(2, 2), stats(2, 3));  % ALNS统计结果
results{62, 4} = sprintf('%d/%d/%d', stats(3, 1), stats(3, 2), stats(3, 3));  % EHG统计结果

% 将结果写入Excel文件
xlswrite('Test1.xlsx', results);

disp('Results have been written to algorithm_comparison_results.xlsx');


% 函数：比较每个算法与CRCEA的差异（使用Wilcoxon秩和检验）
function comparison = compare_with_CRCEA(x, y, epsilon)
    % 进行Wilcoxon秩和检验，比较x和y
    [~, h] = signrank(x, y);  % 对比 x 和 y 的配对样本
%     如果p值大于显著性水平（0.05），认为两者差异不显著，返回“=”
    if ~h
        comparison = '=';
    else
        % 如果p值小于显著性水平，进一步检查均值的差异
        mean_x = mean(x);
        mean_y = mean(y);
        % 容忍度：如果两者均值的差异小于epsilon，则认为两者差异不显著
        if abs(mean_x - mean_y) < mean([mean_x,mean_y])*epsilon
            comparison = '='; % 认为它们相等
        elseif mean_x < mean_y
            comparison = '+';  % 如果x的均值大于y，返回"+"
        else
            comparison = '-';  % 如果x的均值小于y，返回"-"
        end
    end
end
