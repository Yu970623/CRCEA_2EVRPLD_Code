% 给定的维度
num_problems = 15;  % 问题数量
num_algorithms = 5;  % 算法数量
num_runs = 12;  % 每个算法每个问题的运行次数
algorithm_name = {'LNS','ALNS','EHG','CRCEA'};
algorithm_name2 = {'CRCEA1','CRCEA2','CRCEA3','CRCEA4','CRCEA'};
algorithm_name1 = {'CRCEAE2','CRCEAV2','CRCEAW2','CRCEAWEV2','CRCEA'};
% 初始化按问题存储的 IGD_data 矩阵
IGD_data = zeros(num_runs, num_algorithms, num_problems);
Data = cell(5,15);
% for k = 1:4 
%     for i = 4:4:60
%         for j = 1:4
%             filename = ['F:\2E_Data\',algorithm_name{j},'\vrp_data\',algorithm_name{j},'_Set',num2str(k),'_num',num2str(i),'.mat'];
%             load(filename)
%             Data{j,(k-1)*15+ceil(i/4)} = opt_vrps(1:12);
%         end
%     end
% end

for k = 1
    for i = 4:4:60
        for j = 1:5
            filename = ['F:\2E_Data\',algorithm_name1{j},'\vrp_data\CRCEA','_Set',num2str(k),'_num',num2str(i),'.mat'];
            load(filename)
            Data{j,(k-1)*15+ceil(i/4)} = opt_vrps(1:12);
        end
    end
end

results = zeros(12,5);
Fred = zeros(1200,5);
% 循环读取每个问题的文件
for problem = 1:num_problems
    results(:,1) = Data{1,problem}';
    results(:,2) = Data{2,problem}';
    results(:,3) = Data{3,problem}';
    results(:,4) = Data{4,problem}';
    results(:,5) = Data{5,problem}';
    IGD_data(:, :, problem) = results;
    Fred((problem-1)*12 + 1:problem*12, :) = results;
end
% writematrix(Fred, 'D:\OneDrive-CSU\OneDrive - csu.edu.cn\2E-VRP\Experiment\drawNemenyi-master\Data\variant100.xlsx');
% 计算平均 IGD 和标准误差
mean_IGD_data = mean(IGD_data, 1);  % 计算每个算法在所有问题上的平均 IGD，结果为 1 x num_algorithms x num_problems
std_error_data = std(IGD_data, 0, 1) / sqrt(num_runs);  % 计算每个算法在所有问题上的标准误差

% 算法颜色设置
colors = lines(num_algorithms);  % MATLAB 默认颜色序列

% 绘图
figure;
hold on;

% 循环绘制每个算法的平均 IGD 和置信区间
for alg = 1:num_algorithms
    % 绘制平均 IGD
    h_line = plot(1:num_problems, squeeze(mean_IGD_data(1, alg, :)), '-o', 'Color', colors(alg, :), 'MarkerSize', 5);
    
    % 使用 t 分布计算置信区间
    t_critical = tinv(0.9, num_runs - 1);  % 90% 置信水平
    error_range = squeeze(t_critical * std_error_data(1, alg, :));  % 置信区间
    
    % 绘制置信区间
    for i = 1:num_problems
        % 上下界
        line([i, i], [squeeze(mean_IGD_data(1, alg, i)) - error_range(i), squeeze(mean_IGD_data(1, alg, i)) + error_range(i)], ...
            'Color', colors(alg, :), 'LineWidth', 2);
        % 横线
        plot([i-0.1, i+0.1], [squeeze(mean_IGD_data(1, alg, i)) - error_range(i), squeeze(mean_IGD_data(1, alg, i)) - error_range(i)], ...
            'Color', colors(alg, :), 'LineWidth', 2);
        plot([i-0.1, i+0.1], [squeeze(mean_IGD_data(1, alg, i)) + error_range(i), squeeze(mean_IGD_data(1, alg, i)) + error_range(i)], ...
            'Color', colors(alg, :), 'LineWidth', 2);
    end
end
labels15 = {'Ca1-2,3,15','Ca1-3,5,15','Ca1-6,4,15','Ca2-2,3,15','Ca2-3,5,15','Ca2-6,4,15','Ca3-2,3,15',...
        'Ca3-3,5,15','Ca3-6,4,15','Ca4-2,3,15','Ca4-3,5,15','Ca4-6,4,15','Ca5-2,3,15','Ca5-3,5,15','Ca5-6,4,15', ...
        'Cb1-2,3,15','Cb1-3,5,15','Cb1-6,4,15','Cb2-2,3,15','Cb2-3,5,15','Cb2-6,4,15','Cb3-2,3,15',...
        'Cb3-3,5,15','Cb3-6,4,15','Cb4-2,3,15','Cb4-3,5,15','Cb4-6,4,15','Cb5-2,3,15','Cb5-3,5,15','Cb5-6,4,15', ...
        'Cc1-2,3,15','Cc1-3,5,15','Cc1-6,4,15','Cc2-2,3,15','Cc2-3,5,15','Cc2-6,4,15','Cc3-2,3,15',...
        'Cc3-3,5,15','Cc3-6,4,15','Cc4-2,3,15','Cc4-3,5,15','Cc4-6,4,15','Cc5-2,3,15','Cc5-3,5,15','Cc5-6,4,15', ...
        'Cd1-2,3,15','Cd1-3,5,15','Cd1-6,4,15','Cd2-2,3,15','Cd2-3,5,15','Cd2-6,4,15','Cd3-2,3,15',...
        'Cd3-3,5,15','Cd3-6,4,15','Cd4-2,3,15','Cd4-3,5,15','Cd4-6,4,15','Cd5-2,3,15','Cd5-3,5,15','Cd5-6,4,15'};
labels30 = {'Ca1-2,3,30','Ca1-3,5,30','Ca1-6,4,30','Ca2-2,3,30','Ca2-3,5,30','Ca2-6,4,30','Ca3-2,3,30',...
        'Ca3-3,5,30','Ca3-6,4,30','Ca4-2,3,30','Ca4-3,5,30','Ca4-6,4,30','Ca5-2,3,30','Ca5-3,5,30','Ca5-6,4,30', ...
        'Cb1-2,3,30','Cb1-3,5,30','Cb1-6,4,30','Cb2-2,3,30','Cb2-3,5,30','Cb2-6,4,30','Cb3-2,3,30',...
        'Cb3-3,5,30','Cb3-6,4,30','Cb4-2,3,30','Cb4-3,5,30','Cb4-6,4,30','Cb5-2,3,30','Cb5-3,5,30','Cb5-6,4,30', ...
        'Cc1-2,3,30','Cc1-3,5,30','Cc1-6,4,30','Cc2-2,3,30','Cc2-3,5,30','Cc2-6,4,30','Cc3-2,3,30',...
        'Cc3-3,5,30','Cc3-6,4,30','Cc4-2,3,30','Cc4-3,5,30','Cc4-6,4,30','Cc5-2,3,30','Cc5-3,5,30','Cc5-6,4,30', ...
        'Cd1-2,3,30','Cd1-3,5,30','Cd1-6,4,30','Cd2-2,3,30','Cd2-3,5,30','Cd2-6,4,30','Cd3-2,3,30',...
        'Cd3-3,5,30','Cd3-6,4,30','Cd4-2,3,30','Cd4-3,5,30','Cd4-6,4,30','Cd5-2,3,30','Cd5-3,5,30','Cd5-6,4,30'};
labels50 = {'Ca1-2,3,50','Ca1-3,5,50','Ca1-6,4,50','Ca2-2,3,50','Ca2-3,5,50','Ca2-6,4,50','Ca3-2,3,50',...
        'Ca3-3,5,50','Ca3-6,4,50','Ca4-2,3,50','Ca4-3,5,50','Ca4-6,4,50','Ca5-2,3,50','Ca5-3,5,50','Ca5-6,4,50', ...
        'Cb1-2,3,50','Cb1-3,5,50','Cb1-6,4,50','Cb2-2,3,50','Cb2-3,5,50','Cb2-6,4,50','Cb3-2,3,50',...
        'Cb3-3,5,50','Cb3-6,4,50','Cb4-2,3,50','Cb4-3,5,50','Cb4-6,4,50','Cb5-2,3,50','Cb5-3,5,50','Cb5-6,4,50', ...
        'Cc1-2,3,50','Cc1-3,5,50','Cc1-6,4,50','Cc2-2,3,50','Cc2-3,5,50','Cc2-6,4,50','Cc3-2,3,50',...
        'Cc3-3,5,50','Cc3-6,4,50','Cc4-2,3,50','Cc4-3,5,50','Cc4-6,4,50','Cc5-2,3,50','Cc5-3,5,50','Cc5-6,4,50', ...
        'Cd1-2,3,50','Cd1-3,5,50','Cd1-6,4,50','Cd2-2,3,50','Cd2-3,5,50','Cd2-6,4,50','Cd3-2,3,50',...
        'Cd3-3,5,50','Cd3-6,4,50','Cd4-2,3,50','Cd4-3,5,50','Cd4-6,4,50','Cd5-2,3,50','Cd5-3,5,50','Cd5-6,4,50'};
labels100 ={'Ca1-2,3,100','Ca1-3,5,100','Ca1-6,4,100','Ca2-2,3,100','Ca2-3,5,100','Ca2-6,4,100','Ca3-2,3,100',...
        'Ca3-3,5,100','Ca3-6,4,100','Ca4-2,3,100','Ca4-3,5,100','Ca4-6,4,100','Ca5-2,3,100','Ca5-3,5,100','Ca5-6,4,100', ...
        'Cb1-2,3,100','Cb1-3,5,100','Cb1-6,4,100','Cb2-2,3,100','Cb2-3,5,100','Cb2-6,4,100','Cb3-2,3,100',...
        'Cb3-3,5,100','Cb3-6,4,100','Cb4-2,3,100','Cb4-3,5,100','Cb4-6,4,100','Cb5-2,3,100','Cb5-3,5,100','Cb5-6,4,100', ...
        'Cc1-2,3,100','Cc1-3,5,100','Cc1-6,4,100','Cc2-2,3,100','Cc2-3,5,100','Cc2-6,4,100','Cc3-2,3,100',...
        'Cc3-3,5,100','Cc3-6,4,100','Cc4-2,3,100','Cc4-3,5,100','Cc4-6,4,100','Cc5-2,3,100','Cc5-3,5,100','Cc5-6,4,100', ...
        'Cd1-2,3,100','Cd1-3,5,100','Cd1-6,4,100','Cd2-2,3,100','Cd2-3,5,100','Cd2-6,4,100','Cd3-2,3,100',...
        'Cd3-3,5,100','Cd3-6,4,100','Cd4-2,3,100','Cd4-3,5,100','Cd4-6,4,100','Cd5-2,3,100','Cd5-3,5,100','Cd5-6,4,100'};

label15 = {'Ca1-2,3,15','Ca1-3,5,15','Ca1-6,4,15','Ca2-2,3,15','Ca2-3,5,15','Ca2-6,4,15',...
        'Ca3-2,3,15','Ca3-3,5,15','Ca3-6,4,15','Ca4-2,3,15','Ca4-3,5,15'...
        'Ca4-6,4,15','Ca5-2,3,15','Ca5-3,5,15','Ca5-6,4,15'};
label30 = {'Ca1-2,3,30','Ca1-3,5,30','Ca1-6,4,30','Ca2-2,3,30','Ca2-3,5,30','Ca2-6,4,30',...
        'Ca3-2,3,30','Ca3-3,5,30','Ca3-6,4,30','Ca4-2,3,30','Ca4-3,5,30'...
        'Ca4-6,4,30','Ca5-2,3,30','Ca5-3,5,30','Ca5-6,4,30'};
label50 = {'Ca1-2,3,50','Ca1-3,5,50','Ca1-6,4,50','Ca2-2,3,50','Ca2-3,5,50','Ca2-6,4,50',...
        'Ca3-2,3,50','Ca3-3,5,50','Ca3-6,4,50','Ca4-2,3,50','Ca4-3,5,50'...
        'Ca4-6,4,50','Ca5-2,3,50','Ca5-3,5,50','Ca5-6,4,50'};
label100 = {'Ca1-2,3,100','Ca1-3,5,100','Ca1-6,4,100','Ca2-2,3,100','Ca2-3,5,100','Ca2-6,4,100',...
        'Ca3-2,3,100','Ca3-3,5,100','Ca3-6,4,100','Ca4-2,3,100','Ca4-3,5,100'...
        'Ca4-6,4,100','Ca5-2,3,100','Ca5-3,5,100','Ca5-6,4,100'};

% 自定义图形属性
ylabel('cost');
xlim([0,16]);
% ylim([800,3000]);
% ylim([1400,4000]);
% ylim([2000,5500]);
ylim([3500,9500]);
grid on;
hold off;
set(gca,'XTick',1:1:16);
set(gca,'XTickLabel',label100);

