clc; clear; close all;

% 定义算法、客户规模和仓库类别
algorithms = {'CRCEA$^1$', 'CRCEA$^2$', 'CRCEA$^3$', 'CRCEA$^4$', 'CRCEA'};
algorithm_name1 = {'CRCEA1','CRCEA2','CRCEA3','CRCEA4','CRCEA'};
customer_sizes = [15, 30, 50, 100];
num_warehouses = 3;
n_algorithms = length(algorithms);
n_customers = length(customer_sizes);

Cus = cell(4,5,15);
data1 = zeros(n_customers, num_warehouses, n_algorithms); % (4,3,5)

for k = 1:4     %4类客户规模
    for i = k:4:60      %15个案例
        for j = 1:5     %5个算法
            filename = ['F:\2E_Data\',algorithm_name1{j},'\vrp_data\',algorithm_name1{j},'_Set1','_num',num2str(i),'.mat'];
            load(filename)
            Cus{k,j,ceil(i/4)} = opt_vrps(1:12);
        end
    end
end

% 分组索引
groups = {[1,4,7,10,13], [2,5,8,11,14], [3,6,9,12,15]};

% 遍历规模类别 i
for i = 1:4
    % 遍历算法 k
    for k = 1:5
        % 遍历分组 j
        for j = 1:3
            % 取出当前分组的索引
            idx = groups{j};
            % 提取 cell 中的数据并求均值
            values = cellfun(@(x) mean(x(:)), Cus(i, k, idx)); 
            data1(i, j, k) = mean(values);
        end
    end
end


% 设置柱子的索引
base_offset = 20; % 每个客户规模之间的间隔
small_offset = 6; % 每个仓库类别之间的间隔
bar_width = 1; % 柱子宽度

x_positions = zeros(n_customers, num_warehouses, n_algorithms);
base_pos = 0;
for i = 1:n_customers
    for j = 1:num_warehouses
        start_idx = base_pos + (j - 1) * small_offset;
        x_positions(i, j, :) = start_idx + (1:n_algorithms);
    end
    base_pos = base_pos + base_offset;
end

figure;
hold on;
colors = lines(n_algorithms); % 选取默认颜色

% 绘制柱状图
for i = 1:n_customers
    for j = 1:num_warehouses
        for k = 1:n_algorithms
            bar(x_positions(i, j, k), data1(i, j, k), bar_width, 'FaceColor', colors(k, :));
        end
    end
end

% 计算小块（A1-A12）中心位置
sub_positions = mean(x_positions, 3);
sub_positions = sub_positions(:)'; % 展平并转换为行向量
sorted_sub_positions = sort(sub_positions); % 确保是严格递增的数值向量
xticks(sorted_sub_positions);
sub_labels = ['(2,3)';'(3,5)';'(6,4)';'(2,3)';'(3,5)';'(6,4)';'(2,3)';'(3,5)';'(6,4)';'(2,3)';'(3,5)';'(6,4)'];
xticklabels(sub_labels);

% 计算大块（B1-B4）中心位置，位于 A2, A5, A8, A11 正下方
big_indices = 2:3:length(sorted_sub_positions); % 选取每个大块的中间小块
big_positions = sorted_sub_positions(big_indices);
customer_labels = {'15 customers', '30 customers', '50 customers', ' 100 customer'};
for i = 1:n_customers
    label = customer_labels{min(i, numel(customer_labels))};  % Ensure the label index is within bounds
    text(big_positions(i), min(ylim) - 0.1 * range(ylim), label, 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

ylim([0,9000]);

% xlabel('Customer Scale');
ylabel('Mean cost');
grid on;
hold off;
legend(algorithms, 'Location', 'northwest', 'Interpreter', 'latex');

% title('Comparison of mean cost between CRCEA and other variants on instances with different depots, satellites, and customer scale');