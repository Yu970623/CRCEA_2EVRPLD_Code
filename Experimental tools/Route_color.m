% A点和B点的坐标
A = [0, 0];  % 假设 A 点坐标为 (0, 0)
B = [10, 10]; % 假设 B 点坐标为 (10, 10)

% Energy 在 A 和 B 的值
Energy_A = 0.8; % A 点的 Energy
Energy_B = 0.4; % B 点的 Energy

% 定义颜色渐变中的中介颜色
color = [
    0/255, 255/255, 0/255;      % 绿色
    144/255, 255/255, 144/255;  % 浅绿色
    255/255, 255/255, 102/255;  % 浅黄色
    255/255, 165/255, 0/255;    % 浅橙色
    139/255, 69/255, 19/255;    % 橙色
    255/255, 0/255, 0/255;      % 红色
];

% 将这些颜色插值为一个更细腻的渐变标尺
nColors = 100; % 定义渐变颜色的数量

% 使用 `linspace` 生成更细腻的颜色渐变
nSteps = size(color, 1) - 1; % 计算中间颜色的数量

% 为每对相邻的颜色生成渐变
colors = [];
for i = 1:nSteps
    % 对每对颜色进行线性插值
    colors = [colors; [linspace(color(i,1), color(i+1,1), nColors/nSteps)]', ...
                                   [linspace(color(i,2), color(i+1,2), nColors/nSteps)]', ...
                                   [linspace(color(i,3), color(i+1,3), nColors/nSteps)]'];
end

% 确保渐变色的数量为 nColors
colors = colors(1:nColors, :);

% Energy 的标尺是从 1 到 0
Energy_scale = linspace(1, 0, nColors); % Energy 从 1 到 0

% 计算 Energy_A 和 Energy_B 在标尺中的位置
[~, idx_A] = min(abs(Energy_scale - Energy_A));  % 找到 Energy_A 对应的颜色位置
[~, idx_B] = min(abs(Energy_scale - Energy_B));  % 找到 Energy_B 对应的颜色位置

num = idx_B-idx_A;

% 计算渐变直线的坐标
x = linspace(A(1), B(1), num);  % 100 个点的 x 坐标
y = linspace(A(2), B(2), num);  % 100 个点的 y 坐标

% 绘制渐变线
figure;
hold on;
for i = 1:length(x)-1
    % 使用 Energy 标尺的颜色
    plot(x(i:i+1), y(i:i+1), 'Color', colors(idx_A+i, :), 'LineWidth', 2);
end
hold off;

% 显示结果
axis equal;
title('Gradient Line Based on Energy');





