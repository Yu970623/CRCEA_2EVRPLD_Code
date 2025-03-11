%% ----------------------存储计算的结果-----------------------------
% function figroute(coord_dep,coord_sat,coord_cus,opt_sat,opt_cus,max_gen)
color = [0/255, 255/255, 0/255;        % 绿色
%     72/255, 255/255, 72/255;     % 过渡颜色 1
    144/255, 255/255, 144/255;   % 浅绿色
%     199/255, 255/255, 123/255;   % 过渡颜色 2
    255/255, 255/255, 102/255;   % 浅黄色
%     255/255, 210/255, 51/255;    % 过渡颜色 3
    255/255, 165/255, 0/255;     % 浅橙色
%     197/255, 117/255, 9/255;     % 过渡颜色 4
    139/255, 69/255, 19/255;     % 橙色
%     197/255, 34/255, 9/255;      % 过渡颜色 5
    255/255, 0/255, 0/255];        % 红色

% 将这些颜色插值为一个更细腻的渐变标尺
nColors = 1000; % 定义渐变颜色的数量

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
result = 0;
algorithm_name = {'LNS','ALNS','EHG','CRCEA'};
for l = 28
    filePath = fullfile('F:\2E_Data\CRCEAW2\vrp_data\', ['CRCEA_Set1_num', num2str(l), '.mat']);
    disp(filePath);  % 输出生成的文件路径，检查是否正确
    load(filePath);
    coord_dep = vrp2e.coord_dep;
    coord_sat = vrp2e.coord_sat;
    coord_cus = vrp2e.coord_cus;
    type = vrp2e.type;
    dis_sc = vrp2e.dis_sc;
    E = vrp2e.E;
    V = vrp2e.V;
    vlt = find(opt_vlts==0);
    vrp = find(opt_vrps==min(opt_vrps(vlt)),1);
    result = opt_vrps(vrp);
    opt_sat = opt_sats{vrp};
    opt_cus = opt_cuss{vrp};
    opt_sat(opt_sat(1:1:end) == 0)=[];
    opt_cus(opt_cus(1:1:end) == 0)=[];
    figure;
    trans = 1;  %标注的位置平移量
    plot(coord_dep(:,1),coord_dep(:,2),'s','MarkerEdgeColor','b','MarkerFaceColor','r','MarkerSize',10);
    hold on;
    plot(coord_sat(:,1),coord_sat(:,2),'h','MarkerEdgeColor','r','MarkerFaceColor','b','MarkerSize',10);
    hold on;
    for i = 1:size(coord_cus,1)
        if strcmp(type{i}, 'delivery')
            plot(coord_cus(i,1),coord_cus(i,2),'o','MarkerEdgeColor','none','MarkerFaceColor',[0.5, 0.5, 0.5],'MarkerSize',8);
        else
            plot(coord_cus(i,1),coord_cus(i,2),'o','MarkerEdgeColor', [0.5, 0.5, 0.5], 'MarkerFaceColor', 'none', 'LineWidth', 1,'MarkerSize',8);
        end
    end
    
    hold on;

    %----画出第一层路径----
    len_sat = length(opt_sat);
    xy_sat  = zeros(len_sat,2);
    label_s   = cell(1,len_sat);
    k1 = 1;         %记录存储矩阵的位置
    k2 = 1;         %记录卫星点
    xy_sat(k1,:) = coord_dep(opt_sat(k2),:);
    label_s{k1}  = opt_sat(k2);
    for i = 2:len_sat
        k1 = k1+1;
        if opt_sat(i)<=length(coord_dep)
            xy_sat(k1,:) = coord_dep(opt_sat(k2),:);
            label_s{k1}  = opt_sat(k2);
            plot(xy_sat(1:k1,1),xy_sat(1:k1,2),'--b');
%             text(xy_cus(2:r1-1,1)+trans,xy_cus(2:r1-1,2)+trans, label_c(2:r1-1));
            hold on;
            k2 = i;
            k1 = 1;
            xy_sat(k1,:) = coord_dep(opt_sat(k2),:);
            label_s{k1}  = opt_sat(k2);
        else
            xy_sat(k1,:) = coord_sat(opt_sat(i)-length(coord_dep),:);
            label_s{k1}  = opt_sat(i);
        end
    end
    k1 = k1+1;
    xy_sat(k1,:) = coord_dep(opt_sat(k2),:);
    label_s{k1}  = opt_sat(k2);
    plot(xy_sat(1:k1,1),xy_sat(1:k1,2),'--b');
%     text(xy_sat(:,1)+trans,xy_sat(:,2)+trans, label_s);
    hold on;
    
    %----画出第二层路径----
    len_cus = length(opt_cus);
    xy_cus  = zeros(len_cus,2);
    label_c = cell(1,len_cus);
    r1 = 1;         %记录存储矩阵的位置
    r2 = 1;         %记录卫星点
    xy_cus(r1,:) = coord_sat(opt_cus(r2),:);
    label_c{r1}  = opt_cus(r2);
    for i = 2:len_cus
        r1 = r1+1;
        if opt_cus(i)<=length(coord_sat)
            xy_cus(r1,:) = coord_sat(opt_cus(r2),:);
            label_c{r1}  = opt_cus(r2);
            route = [opt_cus(r2:i-1),opt_cus(r2)];  %子路段
            distance_route = dis_sc(sub2ind(size(dis_sc), route(1:end-1), route(2:end))); %子路段长度
            route_cap = energy(vrp2e,route);   %每个路段的载重
            Energy_cap = cell2mat(arrayfun(@(m,n)sum((5+m)*(3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4).*(n/V)), route_cap,distance_route, 'UniformOutput', false)); 
            Energy = (E - [0, cumsum(Energy_cap)])/E;
            for k =1:length(route_cap)
                Energy_A = Energy(k);
                Energy_B = Energy(k+1);
                [~, idx_A] = min(abs(Energy_scale - Energy_A));  % 找到 Energy_A 对应的颜色位置
                [~, idx_B] = min(abs(Energy_scale - Energy_B));  % 找到 Energy_B 对应的颜色位置
                num = idx_B-idx_A;
                % 计算渐变直线的坐标
                x = linspace(xy_cus(k,1), xy_cus(k+1,1), num);  % 100 个点的 x 坐标
                y = linspace(xy_cus(k,2), xy_cus(k+1,2), num);  % 100 个点的 y 坐标
                for p = 1:length(x)-1
                    % 使用 Energy 标尺的颜色
                    plot(x(p:p+1), y(p:p+1), 'Color', colors(idx_A+p, :), 'LineWidth', 2.5);
                end
                plot(xy_cus(k:k+1,1),xy_cus(k:k+1,2), 'LineStyle', '--', 'LineWidth', 1.5,'Color', color(ceil(route_cap(k)+0.01), :));
            end
%             text(xy_cus(2:r1-1,1)+trans,xy_cus(2:r1-1,2)+trans, label_c(2:r1-1));
            hold on;
            r2 = i;
            r1 = 1;
            xy_cus(r1,:) = coord_sat(opt_cus(r2),:);
            label_c{r1}  = opt_cus(r2);
        else
            xy_cus(r1,:) = coord_cus(opt_cus(i)-length(coord_sat),:);
            label_c{r1}  = opt_cus(i);
        end
    end
    r1 = r1+1;
    xy_cus(r1,:) = coord_sat(opt_cus(r2),:);
    label_c{r1}  = opt_cus(r2);
    route = [opt_cus(r2:end),opt_cus(r2)];  %子路段
    distance_route = dis_sc(sub2ind(size(dis_sc), route(1:end-1), route(2:end))); %子路段长度
    route_cap = energy(vrp2e,route);   %每个路段的载重
    Energy_cap = cell2mat(arrayfun(@(m,n)sum((5+m)*(3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4).*(n/V)), route_cap,distance_route, 'UniformOutput', false)); 
    Energy = (E - [0, cumsum(Energy_cap)])/E;
    for k =1:length(route_cap)
        Energy_A = Energy(k);
        Energy_B = Energy(k+1);
        [~, idx_A] = min(abs(Energy_scale - Energy_A));  % 找到 Energy_A 对应的颜色位置
        [~, idx_B] = min(abs(Energy_scale - Energy_B));  % 找到 Energy_B 对应的颜色位置
        num = idx_B-idx_A;
        % 计算渐变直线的坐标
        x = linspace(xy_cus(k,1), xy_cus(k+1,1), num);  % 100 个点的 x 坐标
        y = linspace(xy_cus(k,2), xy_cus(k+1,2), num);  % 100 个点的 y 坐标
        for p = 1:length(x)-1
            % 使用 Energy 标尺的颜色
            plot(x(p:p+1), y(p:p+1), 'Color', colors(idx_A+p, :), 'LineWidth', 2.5);
        end
        plot(xy_cus(k:k+1,1),xy_cus(k:k+1,2), 'LineStyle', '--', 'LineWidth', 1.5,'Color', color(ceil(route_cap(k)+0.01), :));
    end
%     text(xy_cus(2:r1-1,1)+trans,xy_cus(2:r1-1,2)+trans, label_c(2:r1-1));
    hold off;
    xlabel('x-coordinate','FontWeight','bold');
    ylabel('y-coordinate','FontWeight','bold');
    title(['Best cost',num2str(result)],'FontWeight','bold');
%     legend('Even Design',1);
    axis equal;
    grid on;
    box on;
end