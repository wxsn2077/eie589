clc;
clear;
% 创建一个 20x20 的矩阵
matrix_size = 20;
matrix = zeros(matrix_size);  % 初始化矩阵
% 随机选择 120 个红色点
all_coords = (1:matrix_size^2)';  % 生成 1 到 400 的所有坐标
all_coords([1, 20*20]) = [];  % 移除 (1, 1) 和 (20, 20)
random_indices = randperm(length(all_coords), 120);
random_coords = all_coords(random_indices);
% 将线性索引转换为 (x, y) 坐标
[x_red, y_red] = ind2sub([matrix_size, matrix_size], random_coords);
% 将红色点标记为不可通行区域
matrix(sub2ind(size(matrix), x_red, y_red)) = 1;  % 障碍物
% 计算最短路径
[path, distance] = dijkstra(matrix, [1, 1], [20, 20]);
% 绘制结果
figure;
hold on;

% 绘制红色点
scatter(x_red, y_red, 'filled', 'r');  % 绘制红色点

% 绘制绿色点
[x,y] = find(matrix == 0);
scatter(x, y, 'filled', 'g');  % 绘制绿色点

% 绘制起点和终点
scatter(1, 1, 'filled', 'g');  % (1, 1) 点
scatter(20, 20, 'filled', 'g');  % (20, 20) 点
% 绘制路径
if ~isempty(path)
    plot(path(:, 1), path(:, 2), 'k-', 'LineWidth', 2);  % 绘制最短路径
else
    disp('未找到路径');
end
% 设置图形属性
xlim([0.5, 20.5]);ylim([0.5, 20.5]);
xticks(1:20);yticks(1:20);
grid on;
title('通过绿色点的路径');
xlabel('X 坐标');ylabel('Y 坐标');
axis equal;
hold off;
% 输出最短路径的距离
fprintf('最短路径的距离: %.2f\n', distance);

function [path, distance] = dijkstra(matrix, start, goal)
    % Dijkstra算法实现，start和goal位置是[x,x]的坐标矩阵，matrix位置需要输入图像矩阵
    open_set = start;  % open_set是一个变化矩阵
    came_from = zeros(size(matrix));  % 路径，20*20矩阵初始为0，最后came_from矩阵存储的是每个坐标的下一个路径的索引值
    g_score = inf(size(matrix));  % 初始化g_score矩阵，每个值为无穷大inf
    g_score(start(1), start(2)) = 0;  % 起点的 g_score
    assignin('base', 'open_set', open_set);

    while ~isempty(open_set)
        % 找到 g_score 最小的节点
        qpp = g_score(sub2ind(size(matrix), open_set(:, 1), open_set(:, 2)));
        [~, idx] = min(qpp);
        %sub2ind用法：sub2ind([x,y],a,b)，在x行y列的矩阵里找出a行b列的位置值（索引）
        %min用法：
        current = open_set(idx, :); %获得第idx行的元素（其实只有两个元素，所以刚好形成坐标（x，y）
        % 如果到达目标，则构建路径
        if isequal(current, goal)
            path = reconstruct_path(came_from, current);
            distance = g_score(goal(1), goal(2));
            return;
        end
        % 从开放列表中移除当前节点
        open_set(idx, :) = [];
        % 获取邻居节点
        neighbors = get_neighbors(current, size(matrix));
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            if matrix(neighbor(1), neighbor(2)) == 1  % 如果是障碍物，跳过
                continue;
            end
            tentative_g_score = g_score(current(1), current(2)) + sqrt(sum((current - neighbor).^2)); %norm：计算两个坐标的直线距离
            if g_score(neighbor(1), neighbor(2)) == Inf
                came_from(neighbor(1), neighbor(2)) = sub2ind(size(matrix), current(1), current(2));%在neighbor写入current的索引值
                g_score(neighbor(1), neighbor(2)) = tentative_g_score;
                if ~ismember(neighbor, open_set, 'rows')
                    open_set = [open_set; neighbor];  % 添加到开放列表
                end
            end
        end
    end
    path = [];  % 如果没有路径
    distance = inf;  % 如果没有找到路径
end

function path = reconstruct_path(came_from, current)
%came_from矩阵存储的是每个坐标的下一个路径的索引值，所以要通过ind2sub转换为坐标交给prev_x，prev_y，再把坐标存到path
    path = current;  % 从当前节点开始
    while came_from(current(1), current(2)) ~= 0
        [prev_x, prev_y] = ind2sub(size(came_from), came_from(current(1), current(2)));
        current = [prev_x, prev_y];
        path = [current; path];  % 逐步构建路径，最终path存储内容为第一列是x第二列是y的路径坐标
    end
end

function neighbors = get_neighbors(node, grid_size)
    % 获取邻居节点，包括斜向
    directions = [-1, -1; -1, 0; -1, 1; 0, -1; 0, 1; 1, -1; 1, 0; 1, 1];
    neighbors = [];
    for i = 1:size(directions, 1)
        neighbor = node + directions(i, :);
        if all(neighbor > 0) && all(neighbor <= grid_size)
            neighbors = [neighbors; neighbor];  
        end
    end
end

