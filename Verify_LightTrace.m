clc;clear;
%% 光线与房间表面碰撞检测
% 房间参数（单位：米）
room_length = 5;     % 房间长度 (x方向)
room_width = 5;      % 房间宽度 (y方向) 
room_height = 3;      % 房间高度 (z方向)

% 光源位置（基于文档Table 1）
light_position = [2.5, 2.5, 2.9];  % 中心位置，天花板高度
mode = 1;

% 定义房间的六个表面
% 每个表面用平面方程 Ax + By + Cz + D = 0 表示
surfaces = struct();

% 1. 天花板 (z = room_height)
surfaces.ceiling.A = 0; surfaces.ceiling.B = 0; surfaces.ceiling.C = 1; 
surfaces.ceiling.D = -room_height;
surfaces.ceiling.normal = [0, 0, 1];  % 指向房间内部
surfaces.ceiling.center = [room_length/2, room_width/2, room_height];
surfaces.ceiling.bounds = {[0, room_length], [0, room_width], [room_height, room_height]};
surfaces.ceiling.name = 'Ceiling';

% 2. 地板 (z = 0)
surfaces.floor.A = 0; surfaces.floor.B = 0; surfaces.floor.C = 1;
surfaces.floor.D = 0;
surfaces.floor.normal = [0, 0, -1];  % 指向房间内部
surfaces.floor.center = [room_length/2, room_width/2, 0];
surfaces.floor.bounds = {[0, room_length], [0, room_width], [0, 0]};
surfaces.floor.name = 'Floor';

% 3. 前墙 (y = 0)
surfaces.front.A = 0; surfaces.front.B = 1; surfaces.front.C = 0;
surfaces.front.D = 0;
surfaces.front.normal = [0, 1, 0];  % 指向房间内部
surfaces.front.center = [room_length/2, 0, room_height/2];
surfaces.front.bounds = {[0, room_length], [0, 0], [0, room_height]};
surfaces.front.name = 'Front Wall';

% 4. 后墙 (y = room_width)
surfaces.back.A = 0; surfaces.back.B = 1; surfaces.back.C = 0;
surfaces.back.D = -room_width;
surfaces.back.normal = [0, -1, 0];  % 指向房间内部
surfaces.back.center = [room_length/2, room_width, room_height/2];
surfaces.back.bounds = {[0, room_length], [room_width, room_width], [0, room_height]};
surfaces.back.name = 'Back Wall';

% 5. 左墙 (x = 0)
surfaces.left.A = 1; surfaces.left.B = 0; surfaces.left.C = 0;
surfaces.left.D = 0;
surfaces.left.normal = [1, 0, 0];  % 指向房间内部
surfaces.left.center = [0, room_width/2, room_height/2];
surfaces.left.bounds = {[0, 0], [0, room_width], [0, room_height]};
surfaces.left.name = 'Left Wall';

% 6. 右墙 (x = room_length)
surfaces.right.A = 1; surfaces.right.B = 0; surfaces.right.C = 0;
surfaces.right.D = -room_length;
surfaces.right.normal = [-1, 0, 0];  % 指向房间内部
surfaces.right.center = [room_length, room_width/2, room_height/2];
surfaces.right.bounds = {[room_length, room_length], [0, room_width], [0, room_height]};
surfaces.right.name = 'Right Wall';



%% 光线采样和碰撞检测测试
fprintf('\n=== 光线与房间表面碰撞检测 ===\n');
fprintf('房间尺寸: %.1f x %.1f x %.1f 米\n', room_length, room_width, room_height);
fprintf('光源位置: (%.1f, %.1f, %.1f)\n', light_position);

% 测试不同方向的光线
test_rays = 10;
hit_statistics = struct();

for ray_idx = 1:test_rays
    % 从光源发射光线
    start_point = light_position;
    
    % 使用Lambertian分布采样方向
    if ray_idx <= 5
        % 前5条：使用Lambertian采样
        theta = acos(rand^(1/(mode+1)));%加pi/2使得Lambertian采样方向向下，
        phi = 2*pi*rand;
        [x, y, z] = sph2cart(phi, theta, 1);
        direction = [x, y, -z];%Lambertian采样方向向下，是否要改成-z
    else
        % 后5条：测试特定方向
        test_directions = [
            0, 0, -1;    % 垂直向下
            1, 0, 0;     % 向右
            0, 1, 0;     % 向前
            -0.707, 0, -0.707;  % 左下
            0, -0.5, -0.866;   % 后下
        ];
        direction = test_directions(ray_idx-5, :);
    end
    
    % 执行碰撞检测
    [hit_surface, hit_point, hit_distance] = LightTrace_rayIntersectRoom(start_point, direction, surfaces);
    
    % 记录结果
    if ~isempty(hit_surface)
        fprintf('光线 %2d: 方向[%6.3f, %6.3f, %6.3f] → 命中 %s ', ...
                ray_idx, direction(1), direction(2), direction(3), hit_surface);
        fprintf('交点(%.3f, %.3f, %.3f) 距离: %.3f米\n', ...
                hit_point(1), hit_point(2), hit_point(3), hit_distance);
        
        % 统计
        if isfield(hit_statistics, hit_surface)
            hit_statistics.(hit_surface) = hit_statistics.(hit_surface) + 1;
        else
            hit_statistics.(hit_surface) = 1;
        end
    else
        fprintf('光线 %2d: 未命中任何表面\n', ray_idx);
    end
end

%% 批量测试Lambertian采样
fprintf('\n=== 批量Lambertian采样碰撞测试 ===\n');
num_test_rays = 10000;
surface_hits = zeros(6, 1);
surface_names = {'Floor', 'Ceiling', 'Front Wall', 'Back Wall', 'Left Wall', 'Right Wall'};
ray_distances = zeros(num_test_rays, 1);

for i = 1:num_test_rays
    % Lambertian采样方向
    theta = acos(rand^(1/(mode+1)));
    phi = 2*pi*rand;
    [x, y, z] = sph2cart(phi, theta, 1);
    direction = [x, y, -z];%朝下
    
    % 碰撞检测
    [hit_surface, ~, distance] = LightTrace_rayIntersectRoom(light_position, direction, surfaces);
    
    if ~isempty(hit_surface)
        ray_distances(i) = distance;
        
        % 统计命中
        switch hit_surface
            case 'floor'
                surface_hits(1) = surface_hits(1) + 1;
            case 'ceiling'
                surface_hits(2) = surface_hits(2) + 1;
            case 'front'
                surface_hits(3) = surface_hits(3) + 1;
            case 'back'
                surface_hits(4) = surface_hits(4) + 1;
            case 'left'
                surface_hits(5) = surface_hits(5) + 1;
            case 'right'
                surface_hits(6) = surface_hits(6) + 1;
        end
    end
end

% 显示统计结果
fprintf('\n表面命中统计（基于%d条光线）:\n', num_test_rays);
for i = 1:6
    hit_prob = surface_hits(i) / num_test_rays * 100;
    fprintf('  %-12s: %6d次 (%.2f%%)\n', surface_names{i}, surface_hits(i), hit_prob);
end
fprintf('平均命中距离: %.3f米\n', mean(ray_distances(ray_distances>0)));
fprintf('最短距离: %.3f米\n', min(ray_distances(ray_distances>0)));
fprintf('最长距离: %.3f米\n', max(ray_distances));

%% 碰撞检测可视化
figure('Position', [100, 100, 1400, 600]);

% 子图1：房间三维视图
%subplot(1,3,1);
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('房间与光线碰撞检测');
view(45, 30);

% 绘制房间框架
vertices = [0 0 0; room_length 0 0; room_length room_width 0; 0 room_width 0; ...
            0 0 room_height; room_length 0 room_height; ...
            room_length room_width room_height; 0 room_width room_height];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Vertices', vertices, 'Faces', faces, ...
      'FaceColor', [0.8 0.9 1], 'FaceAlpha', 0.2, 'EdgeColor', 'k');

% 绘制光源
plot3(light_position(1), light_position(2), light_position(3), 'ro', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '光源');
%% === 添加墙面标签代码（在这里插入）===
% 前墙标签 (y=0)
text(room_length/2, -0.2, room_height/2, '前墙', ...
     'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', ...
     'Color', 'blue', 'BackgroundColor', 'white');

% 后墙标签 (y=room_width)
text(room_length/2, room_width+0.2, room_height/2, '后墙', ...
     'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', ...
     'Color', 'blue', 'BackgroundColor', 'white');

% 左墙标签 (x=0)
text(-0.2, room_width/2, room_height/2, '左墙', ...
     'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', ...
     'Color', 'red', 'BackgroundColor', 'white', 'Rotation', 90);

% 右墙标签 (x=room_length)
text(room_length+0.2, room_width/2, room_height/2, '右墙', ...
     'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', ...
     'Color', 'red', 'BackgroundColor', 'white', 'Rotation', -90);

%% === 墙面标签代码结束 ===
% 绘制示例光线
example_rays = 50;
for i = 1:example_rays
    % Lambertian采样
    theta = acos(rand^(1/(mode+1)));
    phi = 2*pi*rand;
    [x, y, z] = sph2cart(phi, theta, 1);
    direction = [x, y, -z];
    
    % 碰撞检测
    [hit_surface, hit_point, distance] = LightTrace_rayIntersectRoom(light_position, direction, surfaces);
    
    if ~isempty(hit_surface)
        % 绘制光线
        ray_points = [light_position; hit_point];
        plot3(ray_points(:,1), ray_points(:,2), ray_points(:,3), 'b-', 'LineWidth', 0.5);
        
        % 标记碰撞点
        switch hit_surface
            case 'floor'
                plot3(hit_point(1), hit_point(2), hit_point(3), 'g.', 'MarkerSize', 8);
            case 'ceiling'
                plot3(hit_point(1), hit_point(2), hit_point(3), 'r.', 'MarkerSize', 8);
            otherwise
                plot3(hit_point(1), hit_point(2), hit_point(3), 'b.', 'MarkerSize', 8);
        end
    end
end

legend('房间', '光源', '光线路径', '碰撞点');
xlim([-0.5, room_length+0.5]);
ylim([-0.5, room_width+0.5]);
zlim([-0.5, room_height+0.5]);

% 窗口2：显示子图2和子图3（统计图表）
figure('Position', [1100, 100, 1000, 600], 'Name', '碰撞检测统计结果', 'NumberTitle', 'off');
fprintf('生成统计结果视图...\n');

% 子图2：命中距离分布
subplot(1,2,1);
histogram(ray_distances(ray_distances>0), 50, 'FaceColor', [0.2 0.6 0.8], 'FaceAlpha', 0.7);
xlabel('光线传播距离 (米)');
ylabel('频数');
title('光线命中距离分布');
grid on;

% 添加统计信息
mean_dist = mean(ray_distances(ray_distances>0));
std_dist = std(ray_distances(ray_distances>0));
text(0.6*max(xlim), 0.8*max(ylim), ...
     sprintf('平均: %.3f米\n标准差: %.3f米', mean_dist, std_dist), ...
     'FontSize', 10, 'BackgroundColor', 'white');

% 子图3：表面命中概率分布
subplot(1,2,2);
hit_percentages = surface_hits / sum(surface_hits) * 100;
bar(hit_percentages, 'FaceColor', [0.8 0.4 0.2], 'FaceAlpha', 0.7);
set(gca, 'XTickLabel', surface_names, 'XTickLabelRotation', 45);
ylabel('命中概率 (%)');
title('各表面命中概率分布');
grid on;

% 在柱状图上添加数值
for i = 1:6
    text(i, hit_percentages(i)+0.5, sprintf('%.1f%%', hit_percentages(i)), ...
         'HorizontalAlignment', 'center', 'FontSize', 9);
end


fprintf('\n碰撞检测模块完成！\n');

