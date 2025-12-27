clc; clear;
%% 光线与房间表面碰撞检测（包含反射）
% 房间参数（单位：米）
room_length = 5;     % 房间长度 (x方向)
room_width = 5;      % 房间宽度 (y方向) 
room_height = 3;      % 房间高度 (z方向)

% 光源位置（基于文档Table 1）
light_position = [2.5, 2.5, 2.9];  % 中心位置，天花板高度
mode = 1;

%% 修改一：添加接收机定义
% 根据图片中的接收机参数
receiver_area = 1;           % 面积 1 cm² = 0.0001 m²，用面积为1来测试
receiver_position = [0.5, 1, 0];  % 位置 (x, y, z)
receiver_normal = [0, 0, 1];        % 法向量 (向上)
receiver_fov = 85;                  % 视场角 (度)
receiver_radius = sqrt(receiver_area/pi);  % 等效半径

fprintf('\n=== 接收机参数 ===\n');
fprintf('面积: %.2f cm² (%.6f m²)\n', receiver_area*1e4, receiver_area);
fprintf('位置: (%.1f, %.1f, %.1f) 米\n', receiver_position);
fprintf('法向量: [%.0f, %.0f, %.0f]\n', receiver_normal);
fprintf('视场角: %.0f 度\n', receiver_fov);
fprintf('等效半径: %.6f 米\n', receiver_radius);

%% 定义表面反射系数
fprintf('\n=== 表面反射系数 ===\n');
fprintf('地板和墙壁反射系数: 0.8\n');
fprintf('天花板反射系数: 0.3\n');
fprintf('最大反射次数: 10\n');

% 定义房间的六个表面（添加反射系数）
surfaces = struct();

% 1. 天花板 (z = room_height)
surfaces.ceiling.A = 0; surfaces.ceiling.B = 0; surfaces.ceiling.C = 1; 
surfaces.ceiling.D = -room_height;
surfaces.ceiling.normal = [0, 0, 1];  % 指向房间内部
surfaces.ceiling.center = [room_length/2, room_width/2, room_height];
surfaces.ceiling.bounds = {[0, room_length], [0, room_width], [room_height, room_height]};
surfaces.ceiling.name = 'Ceiling';
surfaces.ceiling.reflectivity = 0.3;  % 天花板反射系数

% 2. 地板 (z = 0)
surfaces.floor.A = 0; surfaces.floor.B = 0; surfaces.floor.C = 1;
surfaces.floor.D = 0;
surfaces.floor.normal = [0, 0, -1];  % 指向房间内部
surfaces.floor.center = [room_length/2, room_width/2, 0];
surfaces.floor.bounds = {[0, room_length], [0, room_width], [0, 0]};
surfaces.floor.name = 'Floor';
surfaces.floor.reflectivity = 0.8;  % 地板反射系数

% 3. 前墙 (y = 0)
surfaces.front.A = 0; surfaces.front.B = 1; surfaces.front.C = 0;
surfaces.front.D = 0;
surfaces.front.normal = [0, 1, 0];  % 指向房间内部
surfaces.front.center = [room_length/2, 0, room_height/2];
surfaces.front.bounds = {[0, room_length], [0, 0], [0, room_height]};
surfaces.front.name = 'Front Wall';
surfaces.front.reflectivity = 0.8;  % 墙壁反射系数

% 4. 后墙 (y = room_width)
surfaces.back.A = 0; surfaces.back.B = 1; surfaces.back.C = 0;
surfaces.back.D = -room_width;
surfaces.back.normal = [0, -1, 0];  % 指向房间内部
surfaces.back.center = [room_length/2, room_width, room_height/2];
surfaces.back.bounds = {[0, room_length], [room_width, room_width], [0, room_height]};
surfaces.back.name = 'Back Wall';
surfaces.back.reflectivity = 0.8;  % 墙壁反射系数

% 5. 左墙 (x = 0)
surfaces.left.A = 1; surfaces.left.B = 0; surfaces.left.C = 0;
surfaces.left.D = 0;
surfaces.left.normal = [1, 0, 0];  % 指向房间内部
surfaces.left.center = [0, room_width/2, room_height/2];
surfaces.left.bounds = {[0, 0], [0, room_width], [0, room_height]};
surfaces.left.name = 'Left Wall';
surfaces.left.reflectivity = 0.8;  % 墙壁反射系数

% 6. 右墙 (x = room_length)
surfaces.right.A = 1; surfaces.right.B = 0; surfaces.right.C = 0;
surfaces.right.D = -room_length;
surfaces.right.normal = [-1, 0, 0];  % 指向房间内部
surfaces.right.center = [room_length, room_width/2, room_height/2];
surfaces.right.bounds = {[room_length, room_length], [0, room_width], [0, room_height]};
surfaces.right.name = 'Right Wall';
surfaces.right.reflectivity = 0.8;  % 墙壁反射系数





%% 批量测试带反射的光线追踪
fprintf('\n=== 带反射的光线追踪测试 ===\n');
num_test_rays = 10000;
max_bounces = 10;

receiver_hits_direct = 0;      % 直接命中
receiver_hits_bounce1 = 0;     % 1次反射后命中
receiver_hits_bounce2 = 0;     % 2次反射后命中
receiver_hits_bounce3plus = 0; % 3次及以上反射后命中
receiver_hits_total = 0;       % 总命中

bounce_statistics = zeros(max_bounces+1, 1);  % 统计各反射次数的光线数
distance_statistics = [];

% 存储命中接收机的光线路径
hit_paths = cell(0);

for ray_idx = 1:num_test_rays
    if mod(ray_idx, 100) == 0
        fprintf('处理第 %d/%d 条光线...\n', ray_idx, num_test_rays);
    end
    
    % Lambertian采样初始方向
    theta = acos(rand^(1/(mode+1)));
    phi = 2*pi*rand;
    [x, y, z] = sph2cart(phi, theta, 1);
    initial_direction = [x, y, -z];
    initial_direction = initial_direction / norm(initial_direction);
    
    % 追踪光线（包含反射）
    [hit_receiver, total_distance, path_points, bounce_count] = ...
        Reflect_traceRayWithBounces(light_position, initial_direction, surfaces, ...
                                   receiver_position, receiver_normal, receiver_fov, receiver_area, max_bounces);
    
    % 记录统计数据
    bounce_statistics(bounce_count+1) = bounce_statistics(bounce_count+1) + 1;
    
    if hit_receiver
        receiver_hits_total = receiver_hits_total + 1;
        distance_statistics(end+1) = total_distance;
        
        % 根据反射次数分类
        if bounce_count == 0
            receiver_hits_direct = receiver_hits_direct + 1;
        elseif bounce_count == 1
            receiver_hits_bounce1 = receiver_hits_bounce1 + 1;
        elseif bounce_count == 2
            receiver_hits_bounce2 = receiver_hits_bounce2 + 1;
        else
            receiver_hits_bounce3plus = receiver_hits_bounce3plus + 1;
        end
        
        % 存储路径（最多存储5条）
        if length(hit_paths) < 5
            hit_paths{end+1} = struct('path', path_points, 'bounces', bounce_count);
        end
    end
end

%% 显示统计结果
fprintf('\n=== 带反射的光线追踪统计结果 ===\n');
fprintf('总测试光线数: %d\n', num_test_rays);
fprintf('总接收机命中数: %d (%.4f%%)\n', receiver_hits_total, receiver_hits_total/num_test_rays*100);
fprintf('\n按反射次数分类:\n');
fprintf('  直接命中: %d (%.4f%%)\n', receiver_hits_direct, receiver_hits_direct/num_test_rays*100);
fprintf('  1次反射后命中: %d (%.4f%%)\n', receiver_hits_bounce1, receiver_hits_bounce1/num_test_rays*100);
fprintf('  2次反射后命中: %d (%.4f%%)\n', receiver_hits_bounce2, receiver_hits_bounce2/num_test_rays*100);
fprintf('  3次及以上反射后命中: %d (%.4f%%)\n', receiver_hits_bounce3plus, receiver_hits_bounce3plus/num_test_rays*100);

fprintf('\n反射次数分布:\n');
for i = 0:max_bounces
    count = bounce_statistics(i+1);
    percentage = count/num_test_rays*100;
    if i < max_bounces
        fprintf('  %d次反射: %d (%.2f%%)\n', i, count, percentage);
    else
        fprintf('  %d+次反射: %d (%.2f%%)\n', i, count, percentage);
    end
end

if ~isempty(distance_statistics)
    fprintf('\n命中接收机的光线传播距离统计:\n');
    fprintf('  平均距离: %.3f米\n', mean(distance_statistics));
    fprintf('  最短距离: %.3f米\n', min(distance_statistics));
    fprintf('  最长距离: %.3f米\n', max(distance_statistics));
    fprintf('  距离标准差: %.3f米\n', std(distance_statistics));
end

%% 可视化带反射的光线路径
figure('Position', [100, 100, 1400, 600]);

% 子图1：三维路径可视化
subplot(1, 2, 1);
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('带反射的光线路径（命中接收机）');
view(45, 30);

% 绘制房间框架
vertices = [0 0 0; room_length 0 0; room_length room_width 0; 0 room_width 0; ...
            0 0 room_height; room_length 0 room_height; ...
            room_length room_width room_height; 0 room_width room_height];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Vertices', vertices, 'Faces', faces, ...
      'FaceColor', [0.8 0.9 1], 'FaceAlpha', 0.1, 'EdgeColor', 'k');

% 绘制光源
plot3(light_position(1), light_position(2), light_position(3), 'ro', ...
      'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', '光源');

% 绘制接收机圆形区域
theta_circle = linspace(0, 2*pi, 100);
x_circle = receiver_radius * cos(theta_circle) + receiver_position(1);
y_circle = receiver_radius * sin(theta_circle) + receiver_position(2);
z_circle = zeros(size(theta_circle)) + 0.01;
fill3(x_circle, y_circle, z_circle, [0.5, 1, 0.5], 'FaceAlpha', 0.5, ...
      'EdgeColor', 'g', 'LineWidth', 2, 'DisplayName', '接收机');
plot3(receiver_position(1), receiver_position(2), receiver_position(3), ...
      'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

% 绘制命中接收机的光线路径（最多5条）
colors = {'r', 'g', 'b', 'm', 'c'};
for i = 1:min(length(hit_paths), 5)
    path = hit_paths{i}.path;
    bounces = hit_paths{i}.bounces;
    
    % 绘制路径线
    plot3(path(:,1), path(:,2), path(:,3), ...
          [colors{i} '-'], 'LineWidth', 1.5, ...
          'DisplayName', sprintf('路径%d (%d次反射)', i, bounces));
    
    % 绘制路径点
    plot3(path(:,1), path(:,2), path(:,3), ...
          [colors{i} 'o'], 'MarkerSize', 4, 'MarkerFaceColor', colors{i});
    
    % 标注反射点
    for j = 2:size(path,1)-1
        text(path(j,1), path(j,2), path(j,3), ...
             sprintf('%d', j-1), 'FontSize', 8, 'Color', colors{i});
    end
end

legend('Location', 'best');
xlim([-0.5, room_length+0.5]);
ylim([-0.5, room_width+0.5]);
zlim([-0.5, room_height+0.5]);

% 子图2：统计图表
subplot(1, 2, 2);

% 反射次数分布条形图
subplot(2, 2, 2);
bar(0:max_bounces, bounce_statistics, 'FaceColor', [0.2 0.6 0.8]);
xlabel('反射次数');
ylabel('光线数量');
title('光线反射次数分布');
grid on;

% 命中接收机统计（按反射次数）
subplot(2, 2, 4);
hit_categories = {'直接命中', '1次反射', '2次反射', '3+次反射'};
hit_counts = [receiver_hits_direct, receiver_hits_bounce1, receiver_hits_bounce2, receiver_hits_bounce3plus];
bar(hit_counts, 'FaceColor', [0.8 0.4 0.2]);
set(gca, 'XTickLabel', hit_categories);
ylabel('命中数');
title('接收机命中按反射次数分类');
grid on;

for i = 1:length(hit_counts)
    text(i, hit_counts(i)+max(hit_counts)*0.02, ...
         sprintf('%d', hit_counts(i)), 'HorizontalAlignment', 'center');
end

fprintf('\n=== Lambertian反射采样验证完成 ===\n');
fprintf('验证了以下特性:\n');
fprintf('1. 墙壁和地板反射系数: 0.8\n');
fprintf('2. 天花板反射系数: 0.3\n');
fprintf('3. 最大反射次数: %d\n', max_bounces);
fprintf('4. 光线命中接收机后被吸收\n');
fprintf('5. 使用Lambertian分布进行反射采样\n');

%% 保留原有的光线追踪函数（不需要修改）
function [hit_surface, hit_point, hit_distance, reached_receiver] = Receiver_checkRayHitReceiver(start_point, direction, surfaces, receiver_position, receiver_normal, receiver_fov, receiver_area)
    % 计算光线与所有表面的交点，并检查是否到达接收机
    surfaces_list = fieldnames(surfaces);
    min_distance = inf;
    hit_surface = '';
    hit_point = [];
    reached_receiver = false;
    
    for i = 1:length(surfaces_list)
        surface_name = surfaces_list{i};
        surface = surfaces.(surface_name);
        
        denominator = surface.A * direction(1) + surface.B * direction(2) + surface.C * direction(3);
        
        if abs(denominator) > 1e-10
            t = -(surface.A * start_point(1) + surface.B * start_point(2) + surface.C * start_point(3) + surface.D) / denominator;
            
            if t > 1e-6
                intersection_point = start_point + t * direction;
                
                if checkPointInBounds(intersection_point, surface.bounds)
                    if t < min_distance
                        min_distance = t;
                        hit_surface = surface_name;
                        hit_point = intersection_point;
                        
                        % 检查是否命中接收机
                        vector_to_receiver = receiver_position - hit_point;
                        distance_to_receiver = norm(vector_to_receiver);
                        
                        if distance_to_receiver <= sqrt(receiver_area/pi) + 1e-6
                            incident_vector = hit_point - receiver_position;
                            incident_direction = incident_vector / distance_to_receiver;
                            cos_theta = dot(incident_direction, receiver_normal);
                            cos_theta = min(max(cos_theta, -1), 1);
                            incident_angle_deg = rad2deg(acos(cos_theta));
                            
                            if incident_angle_deg <= receiver_fov/2
                                reached_receiver = true;
                            end
                        end
                    end
                end
            end
        end
    end
    
    hit_distance = min_distance;
end

%% 辅助函数：检查点是否在表面边界内
function in_bounds = checkPointInBounds(point, bounds)
    tolerance = 1e-6;
    in_bounds = (point(1) >= bounds{1}(1) - tolerance && point(1) <= bounds{1}(2) + tolerance) && ...
                (point(2) >= bounds{2}(1) - tolerance && point(2) <= bounds{2}(2) + tolerance) && ...
                (point(3) >= bounds{3}(1) - tolerance && point(3) <= bounds{3}(2) + tolerance);
end

%% 辅助函数：简单光线追踪（不含接收机检测）
function [hit_surface, hit_point, hit_distance] = LightTrace_rayIntersectRoom(start_point, direction, surfaces)
    surfaces_list = fieldnames(surfaces);
    min_distance = inf;
    hit_surface = '';
    hit_point = [];
    
    for i = 1:length(surfaces_list)
        surface_name = surfaces_list{i};
        surface = surfaces.(surface_name);
        
        denominator = surface.A * direction(1) + surface.B * direction(2) + surface.C * direction(3);
        
        if abs(denominator) > 1e-10
            t = -(surface.A * start_point(1) + surface.B * start_point(2) + surface.C * start_point(3) + surface.D) / denominator;
            
            if t > 1e-6
                intersection_point = start_point + t * direction;
                
                if checkPointInBounds(intersection_point, surface.bounds)
                    if t < min_distance
                        min_distance = t;
                        hit_surface = surface_name;
                        hit_point = intersection_point;
                    end
                end
            end
        end
    end
    
    hit_distance = min_distance;
end