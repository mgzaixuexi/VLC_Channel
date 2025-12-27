%% 光线追踪函数（支持反射）
function [hit_receiver, total_distance, path_points, bounce_count] = Reflect_traceRayWithBounces(start_point, direction, surfaces, receiver_position, receiver_normal, receiver_fov, receiver_area, max_bounces)
    % 追踪光线，考虑多次反射
    % 输入：
    %   start_point: 起始点 [x, y, z]
    %   direction: 初始方向 [dx, dy, dz]
    %   surfaces: 房间表面结构体
    %   receiver_...: 接收机参数
    %   max_bounces: 最大反射次数
    % 输出：
    %   hit_receiver: 是否命中接收机
    %   total_distance: 总传播距离
    %   path_points: 路径点集合
    %   bounce_count: 实际反射次数
    
    hit_receiver = false;
    total_distance = 0;
    path_points = start_point;
    bounce_count = 0;
    current_point = start_point;
    current_direction = direction / norm(direction);
    
    while bounce_count <= max_bounces
        % 检查光线与表面的交点
        [hit_surface, hit_point, hit_distance, reached_receiver] = Receiver_checkRayHitReceiver(current_point, current_direction, surfaces, receiver_position, receiver_normal, receiver_fov, receiver_area);
        
        if isempty(hit_surface)
            % 未命中任何表面，光线逃逸
            break;
        end
        
        % 更新总距离和路径点
        total_distance = total_distance + hit_distance;
        path_points = [path_points; hit_point];
        
        % 检查是否命中接收机
        if reached_receiver
            hit_receiver = true;
            break;
        end
        
        % 获取表面属性
        surface = surfaces.(hit_surface);
        reflectivity = surface.reflectivity;
        
        % 根据反射系数决定是否反射
        if rand() > reflectivity
            % 被吸收
            break;
        end
        
        % 执行Lambertian反射
        current_point = hit_point;
        current_direction = Reflect_sampleLambertianDirection(surface.normal);
        bounce_count = bounce_count + 1;
    end
end