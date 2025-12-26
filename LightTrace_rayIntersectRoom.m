%% 光线追踪函数
function [hit_surface, hit_point, hit_distance] = LightTrace_rayIntersectRoom(start_point, direction, surfaces)
    % 计算光线与所有表面的交点
    % 输入：
    %   start_point: 光线起点 [x, y, z]
    %   direction: 光线方向向量 [dx, dy, dz]
    %   surfaces: 房间表面结构体
    % 输出：
    %   hit_surface: 命中的表面名称
    %   hit_point: 命中点坐标 [x, y, z]
    %   hit_distance: 光线行进距离
    
    surfaces_list = fieldnames(surfaces);%用于获取结构体的字段名称
    min_distance = inf;
    hit_surface = '';
    hit_point = [];
    
    for i = 1:length(surfaces_list)
        surface_name = surfaces_list{i};%
        surface = surfaces.(surface_name);
        
        % 计算光线参数t（平面方程: A*x + B*y + C*z + D = 0）
        % 光线方程: P = start_point + t * direction
        denominator = surface.A * direction(1) + ... %denominator分母
                     surface.B * direction(2) + ...
                     surface.C * direction(3);
        
        if abs(denominator) > 1e-10  % 避免与平面平行 
            t = -(surface.A * start_point(1) + ...
                  surface.B * start_point(2) + ...
                  surface.C * start_point(3) + surface.D) / denominator;
            
            if t > 1e-6  % 代码避免光线向后的交点,如果注释掉会有什么bug呢？懂了，会有反向的bug。
                % 计算交点坐标
                intersection_point = start_point + t * direction;
                
                % 检查交点是否在表面边界内
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

%% 辅助函数：检查点是否在表面边界内
function in_bounds = checkPointInBounds(point, bounds)
    % bounds: {[x_min, x_max], [y_min, y_max], [z_min, z_max]}
    tolerance = 1e-6;  % 容差
    
    in_bounds = (point(1) >= bounds{1}(1) - tolerance && point(1) <= bounds{1}(2) + tolerance) && ...
                (point(2) >= bounds{2}(1) - tolerance && point(2) <= bounds{2}(2) + tolerance) && ...
                (point(3) >= bounds{3}(1) - tolerance && point(3) <= bounds{3}(2) + tolerance);
end