%% 修改三：添加接收机命中检测函数
function [hit_surface, hit_point, hit_distance, reached_receiver] = Receiver_checkRayHitReceiver(start_point, direction, surfaces, receiver_position, receiver_normal, receiver_fov, receiver_area)
    % 计算光线与所有表面的交点，并检查是否到达接收机
    % 输入：
    %   start_point: 光线起点 [x, y, z]
    %   direction: 光线方向向量 [dx, dy, dz]
    %   surfaces: 房间表面结构体
    %   receiver_position: 接收机位置 [x, y, z]
    %   receiver_normal: 接收机法向量
    %   receiver_fov: 接收机视场角 (度)
    %   receiver_area: 接收机面积 (m²)
    % 输出：
    %   hit_surface: 命中的表面名称
    %   hit_point: 命中点坐标 [x, y, z]
    %   hit_distance: 光线行进距离
    %   reached_receiver: 是否到达接收机 (true/false)
    
    surfaces_list = fieldnames(surfaces);
    min_distance = inf;
    hit_surface = '';
    hit_point = [];
    reached_receiver = false;
    
    for i = 1:length(surfaces_list)
        surface_name = surfaces_list{i};
        surface = surfaces.(surface_name);
        
        % 计算光线参数t（平面方程: A*x + B*y + C*z + D = 0）
        denominator = surface.A * direction(1) + surface.B * direction(2) + surface.C * direction(3);
        
        if abs(denominator) > 1e-10
            t = -(surface.A * start_point(1) + surface.B * start_point(2) + surface.C * start_point(3) + surface.D) / denominator;
            
            if t > 1e-6
                intersection_point = start_point + t * direction;
                
                % 检查交点是否在表面边界内
                if checkPointInBounds(intersection_point, surface.bounds)
                    if t < min_distance
                        min_distance = t;
                        hit_surface = surface_name;
                        hit_point = intersection_point;
                        
                        % 判断是否到达接收机
                        vector_to_receiver = receiver_position - hit_point;
                        distance_to_receiver = norm(vector_to_receiver);
                        
                        % 计算命中点到接收机的距离
                        if distance_to_receiver < sqrt(receiver_area/pi) - 1e-6 %1e-6瞎写的，假设为圆形，面积为pi * 半径的平方
                            reached_receiver = true;
                        
                        elseif distance_to_receiver  <= sqrt(receiver_area/pi) + 1e-6
                            % 计算入射角
                            dot_product = dot(vector_to_receiver, receiver_normal) / (distance_to_receiver * norm(receiver_normal));
                            % 处理数值误差
                            dot_product = min(max(dot_product, -1), 1);%由于浮点数误差，dot_product可能略小于 -1 或略大于 1
                            incident_angle = acos(dot_product);
                            
                            % 转换为度
                            incident_angle_deg = rad2deg(incident_angle);
                            
                            % 检查是否在视场角内
                            if incident_angle_deg <= receiver_fov/2
                                reached_receiver = true;
                            else
                                reached_receiver = false;
                            end
                        else
                            reached_receiver = false;
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
    % bounds: {[x_min, x_max], [y_min, y_max], [z_min, z_max]}
    tolerance = 1e-6;  % 容差
    
    in_bounds = (point(1) >= bounds{1}(1) - tolerance && point(1) <= bounds{1}(2) + tolerance) && ...
                (point(2) >= bounds{2}(1) - tolerance && point(2) <= bounds{2}(2) + tolerance) && ...
                (point(3) >= bounds{3}(1) - tolerance && point(3) <= bounds{3}(2) + tolerance);
end