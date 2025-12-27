%% Lambertian反射采样函数
function new_direction = Reflect_sampleLambertianDirection(surface_normal)
    % 根据表面法向量采样Lambertian反射方向
    % 输入：
    %   surface_normal: 表面法向量（单位向量）
    % 输出：
    %   new_direction: 新的反射方向（单位向量）
    
    % 归一化法向量
    normal = surface_normal(:)' / norm(surface_normal);
    
    % 生成符合余弦分布的随机方向
    u1 = rand();
    u2 = rand();
    
    % 采样球坐标
    theta = asin(sqrt(u1));  % 俯仰角，PDF: cosθ
    phi = 2*pi*u2;           % 方位角，均匀分布
    
    % 在局部坐标系中生成方向
    if abs(normal(3)) > 0.999999
        % 法向量接近z轴
        tangent = [1, 0, 0];
        bitangent = [0, 1, 0];
    else
        up = [0, 0, 1];
        tangent = cross(normal, up);
        tangent = tangent / norm(tangent);
        bitangent = cross(normal, tangent);
    end
    
    % 构造方向向量
    sin_theta = sin(theta);
    new_direction = sin_theta*cos(phi)*tangent + ...
                    sin_theta*sin(phi)*bitangent + ...
                    cos(theta)*normal;
    
    % 归一化确保单位向量
    new_direction = new_direction / norm(new_direction);
end