%% 验证采样分布
N = 100000;
mode = 1;
thetas = zeros(1, N);
for i = 1:N
    theta = acos(rand^(1/(mode+1)));
    phi = 2*pi*rand;
    thetas(i) = theta;
end
%已显示cos^m(θ)sin(θ)分布，Lambertian二维方向采样验证成功
%现在请完成三维方向采样验证
    phi = 2*pi*rand;
    
    % 转换为笛卡尔坐标
    [x, y, z] = sph2cart(phi, theta, 1);
    sample_dir = [x, y, z];

%% %% Lambertian方向采样验证主程序
clear; clc; close all;

% 设置验证参数（基于文档Table 1）
mode = 1;           % 光源模式数m=1（文档Table 1）
N_samples = 100000; % 采样数量
fprintf('开始Lambertian三维方向采样验证...\n');
fprintf('模式数 m = %d\n', mode);
fprintf('采样数量 N = %d\n', N_samples);

%% 生成Lambertian采样数据
% 修改内容：生成三维方向采样数据
fprintf('生成三维采样数据中...\n');
thetas = zeros(1, N_samples);% 极角θ
directions = zeros(N_samples, 3);  % 三维方向向量
phis = zeros(1, N_samples);       % 方位角φ

% 基于文档Eq.(4)的采样算法
for i = 1:N_samples
    % 极角采样（保持不变）
    theta = acos(rand^(1/(mode+1)));  % 核心采样公式
    phi = 2*pi*rand;                  % 方位角均匀采样
    
    % 球坐标转笛卡尔坐标
    [x, y, z] = sph2cart(phi, theta, 1);  
    
    % 存储采样结果
    directions(i, :) = [x, y, z];
    thetas(i) = theta;
    phis(i) = phi;
end
fprintf('三维采样完成！平均极角: %.2f度\n', mean(thetas)*180/pi);

%% 计算理论分布曲线
theta_range = linspace(0, pi/2, 1000); % θ角范围0-90度
phi_range = linspace(0, 2*pi, 1000);   % φ角范围0-360度


% 文档中的理论概率密度函数：p(θ) = (m+1)cos^m(θ)sin(θ)
theoretical_pdf_theta = (mode+1) * cos(theta_range).^mode .* sin(theta_range);
theoretical_pdf_phi = ones(size(phi_range)) / (2*pi);  % φ均匀分布


% 计算累积分布函数
theoretical_cdf_theta = 1 - cos(theta_range).^(mode+1);
theoretical_cdf_phi = phi_range / (2*pi);
%% 实验数据统计分析
[counts_theta, bin_edges_theta] = histcounts(thetas, 50, 'Normalization', 'pdf');
bin_centers_theta = (bin_edges_theta(1:end-1) + bin_edges_theta(2:end)) / 2;

[counts_phi, bin_edges_phi] = histcounts(phis, 50, 'Normalization', 'pdf');
bin_centers_phi = (bin_edges_phi(1:end-1) + bin_edges_phi(2:end)) / 2;

% 计算实验累积分布
[ecdf_y_theta, ecdf_x_theta] = ecdf(thetas);
[ecdf_y_phi, ecdf_x_phi] = ecdf(phis);
%牛魔的这里x和y弄错了，我还以为哪里有问题。
% 统计检验
[h_theta, p_value_theta] = kstest(thetas, [theta_range', theoretical_cdf_theta']);
[h_phi, p_value_phi] = kstest(phis, [phi_range', theoretical_cdf_phi']);
%% 修改内容：创建三维方向采样验证图表
figure('Position', [100, 100, 1600, 1200]);

% 子图1：极角θ分布验证（保持原二维验证）
subplot(3,4,1);
plot(theta_range*180/pi, theoretical_pdf_theta, 'r-', 'LineWidth', 3, ...
     'DisplayName', '理论分布');
hold on;
bar(bin_centers_theta*180/pi, counts_theta, 'FaceAlpha', 0.6, 'FaceColor', [0.2 0.6 0.8], ...
    'EdgeColor', 'none', 'DisplayName', '实验采样');
xlabel('极角 θ (度)'); ylabel('概率密度');
title('极角分布验证 (PDF)'); legend('show'); grid on; set(gca, 'FontSize', 10);

% 子图2：方位角φ分布验证（新增）
subplot(3,4,2);
plot(phi_range*180/pi, theoretical_pdf_phi, 'r-', 'LineWidth', 3, ...
     'DisplayName', '理论分布');
hold on;
bar(bin_centers_phi*180/pi, counts_phi, 'FaceAlpha', 0.6, 'FaceColor', [0.8 0.4 0.2], ...
    'EdgeColor', 'none', 'DisplayName', '实验采样');
xlabel('方位角 φ (度)'); ylabel('概率密度');
title('方位角分布验证 (PDF)'); legend('show'); grid on; set(gca, 'FontSize', 10);

% 子图3：极角CDF验证
subplot(3,4,3);
plot(theta_range*180/pi, theoretical_cdf_theta, 'r-', 'LineWidth', 3);
hold on;
stairs(ecdf_x_theta*180/pi, ecdf_y_theta, 'b-', 'LineWidth', 2);
xlabel('极角 θ (度)'); ylabel('累积概率'); title('极角CDF验证'); grid on;

% 子图4：方位角CDF验证
subplot(3,4,4);
plot(phi_range*180/pi, theoretical_cdf_phi, 'r-', 'LineWidth', 3);
hold on;
stairs(ecdf_x_phi*180/pi, ecdf_y_phi, 'b-', 'LineWidth', 2);
xlabel('方位角 φ (度)'); ylabel('累积概率'); title('方位角CDF验证'); grid on;


%%
% 子图5：三维方向分布散点图（新增）
subplot(3,4,5);
% 随机选择1000个点进行可视化避免过度拥挤
idx = randperm(N_samples, min(1000, N_samples));
scatter3(directions(idx,1), directions(idx,2), directions(idx,3), 10, thetas(idx), 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('三维方向分布');
colorbar; colormap(jet); grid on; axis equal;

% 子图6：方向向量分量分布（新增）
subplot(3,4,6);
histogram2(directions(:,1), directions(:,2), 50, 'DisplayStyle','tile', 'FaceAlpha',0.8);
xlabel('X分量'); ylabel('Y分量'); title('XY平面方向密度'); colorbar;

% 子图7：极坐标显示
subplot(3,4,7);
polarax = polaraxes('Position', [0.56, 0.38, 0.14, 0.28]);
polarhistogram(polarax, thetas, 25, 'FaceColor', [0.2 0.8 0.5], 'FaceAlpha', 0.7);
title(polarax, '极角分布'); polarax.FontSize = 8;

% 子图8：方位角分布极坐标
subplot(3,4,8);
polarax2 = polaraxes('Position', [0.76, 0.38, 0.14, 0.28]);
polarhistogram(polarax2, phis, 25, 'FaceColor', [0.8 0.5 0.2], 'FaceAlpha', 0.7);
title(polarax2, '方位角分布'); polarax2.FontSize = 8;