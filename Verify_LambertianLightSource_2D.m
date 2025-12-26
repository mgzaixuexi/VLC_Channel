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
fprintf('开始Lambertian方向采样验证...\n');
fprintf('模式数 m = %d\n', mode);
fprintf('采样数量 N = %d\n', N_samples);

%% 生成Lambertian采样数据
fprintf('生成采样数据中...\n');
thetas = zeros(1, N_samples);

% 基于文档Eq.(4)的采样算法
for i = 1:N_samples
    theta = acos(rand^(1/(mode+1)));  % 核心采样公式
    thetas(i) = theta;
end

fprintf('采样完成！平均角度: %.2f度\n', mean(thetas)*180/pi);
%% 计算理论分布曲线
theta_range = linspace(0, pi/2, 1000); % θ角范围0-90度

% 文档中的理论概率密度函数：p(θ) = (m+1)cos^m(θ)sin(θ)
theoretical_pdf = (mode+1) * cos(theta_range).^mode .* sin(theta_range);

% 计算累积分布函数
theoretical_cdf = 1 - cos(theta_range).^(mode+1);
%% 实验数据统计分析
[counts, bin_edges] = histcounts(thetas, 50, 'Normalization', 'pdf');
bin_centers = (bin_edges(1:end-1) + bin_edges(2:end)) / 2;

% 计算实验累积分布
[ecdf_y, ecdf_x] = ecdf(thetas);
%牛魔的这里x和y弄错了，我还以为哪里有问题。
% 统计检验
[h, p_value] = kstest(thetas, [theta_range', theoretical_cdf']);
%% 创建综合可视化图表
figure('Position', [100, 100, 1400, 900]);

subplot(2,3,1);
% 理论曲线绘制
plot(theta_range*180/pi, theoretical_pdf, 'r-', 'LineWidth', 3, ...
     'DisplayName', '理论分布');
hold on;
% 实验直方图
bar(bin_centers*180/pi, counts, 'FaceAlpha', 0.6, 'FaceColor', [0.2 0.6 0.8], ...
    'EdgeColor', 'none', 'DisplayName', '实验采样');
xlabel('角度 θ (度)');
ylabel('概率密度');
title('Lambertian分布验证 (PDF)');
legend('show', 'Location', 'northeast');
grid on;
set(gca, 'FontSize', 12);
subplot(2,3,2);
plot(theta_range*180/pi, theoretical_cdf, 'r-', 'LineWidth', 3, ...
     'DisplayName', '理论CDF');
hold on;
stairs(ecdf_x*180/pi, ecdf_y, 'b-', 'LineWidth', 2, ...
        'DisplayName', '实验CDF');
xlabel('角度 θ (度)');
ylabel('累积概率');
title('累积分布函数验证');
legend('show', 'Location', 'southeast');
grid on;
set(gca, 'FontSize', 12);
subplot(2,3,3);
thetas_deg = thetas * 180/pi;
histogram(thetas_deg, 50, 'Normalization', 'pdf', ...
          'FaceColor', [0.8 0.4 0.2], 'FaceAlpha', 0.7);
xlabel('角度 θ (度)');
ylabel('概率密度');
title('角度分布直方图');
grid on;
set(gca, 'FontSize', 12);

% 添加统计信息
mean_angle = mean(thetas_deg);
std_angle = std(thetas_deg);
text(0.6*max(xlim), 0.8*max(ylim), ...
     sprintf('均值: %.2f°\n标准差: %.2f°', mean_angle, std_angle), ...
     'FontSize', 11, 'BackgroundColor', 'white');
subplot(2,3,4);
% 测试不同模式数的分布
modes = [1, 2, 5, 10];
colors = lines(length(modes));
hold on;

for i = 1:length(modes)
    m = modes(i);
    theta_test = linspace(0, pi/2, 100);
    pdf_test = (m+1) * cos(theta_test).^m .* sin(theta_test);
    plot(theta_test*180/pi, pdf_test, 'LineWidth', 2, ...
         'Color', colors(i,:), 'DisplayName', sprintf('m=%d', m));
end

xlabel('角度 θ (度)');
ylabel('概率密度');
title('不同模式数对比');
legend('show', 'Location', 'northeast');
grid on;
set(gca, 'FontSize', 12);

%% 修正后的子图5和子图6创建代码
subplot(2,3,5);
polarax = polaraxes('Position', [0.45, 0.12, 0.15, 0.3]);
polarhistogram(polarax, thetas, 25, 'FaceColor', [0.2 0.8 0.5], 'FaceAlpha', 0.7);
title(polarax, '极坐标角度分布');
polarax.FontSize = 10; % 正确设置极坐标轴字体