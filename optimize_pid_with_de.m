% 文件名：optimize_pid_with_adaptive_de.m
% 功能：使用自适应差分进化算法优化火箭内外环PID参数
% 目标：最小化综合适应度（综合衡量跟踪误差、超调和控制平稳性）
% 改进：实现了自适应控制参数策略，参考JADE算法思想

%% 初始化环境
clear; clc; close all;
set(0, 'DefaultAxesFontName', 'SimHei');
set(0, 'DefaultTextFontName', 'SimHei');
set(0, 'DefaultFigureColor', 'w');

%% DE参数配置
nVars = 6; % 优化变量数 [Kp_outer, Ki_outer, Kd_outer, Kp_inner, Ki_inner, Kd_inner]
lb = [0, 0, 0, 0, 0, 0]; % 下界
ub = [10, 10, 10, 10, 10, 10]; % 上界

% DE核心参数
NP = 50; % 种群规模
F_init = 0.8; % 初始变异因子
CR_init = 0.9; % 初始交叉率
maxGen = 100; % 最大迭代次数

% 自适应参数设置
c = 0.1; % 自适应参数学习率
p = 0.1; % 优秀个体比例 (用于更新F和CR)

%% 初始化种群和适应度记录
% 初始化种群，每个个体的每个变量在上下界范围内随机生成
population = repmat(lb, NP, 1) + rand(NP, nVars) .* repmat(ub - lb, NP, 1);
fitness = inf(NP, 1); % 初始适应度设为无穷大，方便后续更新
best_fitness = zeros(maxGen, 1); % 每代最优适应度记录
mean_fitness = zeros(maxGen, 1); % 每代平均适应度记录
best_params = zeros(maxGen, nVars); % 每代最优参数记录
F_history = zeros(maxGen, 1); % 记录F的变化历史
CR_history = zeros(maxGen, 1); % 记录CR的变化历史
diversity_history = zeros(maxGen, 1); % 种群多样性历史

% 初始化自适应参数
F_values = F_init * ones(NP, 1); % 每个个体的F值
CR_values = CR_init * ones(NP, 1); % 每个个体的CR值
mu_F = F_init; % F值的均值
mu_CR = CR_init; % CR值的均值

% 成功的F和CR值记录容器
S_F = [];
S_CR = [];

% 并行计算初始适应度
parfor i = 1:NP
    fitness(i) = rocket_simulation_fitness(population(i, :));
end

%% 创建动态监控图形窗口
figure('Name', '自适应DE优化进程实时监控', 'Position', [100, 100, 1800, 800], 'Color', 'w');

% 子图1：综合适应度变化曲线
subplot(2, 3, 1);
h_plot_fitness = plot(nan, nan, 'b-', 'LineWidth', 1.5);
hold on;
h_plot_mean_fitness = plot(nan, nan, 'r--', 'LineWidth', 1);
title('适应度进化曲线');
xlabel('迭代次数'); ylabel('适应度值');
legend('最优适应度', '平均适应度', 'Location', 'best');
xlim([0 maxGen]); grid on;

% 子图2：当前代适应度分布
subplot(2, 3, 2);
h_bar_fitness = bar(1:NP, fitness, 'FaceColor', [0 0.4470 0.7410]);
title('当前代适应度分布');
xlabel('个体序号'); ylabel('适应度值');
ylim([0 max(fitness) * 1.1]); grid on;

% 子图3：最优PID参数迭代过程
subplot(2, 3, 3);
param_labels = {'Kp_{out}', 'Ki_{out}', 'Kd_{out}', 'Kp_{in}', 'Ki_{in}', 'Kd_{in}'};
h_plot_params = plot(nan(maxGen, nVars), 'LineWidth', 1.5);
legend(param_labels, 'Location', 'best');
title('最优PID参数进化过程');
xlabel('迭代次数'); ylabel('参数值');
xlim([0 maxGen]); ylim([0 max(ub) * 1.1]); grid on;

% 子图4：自适应参数F和CR变化
subplot(2, 3, 4);
h_plot_F = plot(nan, nan, 'g-', 'LineWidth', 1.5);
hold on;
h_plot_CR = plot(nan, nan, 'm-', 'LineWidth', 1.5);
title('自适应参数变化');
xlabel('迭代次数'); ylabel('参数值');
legend('变异因子F', '交叉率CR', 'Location', 'best');
xlim([0 maxGen]); ylim([0 1.1]); grid on;

% 子图5：种群多样性变化
subplot(2, 3, 5);
h_plot_diversity = plot(nan, nan, 'k-', 'LineWidth', 1.5);
title('种群多样性变化');
xlabel('迭代次数'); ylabel('多样性指标');
xlim([0 maxGen]); grid on;

% 子图6：F和CR的分布
subplot(2, 3, 6);
h_scatter_F_CR = scatter(F_values, CR_values, 30, 'filled');
title('F和CR值分布');
xlabel('变异因子F'); ylabel('交叉率CR');
xlim([0 1]); ylim([0 1]); grid on;

%% DE算法主迭代
disp('开始自适应差分进化算法优化PID参数...');
tic; % 开始计时

for gen = 1:maxGen
    % 清空成功F和CR记录
    S_F = [];
    S_CR = [];

    % 生成新的F和CR值
    for i = 1:NP
        % 根据高斯分布生成新的F值，并限制在[0.1, 1]范围内
        F_values(i) = normrnd(mu_F, 0.1);
        F_values(i) = max(0.1, min(1, F_values(i)));

        % 根据高斯分布生成新的CR值，并限制在[0, 1]范围内
        CR_values(i) = normrnd(mu_CR, 0.1);
        CR_values(i) = max(0, min(1, CR_values(i)));
    end

    % 准备新一代种群
    new_population = population;
    new_fitness = fitness;

    % 并行处理每个个体
    parfor i = 1:NP
        % 获取当前个体的F和CR值
        F_i = F_values(i);
        CR_i = CR_values(i);

        % 差分变异：DE/rand/1策略
        candidates = 1:NP;
        candidates(i) = [];
        r = datasample(candidates, 3, 'Replace', false);
        mutant = population(r(1), :) + F_i * (population(r(2), :) - population(r(3), :));

        % 边界反射处理
        mutant = min(max(mutant, lb), ub);

        % 二项式交叉
        cross_points = rand(1, nVars) < CR_i;
        % 确保至少有一个维度发生交叉
        if ~any(cross_points)
            cross_points(randi(nVars)) = true;
        end

        trial = population(i, :);
        trial(cross_points) = mutant(cross_points);

        % 评估新个体的综合适应度
        trial_fitness = rocket_simulation_fitness(trial);

        % 贪婪选择
        if trial_fitness < fitness(i)
            new_population(i, :) = trial;
            new_fitness(i) = trial_fitness;

            % 记录成功的F和CR值
            S_F = [S_F; F_i];
            S_CR = [S_CR; CR_i];
        end

    end

    % 更新种群和适应度
    population = new_population;
    fitness = new_fitness;

    % 更新自适应参数
    if ~isempty(S_F) && ~isempty(S_CR)
        % 计算成功F值的Lehmer平均数
        if length(S_F) > 1
            mu_F = (1 - c) * mu_F + c * sum(S_F .^ 2) / sum(S_F);
        else
            mu_F = (1 - c) * mu_F + c * S_F(1);
        end

        % 计算成功CR值的算术平均数
        mu_CR = (1 - c) * mu_CR + c * mean(S_CR);
    end

    % 记录参数历史
    F_history(gen) = mu_F;
    CR_history(gen) = mu_CR;

    % 计算并记录种群多样性 (使用参数空间的标准差作为多样性指标)
    pop_std = std(population);
    diversity = mean(pop_std ./ (ub - lb)); % 归一化多样性
    diversity_history(gen) = diversity;

    % 记录当前代最优解和平均适应度
    [best_fitness(gen), idx] = min(fitness);
    best_params(gen, :) = population(idx, :);
    mean_fitness(gen) = mean(fitness);

    % 更新动态图形
    % 子图1：适应度曲线
    set(h_plot_fitness, 'XData', 1:gen, 'YData', best_fitness(1:gen));
    set(h_plot_mean_fitness, 'XData', 1:gen, 'YData', mean_fitness(1:gen));

    % 子图2：适应度分布
    set(h_bar_fitness, 'YData', fitness);

    % 子图3：参数变化曲线
    for j = 1:nVars
        set(h_plot_params(j), 'XData', 1:gen, 'YData', best_params(1:gen, j));
    end

    % 子图4：F和CR变化曲线
    set(h_plot_F, 'XData', 1:gen, 'YData', F_history(1:gen));
    set(h_plot_CR, 'XData', 1:gen, 'YData', CR_history(1:gen));

    % 子图5：多样性变化曲线
    set(h_plot_diversity, 'XData', 1:gen, 'YData', diversity_history(1:gen));

    % 子图6：F和CR分布散点图
    set(h_scatter_F_CR, 'XData', F_values, 'YData', CR_values);

    % 动态调整坐标轴和添加信息标注
    subplot(2, 3, 1);

    if gen > 1
        ylim([0 max([best_fitness(1:gen); mean_fitness(1:gen)]) * 1.1]);
    end

    subplot(2, 3, 2);
    ylim([0 max(fitness) * 1.1]);
    title(sprintf('第 %d 代适应度分布 (最优: %.4f)', gen, best_fitness(gen)));

    subplot(2, 3, 5);

    if gen > 1
        ylim([0 max(diversity_history(1:gen)) * 1.1]);
    end

    % 在图表上添加当前信息
    annotation('textbox', [0.35, 0.05, 0.3, 0.08], ...
        'String', sprintf('当前迭代: %d/%d\n最优适应度: %.4f\n平均适应度: %.4f\n当前F值: %.3f\n当前CR值: %.3f', ...
        gen, maxGen, best_fitness(gen), mean_fitness(gen), mu_F, mu_CR), ...
        'FitBoxToText', 'on', 'BackgroundColor', 'w', 'EdgeColor', 'k');

    drawnow limitrate; % 优化绘图性能
    fprintf('第 %d 代: 最优适应度 = %.4f, 平均适应度 = %.4f, F = %.3f, CR = %.3f, 多样性 = %.3f\n', ...
        gen, best_fitness(gen), mean_fitness(gen), mu_F, mu_CR, diversity);
end

elapsed_time = toc; % 结束计时

%% 提取最优结果
[opt_fval, opt_gen] = min(best_fitness);
optimal_params = best_params(opt_gen, :);

%% 显示优化结果
fprintf('\n');
disp('=== 自适应DE优化结果 ===');
disp(['优化耗时：', num2str(elapsed_time), ' 秒']);
disp(['最优综合适应度值：', num2str(opt_fval), ' (在第 ', num2str(opt_gen), ' 代获得)']);
disp('最优PID参数：');
disp(['外环 Kp = ', num2str(optimal_params(1))]);
disp(['外环 Ki = ', num2str(optimal_params(2))]);
disp(['外环 Kd = ', num2str(optimal_params(3))]);
disp(['内环 Kp = ', num2str(optimal_params(4))]);
disp(['内环 Ki = ', num2str(optimal_params(5))]);
disp(['内环 Kd = ', num2str(optimal_params(6))]);

%% 使用最优参数运行完整仿真
disp('使用最优参数重新运行完整仿真...');
rocket_simulation_full(optimal_params);

%% 保存优化结果
save('optimal_adaptive_de_params.mat', 'optimal_params', 'opt_fval', 'best_fitness', ...
    'mean_fitness', 'F_history', 'CR_history', 'diversity_history', 'best_params');
disp('优化结果已保存至 optimal_adaptive_de_params.mat');

%% 绘制完整进化历程图表
figure('Name', '自适应DE优化完整历程', 'Position', [200, 200, 1200, 900], 'Color', 'w');

% 适应度进化曲线
subplot(3, 2, 1);
plot(1:maxGen, best_fitness, 'b-', 'LineWidth', 2);
hold on;
plot(1:maxGen, mean_fitness, 'r--', 'LineWidth', 1.5);
title('适应度进化曲线');
xlabel('迭代次数'); ylabel('适应度值');
legend('最优适应度', '平均适应度', 'Location', 'best');
grid on;

% 自适应参数变化
subplot(3, 2, 2);
plot(1:maxGen, F_history, 'g-', 'LineWidth', 2);
hold on;
plot(1:maxGen, CR_history, 'm-', 'LineWidth', 2);
title('自适应参数变化曲线');
xlabel('迭代次数'); ylabel('参数值');
legend('变异因子F', '交叉率CR', 'Location', 'best');
grid on;

% 种群多样性变化
subplot(3, 2, 3);
plot(1:maxGen, diversity_history, 'k-', 'LineWidth', 2);
title('种群多样性变化曲线');
xlabel('迭代次数'); ylabel('多样性指标');
grid on;

% 各PID参数变化 - 外环
subplot(3, 2, 4);
plot(1:maxGen, best_params(:, 1), 'r-', 'LineWidth', 2);
hold on;
plot(1:maxGen, best_params(:, 2), 'g-', 'LineWidth', 2);
plot(1:maxGen, best_params(:, 3), 'b-', 'LineWidth', 2);
title('外环PID参数进化曲线');
xlabel('迭代次数'); ylabel('参数值');
legend('Kp_{out}', 'Ki_{out}', 'Kd_{out}', 'Location', 'best');
grid on;

% 各PID参数变化 - 内环
subplot(3, 2, 5);
plot(1:maxGen, best_params(:, 4), 'r-', 'LineWidth', 2);
hold on;
plot(1:maxGen, best_params(:, 5), 'g-', 'LineWidth', 2);
plot(1:maxGen, best_params(:, 6), 'b-', 'LineWidth', 2);
title('内环PID参数进化曲线');
xlabel('迭代次数'); ylabel('参数值');
legend('Kp_{in}', 'Ki_{in}', 'Kd_{in}', 'Location', 'best');
grid on;

% 综合信息面板
subplot(3, 2, 6);
text(0.1, 0.9, '优化综合信息', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.8, ['运行时间: ', num2str(elapsed_time), ' 秒'], 'FontSize', 12);
text(0.1, 0.7, ['总迭代次数: ', num2str(maxGen)], 'FontSize', 12);
text(0.1, 0.6, ['最优解在第 ', num2str(opt_gen), ' 代获得'], 'FontSize', 12);
text(0.1, 0.5, ['最优适应度值: ', num2str(opt_fval)], 'FontSize', 12);
text(0.1, 0.4, ['初始F值: ', num2str(F_init), ', 最终F值: ', num2str(F_history(end))], 'FontSize', 12);
text(0.1, 0.3, ['初始CR值: ', num2str(CR_init), ', 最终CR值: ', num2str(CR_history(end))], 'FontSize', 12);
text(0.1, 0.2, ['初始多样性: ', num2str(diversity_history(1)), ', 最终多样性: ', num2str(diversity_history(end))], 'FontSize', 12);
axis off;

% 保存最终图表
saveas(gcf, 'adaptive_de_optimization_results.fig');
saveas(gcf, 'adaptive_de_optimization_results.png');
disp('优化完整结果图表已保存。');

disp('自适应差分进化算法优化任务完成！');
