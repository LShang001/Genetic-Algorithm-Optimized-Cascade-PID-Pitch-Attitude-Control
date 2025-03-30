% 文件名：optimize_pid_position_x_de.m
% 功能：使用自适应差分进化算法优化火箭水平位置(X)控制PID参数
%       固定使用之前优化的姿态控制PID参数
% 基于：LShang的optimize_pid_with_adaptive_de.m

%% 初始化环境
clear; clc; close all;
set(0, 'DefaultAxesFontName', 'SimHei');
set(0, 'DefaultTextFontName', 'SimHei');
set(0, 'DefaultFigureColor', 'w');

%% DE参数配置
nVars = 6; % 优化变量数 [Kp_pos_x, Ki_pos_x, Kd_pos_x, Kp_vel_x, Ki_vel_x, Kd_vel_x]
lb = [0.1, 0, 0, 8, 0, 0]; % 下界 (Kp不能为0)
ub = [20, 10, 10, 20, 10, 0]; % 上界 (适当放宽)

% DE核心参数
NP = 50; % 种群规模
F_init = 0.8; % 初始变异因子
CR_init = 0.9; % 初始交叉率
maxGen = 80; % 最大迭代次数

% 自适应参数设置
c = 0.1; % 自适应参数学习率
p = 0.1; % 优秀个体比例

%% 初始化种群和记录
population = repmat(lb, NP, 1) + rand(NP, nVars) .* repmat(ub - lb, NP, 1);
fitness = inf(NP, 1);
best_fitness = zeros(maxGen, 1);
mean_fitness = zeros(maxGen, 1);
best_params = zeros(maxGen, nVars);
F_history = zeros(maxGen, 1);
CR_history = zeros(maxGen, 1);
diversity_history = zeros(maxGen, 1);

% 初始化自适应参数
F_values = F_init * ones(NP, 1);
CR_values = CR_init * ones(NP, 1);
mu_F = F_init;
mu_CR = CR_init;
S_F = [];
S_CR = [];

% 并行计算初始适应度
disp('正在并行计算初始适应度...');

parfor i = 1:NP
    fitness(i) = rocket_simulation_fitness_position_x(population(i, :));
end

disp('初始适应度计算完成。');

%% 创建动态监控图形窗口
figure('Name', '水平位置环PID优化进程监控', 'Position', [100, 100, 1400, 800], 'Color', 'w');

% 子图1：适应度曲线
subplot(2, 3, 1);
h_plot_fitness = plot(nan, nan, 'b-', 'LineWidth', 1.5); hold on;
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
ylim([0 max(fitness(isfinite(fitness))) * 1.1 + eps]);
grid on;

% 子图3：最优PID参数迭代过程
subplot(2, 3, 3);
param_labels = {'Kp_{pos}', 'Ki_{pos}', 'Kd_{pos}', 'Kp_{vel}', 'Ki_{vel}', 'Kd_{vel}'};
h_plot_params = gobjects(nVars, 1);
colors = lines(nVars);

for i = 1:nVars
    h_plot_params(i) = plot(nan, nan, 'Color', colors(i, :), 'LineWidth', 1.5);
    hold on;
end

legend(param_labels, 'Location', 'bestoutside');
title('最优PID参数进化过程');
xlabel('迭代次数'); ylabel('参数值');
xlim([0 maxGen]); ylim([0 max(ub) * 1.1]);
grid on;

% 子图4：自适应参数F和CR变化
subplot(2, 3, 4);
h_plot_F = plot(nan, nan, 'g-', 'LineWidth', 1.5); hold on;
h_plot_CR = plot(nan, nan, 'm-', 'LineWidth', 1.5);
title('自适应参数变化');
xlabel('迭代次数'); ylabel('参数值');
legend('变异因子F', '交叉率CR', 'Location', 'best');
xlim([0 maxGen]); ylim([0 1.1]);
grid on;

% 子图5：种群多样性变化
subplot(2, 3, 5);
h_plot_diversity = plot(nan, nan, 'k-', 'LineWidth', 1.5);
title('种群多样性变化');
xlabel('迭代次数'); ylabel('多样性指标');
xlim([0 maxGen]); ylim([0 1]);
grid on;

% 子图6：F和CR的分布
subplot(2, 3, 6);
h_scatter_F_CR = scatter(F_values, CR_values, 30, 'filled');
title('F和CR值分布');
xlabel('变异因子F'); ylabel('交叉率CR');
xlim([0 1]); ylim([0 1]);
grid on;

%% DE算法主迭代
disp('开始自适应差分进化算法优化水平位置环PID参数...');
start_time = tic;

for gen = 1:maxGen
    % 清空成功F和CR记录
    S_F = [];
    S_CR = [];

    % 生成新的F和CR值
    for i = 1:NP
        F_values(i) = normrnd(mu_F, 0.1);
        F_values(i) = max(0.1, min(1, F_values(i)));
        CR_values(i) = normrnd(mu_CR, 0.1);
        CR_values(i) = max(0, min(1, CR_values(i)));
    end

    % 准备新一代种群
    new_population = population;
    new_fitness = fitness;

    % 并行处理每个个体
    parfor i = 1:NP
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

        if ~any(cross_points)
            cross_points(randi(nVars)) = true;
        end

        trial = population(i, :);
        trial(cross_points) = mutant(cross_points);

        % 评估新个体的综合适应度
        trial_fitness = rocket_simulation_fitness_position_x(trial);

        % 贪婪选择
        if trial_fitness < fitness(i)
            new_population(i, :) = trial;
            new_fitness(i) = trial_fitness;
            S_F = [S_F; F_i];
            S_CR = [S_CR; CR_i];
        end

    end

    % 更新种群和适应度
    population = new_population;
    fitness = new_fitness;

    % 更新自适应参数
    if ~isempty(S_F) && ~isempty(S_CR)

        if length(S_F) > 1
            mu_F = (1 - c) * mu_F + c * sum(S_F .^ 2) / sum(S_F); % Lehmer mean
        else
            mu_F = (1 - c) * mu_F + c * S_F(1);
        end

        mu_CR = (1 - c) * mu_CR + c * mean(S_CR); % Arithmetic mean
    end

    % 记录参数历史
    F_history(gen) = mu_F;
    CR_history(gen) = mu_CR;
    pop_std = std(population);
    diversity_history(gen) = mean(pop_std ./ (ub - lb)); % 归一化多样性

    % 记录当前代最优解和平均适应度
    [best_fitness(gen), idx] = min(fitness);
    best_params(gen, :) = population(idx, :);
    mean_fitness(gen) = mean(fitness);

    % 更新动态图形
    set(h_plot_fitness, 'XData', 1:gen, 'YData', best_fitness(1:gen));
    set(h_plot_mean_fitness, 'XData', 1:gen, 'YData', mean_fitness(1:gen));
    set(h_bar_fitness, 'YData', fitness);

    for i = 1:nVars
        set(h_plot_params(i), 'XData', 1:gen, 'YData', best_params(1:gen, i));
    end

    set(h_plot_F, 'XData', 1:gen, 'YData', F_history(1:gen));
    set(h_plot_CR, 'XData', 1:gen, 'YData', CR_history(1:gen));
    set(h_plot_diversity, 'XData', 1:gen, 'YData', diversity_history(1:gen));
    set(h_scatter_F_CR, 'XData', F_values, 'YData', CR_values);

    % 显示当前信息
    subplot(2, 3, 2);
    title(sprintf('第 %d 代适应度分布 (最优: %.4f)', gen, best_fitness(gen)));

    % 添加信息标注
    annotation('textbox', [0.35, 0.05, 0.3, 0.08], ...
        'String', sprintf('当前迭代: %d/%d\n最优适应度: %.4f\n平均适应度: %.4f\n当前F值: %.3f\n当前CR值: %.3f', ...
        gen, maxGen, best_fitness(gen), mean_fitness(gen), mu_F, mu_CR), ...
        'FitBoxToText', 'on', 'BackgroundColor', 'w', 'EdgeColor', 'k');

    drawnow limitrate;
    fprintf('第 %d 代: 最优适应度 = %.4f, 平均适应度 = %.4f, F = %.3f, CR = %.3f, 多样性 = %.3f\n', ...
        gen, best_fitness(gen), mean_fitness(gen), mu_F, mu_CR, diversity_history(gen));
end

elapsed_time = toc(start_time);

%% 提取最优结果
[opt_fval, opt_gen] = min(best_fitness);
optimal_params = best_params(opt_gen, :);

%% 显示优化结果
fprintf('\n=== 水平位置环PID优化结果 ===\n');
disp(['优化耗时：', num2str(elapsed_time / 60, '%.1f'), ' 分钟']);
disp(['最优综合适应度值：', num2str(opt_fval, '%.4f'), ' (在第 ', num2str(opt_gen), ' 代获得)']);
disp('最优位置和速度环PID参数：');
disp(['位置环 Kp = ', num2str(optimal_params(1), '%.4f')]);
disp(['位置环 Ki = ', num2str(optimal_params(2), '%.4f')]);
disp(['位置环 Kd = ', num2str(optimal_params(3), '%.4f')]);
disp(['速度环 Kp = ', num2str(optimal_params(4), '%.4f')]);
disp(['速度环 Ki = ', num2str(optimal_params(5), '%.4f')]);
disp(['速度环 Kd = ', num2str(optimal_params(6), '%.4f')]);

%% 保存优化结果
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
save(['optimal_position_x_de_params_', timestamp, '.mat'], ...
    'optimal_params', 'opt_fval', 'best_fitness', 'mean_fitness', ...
    'F_history', 'CR_history', 'diversity_history', 'best_params');
disp(['优化结果已保存至 optimal_position_x_de_params_', timestamp, '.mat']);

%% 绘制完整进化历程图表
figure('Name', '水平位置环PID优化完整历程', 'Position', [200, 200, 1200, 900], 'Color', 'w');

% 适应度进化曲线
subplot(3, 2, 1);
plot(1:maxGen, best_fitness, 'b-', 'LineWidth', 2); hold on;
plot(1:maxGen, mean_fitness, 'r--', 'LineWidth', 1.5);
title('适应度进化曲线');
xlabel('迭代次数'); ylabel('适应度值');
legend('最优适应度', '平均适应度', 'Location', 'best');
grid on;

% 自适应参数变化
subplot(3, 2, 2);
plot(1:maxGen, F_history, 'g-', 'LineWidth', 2); hold on;
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

% 位置环PID参数变化
subplot(3, 2, 4);
plot(1:maxGen, best_params(:, 1), 'r-', 'LineWidth', 2); hold on;
plot(1:maxGen, best_params(:, 2), 'g-', 'LineWidth', 2);
plot(1:maxGen, best_params(:, 3), 'b-', 'LineWidth', 2);
title('位置环PID参数进化');
xlabel('迭代次数'); ylabel('参数值');
legend('Kp_{pos}', 'Ki_{pos}', 'Kd_{pos}', 'Location', 'best');
grid on;

% 速度环PID参数变化
subplot(3, 2, 5);
plot(1:maxGen, best_params(:, 4), 'r-', 'LineWidth', 2); hold on;
plot(1:maxGen, best_params(:, 5), 'g-', 'LineWidth', 2);
plot(1:maxGen, best_params(:, 6), 'b-', 'LineWidth', 2);
title('速度环PID参数进化');
xlabel('迭代次数'); ylabel('参数值');
legend('Kp_{vel}', 'Ki_{vel}', 'Kd_{vel}', 'Location', 'best');
grid on;

% 综合信息面板
subplot(3, 2, 6);
text(0.1, 0.9, '优化综合信息', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.8, ['运行时间: ', num2str(elapsed_time / 60, '%.1f'), ' 分钟'], 'FontSize', 12);
text(0.1, 0.7, ['总迭代次数: ', num2str(maxGen)], 'FontSize', 12);
text(0.1, 0.6, ['最优解在第 ', num2str(opt_gen), ' 代获得'], 'FontSize', 12);
text(0.1, 0.5, ['最优适应度值: ', num2str(opt_fval, '%.4f')], 'FontSize', 12);
text(0.1, 0.4, ['初始F值: ', num2str(F_init), ', 最终F值: ', num2str(F_history(end), '%.3f')], 'FontSize', 12);
text(0.1, 0.3, ['初始CR值: ', num2str(CR_init), ', 最终CR值: ', num2str(CR_history(end), '%.3f')], 'FontSize', 12);
text(0.1, 0.2, ['初始多样性: ', num2str(diversity_history(1), '%.3f'), ...
                    ', 最终多样性: ', num2str(diversity_history(end), '%.3f')], 'FontSize', 12);
axis off;

% 保存最终图表
saveas(gcf, ['position_x_de_optimization_results_', timestamp, '.fig']);
saveas(gcf, ['position_x_de_optimization_results_', timestamp, '.png']);
disp('优化完整结果图表已保存。');

disp('水平位置环PID参数优化任务完成！');
