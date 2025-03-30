% 文件名：optimize_pid_with_ga.m
% 作者：Grok 3 (xAI优化版)
% 功能：使用MATLAB遗传算法工具箱优化火箭仿真中的内外环PID参数
% 目标：最小化俯仰角跟踪的均方误差 (RMSE)

%% 初始化环境
clear; clc; close all; % 清除变量、命令行和关闭图形窗口

% 设置中文显示，确保绘图中的文字显示为中文
set(0, 'DefaultAxesFontName', 'SimHei');
set(0, 'DefaultTextFontName', 'SimHei');
set(0, 'DefaultFigureColor', 'w'); % 设置图形背景为白色

%% 定义优化问题
nVars = 6; % 优化变量数：[Kp_outer, Ki_outer, Kd_outer, Kp_inner, Ki_inner, Kd_inner]

% 参数范围（根据物理意义和稳定性设定）
lb = [0, 0, 0, 0, 0, 0];           % 下界，所有参数非负
ub = [10, 10, 10, 10, 10, 10];        % 上界，限制增益大小以避免不稳定

% 适应度函数（调用独立的rocket_simulation_fitness.m）
fitnessFcn = @rocket_simulation_fitness;

%% 配置遗传算法选项
options = optimoptions('ga', ...
    'PopulationSize', 60, ...      % 种群大小，每次迭代50个个体
    'MaxGenerations', 80, ...      % 最大代数，迭代40代
    'CrossoverFraction', 0.8, ...  % 交叉概率，80%的个体参与交叉
    'MutationFcn', @mutationadaptfeasible, ... % 自适应变异函数
    'Display', 'iter', ...         % 每代显示迭代信息
    'PlotFcn', {@gaplotbestf, @gaplotscores}, ... % 绘制最优适应度和分数分布
    'UseParallel', true, ...      % 是否并行计算（可根据硬件启用）
    'OutputFcn', @ga_output_function); % 自定义输出函数

% 自定义输出函数：记录每代最优解并保存
function [state, options, optchanged] = ga_output_function(options, state, flag)
    persistent best_solutions; % 持久变量，保存历史数据
    if strcmp(flag, 'init') % 初始化阶段
        best_solutions = [];
    end
    if strcmp(flag, 'iter') % 每代迭代
        best_solutions = [best_solutions; [state.Generation, state.Best(end)]];
    end
    if strcmp(flag, 'done') % 优化结束
        save('ga_optimization_history.mat', 'best_solutions');
        disp('优化历史已保存至 ga_optimization_history.mat');
        % 可视化优化过程
        figure('Name', '遗传算法优化过程', 'NumberTitle', 'off', 'Color', 'w');
        plot(best_solutions(:,1), best_solutions(:,2), 'b-', 'LineWidth', 2);
        title('最优适应度值随代数变化');
        xlabel('代数'); ylabel('RMSE (°)');
        grid on;
    end
    optchanged = false; % 未修改选项
end

%% 运行遗传算法
disp('开始遗传算法优化PID参数...');
tic; % 开始计时
[optimal_params, fval, exitflag, output] = ga(fitnessFcn, nVars, ...
    [], [], [], [], lb, ub, [], options); % 调用GA优化
elapsed_time = toc; % 结束计时

%% 显示优化结果
disp('=== 优化结果 ===');
disp(['优化耗时：', num2str(elapsed_time), ' 秒']);
disp(['最优适应度值（RMSE）：', num2str(fval), '°']);
disp('最优PID参数：');
disp(['外环 Kp = ', num2str(optimal_params(1))]);
disp(['外环 Ki = ', num2str(optimal_params(2))]);
disp(['外环 Kd = ', num2str(optimal_params(3))]);
disp(['内环 Kp = ', num2str(optimal_params(4))]);
disp(['内环 Ki = ', num2str(optimal_params(5))]);
disp(['内环 Kd = ', num2str(optimal_params(6))]);
disp(['退出标志：', num2str(exitflag)]);
disp(['迭代次数：', num2str(output.generations)]);

%% 用最优参数重新运行完整仿真
disp('使用最优参数重新运行完整仿真...');
rocket_simulation_full(optimal_params); % 调用完整仿真程序验证结果

%% 保存优化结果
save('optimal_pid_params.mat', 'optimal_params', 'fval');
disp('最优参数已保存至 optimal_pid_params.mat');

disp('优化任务完成！');
