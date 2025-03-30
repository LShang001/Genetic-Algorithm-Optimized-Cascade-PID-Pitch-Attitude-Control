% 文件名：rocket_simulation_fitness.m
% 作者：Grok 3 (xAI优化版)
% 功能：简化版火箭仿真，用于遗传算法优化PID参数，计算带超调抑制和控制量变化率惩罚的适应度值
% 输入：params - PID参数数组 [Kp_outer, Ki_outer, Kd_outer, Kp_inner, Ki_inner, Kd_inner]
% 输出：fitness - 加权适应度值（RMSE + 超调惩罚 + 控制量变化率惩罚），综合衡量跟踪误差、超调和控制平稳性

function fitness = rocket_simulation_fitness(params)
    %% 初始化参数
    % 从输入数组提取内外环PID控制器的增益参数
    Kp_outer = params(1); % 外环比例增益（Kp），控制角度误差的响应强度
    Ki_outer = params(2); % 外环积分增益（Ki），用于消除角度稳态误差
    Kd_outer = params(3); % 外环微分增益（Kd），抑制角度变化速率，增加阻尼
    Kp_inner = params(4); % 内环比例增益（Kp），控制角速度误差的响应
    Ki_inner = params(5); % 内环积分增益（Ki），减小角速度稳态偏差
    Kd_inner = params(6); % 内环微分增益（Kd），提高角速度控制的平稳性

    %% 子函数定义
    % 定义低通滤波器、PID控制器和动力学更新的辅助函数，嵌入式声明便于主函数调用

    % 低通滤波器初始化函数
    function lpf = initLowPassFilter(alpha)
        % 初始化低通滤波器，用于平滑信号，减少高频噪声
        % 输入：alpha - 滤波系数，取值(0,1)，值越小滤波强度越大
        lpf.alpha = alpha;    % 保存滤波系数到结构体
        lpf.y_prev = 0;       % 初始化前一输出值为0，作为滤波起点
    end

    % 低通滤波器更新函数
    function lpf = updateLowPassFilter(lpf, x)
        % 更新低通滤波器状态，计算当前滤波输出
        % 输入：lpf - 滤波器结构体（包含alpha和y_prev），x - 当前输入值
        % 输出：lpf - 更新后的滤波器结构体
        lpf.y_prev = lpf.alpha * x + (1 - lpf.alpha) * lpf.y_prev; % 一阶低通滤波公式，平滑输入信号
    end

    % PID控制器初始化函数
    function pid = initPIDController(Kp, Ki, Kd, dt, integral_max, integral_min, alpha_d, alpha_output)
        % 初始化PID控制器，设置参数和初始状态
        % 输入：Kp, Ki, Kd - 比例、积分、微分增益；dt - 控制周期；integral_max/min - 积分限幅；alpha_d/output - 微分和输出滤波系数
        % 输出：pid - PID控制器结构体
        pid = struct(... % 创建结构体存储PID参数和状态
            'Kp', Kp,...           % 比例增益
            'Ki', Ki,...           % 积分增益
            'Kd', Kd,...           % 微分增益
            'dt', dt,...           % 控制时间步长（秒）
            'integral', 0,...      % 初始积分值为0
            'prev_error', 0,...    % 前一误差值，初始为0，用于微分计算
            'integral_max', integral_max,... % 积分上限，防止积分饱和
            'integral_min', integral_min,... % 积分下限
            'lpf_d', initLowPassFilter(alpha_d),... % 微分项低通滤波器，平滑微分信号
            'lpf_output', initLowPassFilter(alpha_output)); % 输出低通滤波器，平滑控制量
    end

    % PID控制器更新函数
    function [pid, control_filtered] = updatePIDController(pid, error)
        % 更新PID控制器状态，计算控制输出
        % 输入：pid - PID结构体，error - 当前误差（目标值与实际值的差）
        % 输出：pid - 更新后的控制器状态，control_filtered - 滤波后的控制输出
        P = pid.Kp * error; % 计算比例项，放大误差响应
        pid.integral = pid.integral + error * pid.dt; % 更新积分项，累积误差（矩形积分法）
        pid.integral = max(min(pid.integral, pid.integral_max), pid.integral_min); % 限制积分值，避免饱和
        I = pid.Ki * pid.integral; % 计算积分项输出，消除稳态误差
        derivative = (error - pid.prev_error) / pid.dt; % 计算微分项，后向差分法求变化率
        pid.lpf_d = updateLowPassFilter(pid.lpf_d, derivative); % 对微分项滤波，减少高频噪声
        D = pid.Kd * pid.lpf_d.y_prev; % 计算微分项输出，增加系统阻尼
        control = P + I + D; % 未滤波的PID总输出
        pid.lpf_output = updateLowPassFilter(pid.lpf_output, control); % 对总输出滤波，平滑控制信号
        control_filtered = pid.lpf_output.y_prev; % 获取滤波后的控制输出
        pid.prev_error = error; % 保存当前误差，用于下次微分计算
    end

    % RK4动力学更新函数
    function state_next = updateDynamicsRK4(state, thrust, gimbal_angle, mass, inertia, gimbal_to_cg, dt)
        % 使用4阶Runge-Kutta方法更新火箭动力学状态
        % 输入：state - 当前状态向量 [俯仰角, 角速度, x位置, x速度, y位置, y速度]，thrust - 推力 (N)，
        %       gimbal_angle - 万向节角度 (°)，mass - 当前质量 (kg)，inertia - 转动惯量 (kg·m²)，gimbal_to_cg - 质心距离 (m)，dt - 时间步长 (s)
        % 输出：state_next - 更新后的状态向量
        function dxdt = dynamics(x, thrust, gimbal_angle, mass, inertia, gimbal_to_cg)
            theta = x(1);    % 俯仰角 (°)，逆时针为正（x轴正方向向左）
            omega = x(2);
            x_vel = x(4);    % x速度：正为左
            y_vel = x(6);    % y速度：正为上

            % 推力方向（x轴正方向向左）
            thrust_angle = theta - gimbal_angle;
            thrust_x = -thrust * sind(thrust_angle); % 正为左
            thrust_y = thrust * cosd(thrust_angle);  % 正为上

            acc_x = thrust_x / mass;
            acc_y = (thrust_y - 9.81 * mass) / mass;


            % 力矩计算（顺时针摆角产生逆时针力矩）
            torque = thrust * gimbal_to_cg * sind(gimbal_angle); % 正力矩
            alpha = rad2deg(torque / inertia);

            dxdt = [omega; alpha; x_vel; acc_x; y_vel; acc_y];
        end

        % RK4积分
        k1 = dynamics(state, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);
        k2 = dynamics(state + dt/2*k1, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);
        k3 = dynamics(state + dt/2*k2, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);
        k4 = dynamics(state + dt*k3, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);

        state_next = state + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        torque = -thrust * gimbal_to_cg * sind(gimbal_angle);
        pitch_acceleration = rad2deg(torque / inertia);
    end

    %% 主程序参数配置
    % 定义仿真参数和火箭物理特性
    dt_control = 0.004;        % 控制周期 (s)，250Hz采样率
    dt_dynamics = 0.001;       % 动力学更新步长 (s)，1kHz更新频率
    time_duration = 4;        % 仿真总时长 (s)，4秒仿真
    control_steps = ceil(time_duration / dt_control); % 总控制步数，向上取整
    initial_mass = 15.8;       % 初始质量 (kg)，包括燃料
    fuel_mass = 3.0;           % 初始燃料质量 (kg)
    empty_mass = 12.8;         % 空载质量 (kg)，无燃料时
    max_thrust = 200;          % 最大推力 (N)
    max_mass_flow_rate = 0.12; % 最大质量流率 (kg/s)，燃料消耗速率
    gimbal_to_cg_full = 0.58;  % 满载时万向节到质心距离 (m)
    gimbal_to_cg_empty = 0.50; % 空载时万向节到质心距离 (m)
    inertia_full = 2.76;       % 满载时转动惯量 (kg·m²)
    inertia_empty = 2.1;       % 空载时转动惯量 (kg·m²)
    max_gimbal_angle = 15;     % 万向节最大摆角 (°)
    max_omega = 80;            % 角速度最大值 (°/s)，限制外环输出
    manual_thrust = 160;       % 固定推力值 (N)，手动设定
    integral_max_outer = 100;  % 外环积分上限，允许较大积分范围
    integral_min_outer = -100; % 外环积分下限
    integral_max_inner = 100;  % 内环积分上限
    integral_min_inner = -100; % 内环积分下限
    alpha = 0.3;               % 目标俯仰角滤波系数，平滑目标信号
    alpha_d_outer = 1;         % 外环微分滤波系数，1表示无滤波
    alpha_output_outer = 1;    % 外环输出滤波系数，1表示无滤波
    alpha_d_inner = 1;         % 内环微分滤波系数，1表示无滤波
    alpha_output_inner = 1;    % 内环输出滤波系数，1表示无滤波
    delay_time = 0.01;         % 执行机构延迟时间 (s)，模拟机械响应延迟
    delay_steps = round(delay_time / dt_control); % 延迟步数，基于控制周期计算
    tau_actuator = 0.08;       % 执行机构时间常数 (s)，一阶滞后特性
    alpha_actuator = dt_control / (tau_actuator + dt_control); % 一阶滞后离散系数

    % 噪声参数
    gimbal_angle_error = 0.1;% 万向节角度固定误差 (°)
    gimbal_angle_noise_std = 0.3; % 万向节角度噪声标准差 (°)，增加以模拟扰动

    %% 初始化系统
    % 设置控制器和初始状态变量
    pid_outer = initPIDController(Kp_outer, Ki_outer, Kd_outer, dt_control, ...
        integral_max_outer, integral_min_outer, alpha_d_outer, alpha_output_outer); % 初始化外环PID控制器
    pid_inner = initPIDController(Kp_inner, Ki_inner, Kd_inner, dt_control, ...
        integral_max_inner, integral_min_inner, alpha_d_inner, alpha_output_inner); % 初始化内环PID控制器
    lpf = initLowPassFilter(alpha); % 初始化目标角度低通滤波器
    gimbal_angle_buffer = zeros(1, delay_steps); % 执行机构延迟缓冲区，存储历史控制指令
    current_gimbal_angle_delayed = 0; % 延迟后的万向节角度，初始为0
    current_gimbal_angle_actual = 0;  % 滞后后的实际角度，初始为0
    time_list_control = zeros(1, control_steps); % 预分配控制时间数组（250Hz）
    pitch_angle_list_control = zeros(1, control_steps); % 预分配实际俯仰角数组
    target_pitch_angle_list_control = zeros(1, control_steps); % 预分配目标俯仰角数组
    gimbal_angle_list_control = zeros(1, control_steps); % 预分配控制量（万向节角度）数组，用于变化率计算
    state = [0; 0; 0; 0; 0; 0]; % 初始状态向量：[俯仰角, 角速度, x位置, x速度, y位置, y速度]
    current_mass = initial_mass; % 当前质量，初始为满载质量
    current_inertia = inertia_full; % 当前转动惯量，初始为满载值
    current_gimbal_to_cg = gimbal_to_cg_full; % 当前质心距离，初始为满载值

    %% 主仿真循环
    % 模拟火箭的姿态控制和平动过程
    t = 0; % 当前仿真时间 (s)，初始为0
    step = 1; % 控制步数计数器，初始为1
    while t < time_duration
        % 控制更新（250Hz采样率）
        if step <= control_steps && abs(t - (step-1)*dt_control) < 1e-10
            % 检查是否到达控制周期点（误差小于1e-10，确保时间同步）
            t_control = (step-1) * dt_control; % 当前控制时间点，从0开始递增
            target_pitch_angle = 5; % 生成目标俯仰角
            lpf = updateLowPassFilter(lpf, target_pitch_angle); % 对目标角度进行低通滤波，平滑过渡
            filtered_target_pitch_angle = lpf.y_prev; % 获取滤波后的目标角度

            theta_error = filtered_target_pitch_angle - state(1); % 计算角度误差（目标 - 实际）
            
            [pid_outer, omega_ref] = updatePIDController(pid_outer, theta_error); % 外环PID计算角速度指令
            omega_ref = max(min(omega_ref, max_omega), -max_omega); % 限制角速度指令在±max_omega°/s范围内
            
            omega_error = omega_ref - state(2); % 计算角速度误差（指令 - 实际）
            [pid_inner, control] = updatePIDController(pid_inner, omega_error); % 内环PID计算控制量
            current_gimbal_angle = max(min(control, max_gimbal_angle), -max_gimbal_angle); % 限制控制量（万向节角度）在±15°内

            gimbal_angle_buffer = [current_gimbal_angle, gimbal_angle_buffer(1:end-1)]; % 更新延迟缓冲区（先进先出）
            current_gimbal_angle_delayed = gimbal_angle_buffer(end); % 获取延迟后的角度
            current_gimbal_angle_actual = (1 - alpha_actuator) * current_gimbal_angle_actual + ...
                alpha_actuator * current_gimbal_angle_delayed; % 一阶滞后更新实际角度，模拟执行机构动态

            gimbal_angle_noise = gimbal_angle_noise_std * randn;
            current_gimbal_angle_actual = current_gimbal_angle_actual + gimbal_angle_error + gimbal_angle_noise; % 添加噪声

            time_list_control(step) = t_control; % 记录当前控制时间
            pitch_angle_list_control(step) = state(1); % 记录当前实际俯仰角
            target_pitch_angle_list_control(step) = target_pitch_angle; % 记录当前目标俯仰角
            gimbal_angle_list_control(step) = current_gimbal_angle; % 记录当前控制量（万向节角度）
            step = step + 1; % 控制步数加1
        end
        % 动力学更新（1kHz采样率）
        if fuel_mass > 0
            current_mass_flow_rate = max_mass_flow_rate * (manual_thrust/max_thrust); % 计算当前质量流率，随推力比例变化
            fuel_mass = max(fuel_mass - current_mass_flow_rate * dt_dynamics, 0); % 更新燃料质量，确保非负
            current_mass = empty_mass + fuel_mass; % 更新总质量
        end
        mass_ratio = (initial_mass - current_mass)/(initial_mass - empty_mass); % 计算质量消耗比例（0到1）
        current_gimbal_to_cg = gimbal_to_cg_full - mass_ratio*(gimbal_to_cg_full - gimbal_to_cg_empty); % 线性插值质心距离
        current_inertia = inertia_full - mass_ratio*(inertia_full - inertia_empty); % 线性插值转动惯量
        state = updateDynamicsRK4(state, manual_thrust, current_gimbal_angle_actual, ...
            current_mass, current_inertia, current_gimbal_to_cg, dt_dynamics); % 使用RK4更新动力学状态
        t = t + dt_dynamics; % 时间推进一个动力学步长
    end

    %% 计算适应度值（RMSE + 超调抑制 + 控制量变化率惩罚）
    % 计算RMSE（均方误差）
    rmse = sqrt(mean((target_pitch_angle_list_control - pitch_angle_list_control).^2)); % 计算实际与目标俯仰角的均方根误差，单位：°

    % 计算超调量（分阶段处理）
    overshoot = 0; % 初始化最大超调量，单位：°
    for i = 1:control_steps
        target = target_pitch_angle_list_control(i); % 当前步的目标俯仰角
        actual = pitch_angle_list_control(i); % 当前步的实际俯仰角     
        os = max(actual - target, 0); % 计算正向超调（超出目标的部分）
        overshoot = max(overshoot, os); % 更新全局最大超调量，取整个仿真中的最大值
    end

    % 计算控制量变化率（RMS）
    gimbal_rate = diff(gimbal_angle_list_control) / dt_control; % 计算控制量（万向节角度）相邻步的变化率，单位：°/s
    control_rate_rms = sqrt(mean(gimbal_rate.^2))/100; % 计算变化率的均方根值，衡量控制信号的总波动强度，归一化到 100 °/s

    time_ratio = t / time_duration;
    w1 = 0.75 + 0.15 * (1 - time_ratio); % 前期更重 RMSE
    w2 = 0.1 + 0.15 * time_ratio; % 后期更重超调
    w3 = 0.15; % 变化率权重不变

    fitness = w1 * rmse + w2 * overshoot + w3 * control_rate_rms; % 计算最终适应度值，综合三项指标
end
