% 文件名：rocket_simulation_full.m
% 作者：Grok 3 (xAI优化版)
% 版本：2.1 (RK4积分 + 内外环PID + 执行机构物理特性)
% 功能：实现带燃料消耗的火箭俯仰姿态控制与质心平动联合仿真，支持外部传入PID参数

function rocket_simulation_full(params)
    %% 初始化环境

    % 设置中文显示，确保绘图中的文字显示为中文
    set(0, 'DefaultAxesFontName', 'SimHei');
    set(0, 'DefaultTextFontName', 'SimHei');
    set(0, 'DefaultFigureColor', 'w'); % 设置图形背景为白色

    %% 从输入参数提取PID值（若无输入则使用默认值）
    if nargin == 0 % 如果没有输入参数，使用默认值
        Kp_outer = 3.5;  % 外环比例增益
        Ki_outer = 0.0;  % 外环积分增益
        Kd_outer = 0.0;  % 外环微分增益
        Kp_inner = 1.8;  % 内环比例增益
        Ki_inner = 1.2;  % 内环积分增益
        Kd_inner = 0.08; % 内环微分增益
    else % 使用外部传入的参数
        Kp_outer = params(1); % 外环比例增益
        Ki_outer = params(2); % 外环积分增益
        Kd_outer = params(3); % 外环微分增益
        Kp_inner = params(4); % 内环比例增益
        Ki_inner = params(5); % 内环积分增益
        Kd_inner = params(6); % 内环微分增益
    end

    %% 子函数定义
    % 低通滤波器初始化
    function lpf = initLowPassFilter(alpha)
        % 初始化低通滤波器，用于平滑信号
        % 输入：alpha - 滤波系数 (0,1)，值越小滤波越强
        lpf.alpha = alpha; % 保存滤波系数
        lpf.y_prev = 0;    % 初始化前一输出值为0
    end

    % 低通滤波器更新
    function lpf = updateLowPassFilter(lpf, x)
        % 更新低通滤波器状态，计算当前滤波输出
        % 输入：lpf - 滤波器结构体，x - 当前输入值
        lpf.y_prev = lpf.alpha * x + (1 - lpf.alpha) * lpf.y_prev; % 一阶低通滤波公式
    end

    % PID控制器初始化
    function pid = initPIDController(Kp, Ki, Kd, dt, integral_max, integral_min, alpha_d, alpha_output)
        % 初始化PID控制器
        % 输入：Kp, Ki, Kd - 增益；dt - 控制周期；integral_max/min - 积分限幅；alpha_d/output - 滤波系数
        pid = struct(... % 创建结构体存储PID参数和状态
            'Kp', Kp,...
            'Ki', Ki,...
            'Kd', Kd,...
            'dt', dt,...
            'integral', 0,... % 初始积分值为0
            'prev_error', 0,... % 前一误差值为0
            'integral_max', integral_max,...
            'integral_min', integral_min,...
            'p_list', [],... % 比例项记录
            'i_list', [],... % 积分项记录
            'd_list', [],... % 微分项记录
            'integral_list', [],... % 积分值记录
            'lpf_d', initLowPassFilter(alpha_d),... % 微分项低通滤波器
            'lpf_output', initLowPassFilter(alpha_output)... % 输出低通滤波器
        );
    end

    % PID控制器更新
    function [pid, control_filtered] = updatePIDController(pid, error)
        % 更新PID控制器状态，计算控制输出
        % 输入：pid - PID结构体，error - 当前误差
        % 输出：pid - 更新后的结构体，control_filtered - 滤波后的控制输出
        P = pid.Kp * error; % 比例项
        pid.integral = pid.integral + error * pid.dt; % 更新积分项
        pid.integral = max(min(pid.integral, pid.integral_max), pid.integral_min); % 积分限幅
        I = pid.Ki * pid.integral; % 积分项输出
        derivative = (error - pid.prev_error) / pid.dt; % 微分项（后向差分）
        pid.lpf_d = updateLowPassFilter(pid.lpf_d, derivative); % 滤波微分信号
        D = pid.Kd * pid.lpf_d.y_prev; % 微分项输出
        control = P + I + D; % 未滤波的总输出
        pid.lpf_output = updateLowPassFilter(pid.lpf_output, control); % 滤波总输出
        control_filtered = pid.lpf_output.y_prev; % 获取滤波后的控制输出
        pid.prev_error = error; % 保存当前误差
        pid.p_list(end+1) = P; % 记录比例项
        pid.i_list(end+1) = I; % 记录积分项
        pid.d_list(end+1) = D; % 记录微分项
        pid.integral_list(end+1) = pid.integral; % 记录积分值
    end

    % RK4动力学更新
    function [state_next, pitch_acceleration] = updateDynamicsRK4(state, thrust, gimbal_angle, mass, inertia, gimbal_to_cg, dt)
        % 使用4阶Runge-Kutta方法更新火箭动力学状态
        % 输入：state - 当前状态向量，thrust - 推力，gimbal_angle - 摆角，mass - 质量，inertia - 转动惯量，gimbal_to_cg - 质心距离，dt - 时间步长
        % 输出：state_next - 更新后的状态，pitch_acceleration - 角加速度
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
    % 仿真参数
    dt_control = 0.004;      % 控制周期 (s)，250Hz
    dt_dynamics = 0.001;    % 动力学更新步长 (s)，1kHz
    time_duration = 12;      % 仿真总时长 (s)
    control_steps = ceil(time_duration / dt_control); % 总控制步数

    % 火箭参数
    initial_mass = 15.8;     % 初始质量 (kg)
    fuel_mass = 3.0;         % 初始燃料质量 (kg)
    empty_mass = 12.8;       % 空载质量 (kg)
    max_thrust = 200;        % 最大推力 (N)
    max_mass_flow_rate = 0.12; % 最大质量流率 (kg/s)
    gimbal_to_cg_full = 0.58; % 满载时质心距离 (m)
    gimbal_to_cg_empty = 0.50;% 空载时质心距离 (m)
    inertia_full = 2.76;     % 满载时转动惯量 (kg·m²)
    inertia_empty = 2.1;     % 空载时转动惯量 (kg·m²)
    max_gimbal_angle = 15;   % 最大摆角 (°)
    max_omega = 80;            % 外环输出最大角速度
    manual_thrust = 160;     % 固定推力 (N)

    % 控制参数（内外环PID）
    integral_max_outer = 100; integral_min_outer = -100; % 外环积分限幅
    integral_max_inner = 100; integral_min_inner = -100; % 内环积分限幅
    alpha = 0.3;            % 目标俯仰角滤波系数
    alpha_d_outer = 1;      % 外环微分滤波系数
    alpha_output_outer = 1; % 外环输出滤波系数
    alpha_d_inner = 1;    % 内环微分滤波系数
    alpha_output_inner = 1; % 内环输出滤波系数

    % 执行机构参数
    delay_time = 0.01;       % 执行机构延迟时间 (s)
    delay_steps = round(delay_time / dt_control); % 延迟步数
    tau_actuator = 0.05;     % 执行机构时间常数 (s)
    alpha_actuator = dt_control / (tau_actuator + dt_control); % 一阶滞后系数

    % 噪声参数
    gimbal_angle_error = 0.0; % 万向节角度固定误差 (°)
    gimbal_angle_noise_std = 0.0; % 万向节角度噪声标准差 (°)
    sensor_noise_std = 0.00; % 传感器角度噪声标准差 (°)
    sensor_rate_noise_std = 0.0; % 传感器角速度噪声标准差 (°/s)

    %% 初始化系统
    % 初始化PID控制器和滤波器
    pid_outer = initPIDController(Kp_outer, Ki_outer, Kd_outer, dt_control, ...
        integral_max_outer, integral_min_outer, alpha_d_outer, alpha_output_outer);
    pid_inner = initPIDController(Kp_inner, Ki_inner, Kd_inner, dt_control, ...
        integral_max_inner, integral_min_inner, alpha_d_inner, alpha_output_inner);
    lpf = initLowPassFilter(alpha);

    % 初始化执行机构状态
    gimbal_angle_buffer = zeros(1, delay_steps); % 延迟缓冲区
    current_gimbal_angle_delayed = 0; % 延迟后的角度
    current_gimbal_angle_actual = 0; % 滞后后的实际角度

    % 预分配控制数据数组（250Hz）
    time_list_control = zeros(1, control_steps);
    pitch_angle_list_control = zeros(1, control_steps);
    pitch_angle_measured_list_control = zeros(1, control_steps);
    pitch_rate_list_control = zeros(1, control_steps);
    pitch_rate_measured_list_control = zeros(1, control_steps);
    gimbal_angle_list_control = zeros(1, control_steps);
    gimbal_angle_actual_list_control = zeros(1, control_steps);
    mass_list_control = zeros(1, control_steps);
    inertia_list_control = zeros(1, control_steps);
    target_pitch_angle_list_control = zeros(1, control_steps);
    filtered_target_pitch_angle_list_control = zeros(1, control_steps);
    omega_ref_list_control = zeros(1, control_steps);
    error_outer_list_control = zeros(1, control_steps);
    error_inner_list_control = zeros(1, control_steps);
    gimbal_to_cg_list_control = zeros(1, control_steps);

    % 动力学数据（1kHz，动态扩展）
    time_list = [];
    x_position_list = [];
    y_position_list = [];
    x_velocity_list = [];
    y_velocity_list = [];
    pitch_acceleration_list = [];

    % 系统初始状态
    current_pitch_angle = 0;       % 初始俯仰角 (°)
    current_pitch_rate = 0;        % 初始角速度 (°/s)
    x_position = 0;          % 初始水平位置 (m)
    y_position = 0;          % 初始竖直位置 (m)
    x_velocity = 0;          % 初始水平速度 (m/s)
    y_velocity = 0;          % 初始竖直速度 (m/s)
    current_mass = initial_mass;
    current_inertia = inertia_full;
    current_gimbal_to_cg = gimbal_to_cg_full;
    filtered_target_pitch_angle = 0;
    target_pitch_angle = 0;
    latest_pitch_angle_measured = 0;
    latest_pitch_rate_measured = 0;

    % 初始状态向量
    state = [current_pitch_angle; current_pitch_rate; x_position; x_velocity; y_position; y_velocity];

    %% 主仿真循环
    t = 0; % 当前仿真时间 (s)
    step = 1; % 控制步数计数器

    while t < time_duration
        % 控制更新（250Hz）
        if step <= control_steps && abs(t - (step-1)*dt_control) < 1e-10
            t_control = (step - 1) * dt_control;
            % 生成目标俯仰角：前6秒5°，后6秒0°
            target_pitch_angle = 5*(t_control<6) + 0*(t_control>=6);
            lpf = updateLowPassFilter(lpf, target_pitch_angle);
            filtered_target_pitch_angle = lpf.y_prev;

            % 模拟传感器测量，添加噪声
            sensor_noise = sensor_noise_std * randn;
            current_pitch_angle_measured = current_pitch_angle + sensor_noise;
            latest_pitch_angle_measured = current_pitch_angle_measured;
            rate_noise = sensor_rate_noise_std * randn;
            current_pitch_rate_measured = current_pitch_rate + rate_noise;
            latest_pitch_rate_measured = current_pitch_rate_measured;

            % 外环PID：计算角速度指令
            theta_error = filtered_target_pitch_angle - current_pitch_angle_measured;
            [pid_outer, omega_ref] = updatePIDController(pid_outer, theta_error);
            omega_ref= max(min(omega_ref, max_omega), -max_omega);

            % 内环PID：计算万向节角度
            omega_error = omega_ref - current_pitch_rate_measured;
            [pid_inner, control] = updatePIDController(pid_inner, omega_error);

            % 执行机构延迟与滞后
            current_gimbal_angle = max(min(control, max_gimbal_angle), -max_gimbal_angle);
            gimbal_angle_buffer = [current_gimbal_angle, gimbal_angle_buffer(1:end-1)];
            current_gimbal_angle_delayed = gimbal_angle_buffer(end);
            current_gimbal_angle_actual = (1 - alpha_actuator) * current_gimbal_angle_actual + ...
                alpha_actuator * current_gimbal_angle_delayed;
            gimbal_angle_noise = gimbal_angle_noise_std * randn;
            current_gimbal_angle_actual = current_gimbal_angle_actual + gimbal_angle_error + gimbal_angle_noise;

            % 存储控制数据
            time_list_control(step) = t_control;
            pitch_angle_list_control(step) = current_pitch_angle;
            pitch_angle_measured_list_control(step) = latest_pitch_angle_measured;
            pitch_rate_list_control(step) = current_pitch_rate;
            pitch_rate_measured_list_control(step) = latest_pitch_rate_measured;
            gimbal_angle_list_control(step) = current_gimbal_angle;
            gimbal_angle_actual_list_control(step) = current_gimbal_angle_actual;
            mass_list_control(step) = current_mass;
            inertia_list_control(step) = current_inertia;
            target_pitch_angle_list_control(step) = target_pitch_angle;
            filtered_target_pitch_angle_list_control(step) = filtered_target_pitch_angle;
            omega_ref_list_control(step) = omega_ref;
            error_outer_list_control(step) = theta_error;
            error_inner_list_control(step) = omega_error;
            gimbal_to_cg_list_control(step) = current_gimbal_to_cg;

            step = step + 1;
        end

        % 动力学更新（1kHz）
        t_next = min(t + dt_dynamics, time_duration);

        % 计算燃料消耗
        if fuel_mass > 0
            current_mass_flow_rate = max_mass_flow_rate * (manual_thrust/max_thrust);
            fuel_mass = fuel_mass - current_mass_flow_rate * dt_dynamics;
            fuel_mass = max(fuel_mass, 0);
            current_mass = empty_mass + fuel_mass;
        end

        % 根据燃料消耗插值质心位置和转动惯量
        mass_ratio = (initial_mass - current_mass)/(initial_mass - empty_mass);
        current_gimbal_to_cg = gimbal_to_cg_full - mass_ratio*(gimbal_to_cg_full - gimbal_to_cg_empty);
        current_inertia = inertia_full - mass_ratio*(inertia_full - inertia_empty);

        % 使用RK4更新动力学状态
        [state, current_pitch_acceleration] = updateDynamicsRK4(state, manual_thrust, ...
            current_gimbal_angle_actual, current_mass, current_inertia, current_gimbal_to_cg, dt_dynamics);

        % 更新状态变量
        current_pitch_angle = state(1);
        current_pitch_rate = state(2);
        x_position = state(3);
        x_velocity = state(4);
        y_position = state(5);
        y_velocity = state(6);

        % 存储动力学数据
        time_list(end+1) = t;
        x_position_list(end+1) = x_position;
        y_position_list(end+1) = y_position;
        x_velocity_list(end+1) = x_velocity;
        y_velocity_list(end+1) = y_velocity;
        pitch_acceleration_list(end+1) = current_pitch_acceleration;

        % 时间推进
        t = t_next;
    end

    %% 可视化结果
    figure('Name', '火箭姿态与平动仿真（带执行机构延迟和滞后）', 'NumberTitle', 'off', ...
        'Position', [100 100 1200 900], 'Color', 'w');

    % 俯仰角跟踪性能
    subplot(3,3,1);
    plot(time_list_control, pitch_angle_list_control, 'b', 'LineWidth', 1.5);
    hold on;
    plot(time_list_control, pitch_angle_measured_list_control, 'Color', [1 0.5 0], 'LineWidth', 1);
    plot(time_list_control, target_pitch_angle_list_control, '--g', 'LineWidth', 1.5);
    plot(time_list_control, filtered_target_pitch_angle_list_control, '--m', 'LineWidth', 1.5);
    title('俯仰角跟踪性能');
    xlabel('时间 (s)'); ylabel('角度 (°)');
    legend('实际值', '测量值', '目标值', '滤波目标', 'Location', 'best');
    grid on;

    % 控制误差分析（外环）
    subplot(3,3,2);
    plot(time_list_control, error_outer_list_control, 'r', 'LineWidth', 1.5);
    title('外环控制误差分析');
    xlabel('时间 (s)'); ylabel('误差 (°)');
    grid on;

    % 发动机摆角响应
    subplot(3,3,4);
    plot(time_list_control, gimbal_angle_list_control, 'g', 'LineWidth', 1.5);
    hold on;
    plot(time_list_control, gimbal_angle_actual_list_control, 'Color', [1 0.5 0], 'LineWidth', 1);
    title('发动机摆角响应');
    xlabel('时间 (s)'); ylabel('摆角 (°)');
    legend('理论值', '实际值', 'Location', 'best');
    grid on;

    % 质心水平位置
    subplot(3,3,8);
    plot(time_list, x_position_list, 'b', 'LineWidth', 1.5);
    title('质心水平位置');
    xlabel('时间 (s)'); ylabel('位置 (m)');
    grid on;

    % 质心竖直位置
    subplot(3,3,5);
    plot(time_list, y_position_list, 'r', 'LineWidth', 1.5);
    title('质心竖直位置');
    xlabel('时间 (s)'); ylabel('位置 (m)');
    grid on;

    % 质心水平速度
    subplot(3,3,6);
    plot(time_list, x_velocity_list, 'g', 'LineWidth', 1.5);
    title('质心水平速度');
    xlabel('时间 (s)'); ylabel('速度 (m/s)');
    grid on;

    % 质心竖直速度
    subplot(3,3,7);
    plot(time_list, y_velocity_list, 'm', 'LineWidth', 1.5);
    title('质心竖直速度');
    xlabel('时间 (s)'); ylabel('速度 (m/s)');
    grid on;

    % 角速度跟踪性能
    subplot(3,3,3);
    plot(time_list_control, pitch_rate_list_control, 'b', 'LineWidth', 1.5);
    hold on;
    plot(time_list_control, omega_ref_list_control, '--r', 'LineWidth', 1.5);
    title('角速度跟踪性能');
    xlabel('时间 (s)'); ylabel('角速度 (°/s)');
    legend('实际值', '指令值', 'Location', 'best');
    grid on;

    % 质心位置变化
    subplot(3,3,9);
    plot(time_list_control, mass_list_control, 'Color', [0 0.5 0.5], 'LineWidth', 1.5);
    title('火箭质量变化');
    xlabel('时间 (s)'); ylabel('质量(kg)');
    grid on;

    sgtitle('火箭俯仰姿态与质心平动联合仿真结果 (RK4 + 内外环PID + 执行机构延迟与滞后)', 'FontSize', 16);

    % 计算并显示均方误差（RMSE）
    rmse = sqrt(mean(error_outer_list_control.^2));
    disp(['角度跟踪均方误差 (RMSE): ', num2str(rmse), '°']);
end
