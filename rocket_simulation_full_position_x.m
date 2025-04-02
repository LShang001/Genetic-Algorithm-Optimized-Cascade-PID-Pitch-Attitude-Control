% 文件名：rocket_simulation_full_position_x.m
% 功能：完整火箭水平位置控制仿真，使用优化后的位置环PID参数和固定的姿态环PID参数
%       可视化水平位置跟踪性能、控制指令和各状态变量

function rocket_simulation_full_position_x(position_pid_params)
    %% 初始化环境
    set(0, 'DefaultAxesFontName', 'SimHei');
    set(0, 'DefaultTextFontName', 'SimHei');
    set(0, 'DefaultFigureColor', 'w');

    %% 参数配置
    % 如果未提供位置环PID参数，尝试从文件加载
    if nargin == 0

        try
            loaded_data = load('optimal_position_x_de_params.mat', 'optimal_params');
            % position_pid_params = loaded_data.optimal_params;
            position_pid_params = [1.2, 0.0, 0.00, 13, 0.0, 0.0];
            disp('成功加载优化后的水平位置环PID参数。');
        catch
            warning('无法加载 optimal_position_x_de_params.mat，将使用默认位置环PID参数。');
            position_pid_params = [1.4771, 0.0000, 1.6486, 3.3987, 0.0000, 0.0000]; % 修改后的默认值
        end

    end

    % 加载固定的姿态环PID参数
    try
        att_data = load('optimal_adaptive_de_params.mat', 'optimal_params');
        att_params = att_data.optimal_params;
        % Kp_outer_att = att_params(1); Ki_outer_att = att_params(2); Kd_outer_att = att_params(3);
        % Kp_inner_att = att_params(4); Ki_inner_att = att_params(5); Kd_inner_att = att_params(6);
        Kp_outer_att = 3.9014; Ki_outer_att = 0; Kd_outer_att = 0;
        Kp_inner_att = 1.1355; Ki_inner_att = 3.0861; Kd_inner_att = 0.033825;
        disp('成功加载固定的姿态PID参数。');
    catch
        warning('无法加载 optimal_adaptive_de_params.mat，将使用默认姿态PID参数。');
        Kp_outer_att = 3.9014; Ki_outer_att = 0; Kd_outer_att = 0;
        Kp_inner_att = 1.1355; Ki_inner_att = 3.0861; Kd_inner_att = 0.033825;
    end

    % 从输入数组提取水平位置和速度环PID参数
    Kp_pos_x = position_pid_params(1);
    Ki_pos_x = position_pid_params(2);
    Kd_pos_x = position_pid_params(3);
    Kp_vel_x = position_pid_params(4);
    Ki_vel_x = position_pid_params(5);
    Kd_vel_x = position_pid_params(6);

    %% 仿真参数
    dt_control = 0.004; % 控制周期 (s)，250Hz
    dt_dynamics = 0.001; % 动力学更新步长 (s)，1kHz
    time_duration = 12; % 仿真总时长 (s)
    control_steps = ceil(time_duration / dt_control); % 总控制步数

    % 火箭物理参数
    initial_mass = 15.8; % 初始质量 (kg)
    fuel_mass = 3.0; % 初始燃料质量 (kg)
    empty_mass = 12.8; % 空载质量 (kg)
    max_thrust = 200; % 最大推力 (N)
    max_mass_flow_rate = 0.12; % 最大质量流率 (kg/s)
    gimbal_to_cg_full = 0.58; % 满载时质心距离 (m)
    gimbal_to_cg_empty = 0.50; % 空载时质心距离 (m)
    inertia_full = 2.76; % 满载时转动惯量 (kg·m²)
    inertia_empty = 2.1; % 空载时转动惯量 (kg·m²)
    max_gimbal_angle = 15; % 最大摆角 (°)
    max_omega_att = 80; % 姿态内环输出最大角速度 (°/s)
    manual_thrust = 160; % 固定推力 (N)

    % 水平位置控制参数
    max_target_velocity_x = 3.0; % 水平位置环输出的最大目标速度 (m/s)
    min_target_velocity_x = -3.0; % 水平位置环输出的最小目标速度 (m/s)
    max_target_pitch = 20; % 速度环输出的最大目标俯仰角 (°)
    min_target_pitch = -20; % 速度环输出的最小目标俯仰角 (°)

    % PID控制器参数 (限幅和滤波)
    integral_max_pos_x = 50; integral_min_pos_x = -50; % 水平位置环积分限幅 (m*s)
    integral_max_vel_x = 50; integral_min_vel_x = -50; % 水平速度环积分限幅 (deg*s)
    integral_max_att_outer = 100; integral_min_att_outer = -100; % 固定姿态外环积分限幅
    integral_max_att_inner = 100; integral_min_att_inner = -100; % 固定姿态内环积分限幅
    alpha_d_pos_x = 1; alpha_output_pos_x = 1; % 水平位置环滤波系数 (1=无滤波)
    alpha_d_vel_x = 1; alpha_output_vel_x = 1; % 水平速度环滤波系数 (1=无滤波)
    alpha_d_att_outer = 1; alpha_output_att_outer = 1; % 固定姿态外环滤波
    alpha_d_att_inner = 1; alpha_output_att_inner = 1; % 固定姿态内环滤波

    % 执行机构参数
    delay_time = 0.01; % 执行机构延迟时间 (s)
    delay_steps = round(delay_time / dt_control); % 延迟步数
    tau_actuator = 0.08; % 执行机构时间常数 (s)
    alpha_actuator = dt_control / (tau_actuator + dt_control); % 一阶滞后系数

    % 噪声参数
    gimbal_angle_error = 0.20; % 万向节角度固定误差 (°)
    gimbal_angle_noise_std = 0.0; % 万向节角度噪声标准差 (°)
    sensor_noise_std = 0.0; % 传感器角度噪声标准差 (°)
    sensor_rate_noise_std = 0.0; % 传感器角速度噪声标准差 (°/s)

    %% 初始化系统
    % 初始化所有PID控制器
    pid_pos_x = initPIDController(Kp_pos_x, Ki_pos_x, Kd_pos_x, dt_control, ...
        integral_max_pos_x, integral_min_pos_x, alpha_d_pos_x, alpha_output_pos_x);
    pid_vel_x = initPIDController(Kp_vel_x, Ki_vel_x, Kd_vel_x, dt_control, ...
        integral_max_vel_x, integral_min_vel_x, alpha_d_vel_x, alpha_output_vel_x);
    pid_att_outer = initPIDController(Kp_outer_att, Ki_outer_att, Kd_outer_att, dt_control, ...
        integral_max_att_outer, integral_min_att_outer, alpha_d_att_outer, alpha_output_att_outer);
    pid_att_inner = initPIDController(Kp_inner_att, Ki_inner_att, Kd_inner_att, dt_control, ...
        integral_max_att_inner, integral_min_att_inner, alpha_d_att_inner, alpha_output_att_inner);

    % 初始化执行机构状态
    gimbal_angle_buffer = zeros(1, delay_steps);
    current_gimbal_angle_delayed = 0;
    current_gimbal_angle_actual = 0;

    % 预分配存储数组 (控制周期数据)
    time_list_control = zeros(1, control_steps);
    x_position_list_control = zeros(1, control_steps);
    x_velocity_list_control = zeros(1, control_steps);
    target_x_position_list_control = zeros(1, control_steps);
    target_x_velocity_list_control = zeros(1, control_steps);
    target_pitch_angle_list_control = zeros(1, control_steps);
    pitch_angle_list_control = zeros(1, control_steps);
    pitch_rate_list_control = zeros(1, control_steps);
    gimbal_angle_list_control = zeros(1, control_steps);
    gimbal_angle_actual_list_control = zeros(1, control_steps);
    mass_list_control = zeros(1, control_steps);

    % 预分配存储数组 (动力学周期数据)
    time_list = [];
    x_position_list = [];
    y_position_list = [];
    x_velocity_list = [];
    y_velocity_list = [];
    pitch_angle_list = [];
    pitch_rate_list = [];
    pitch_acceleration_list = [];

    % 系统初始状态
    state = [0.0; 0; 0; 0.0; 0; 0]; % [俯仰角(°); 角速度(°/s); x位置(m); x速度(m/s); y位置(m); y速度(m/s)]
    current_mass = initial_mass;
    current_inertia = inertia_full;
    current_gimbal_to_cg = gimbal_to_cg_full;

    %% 主仿真循环
    t = 0; step = 1;

    while t < time_duration
        % 控制更新（250Hz）
        if step <= control_steps && abs(t - (step - 1) * dt_control) < 1e-10
            t_control = (step - 1) * dt_control;

            % --- 目标位置生成 ---
            % 前1秒保持原位，然后移动到X=1m
            if t_control < 1.0
                target_x_position = 0;
            else
                target_x_position = 1.0;
            end

            % --- 控制逻辑 ---
            % 1. 水平位置环 (X-Position PID)
            position_error_x = target_x_position - state(3);
            [pid_pos_x, target_x_velocity] = updatePIDController(pid_pos_x, position_error_x);
            % target_x_velocity = 5; % 测试用：将目标速度固定为5m/s，可根据实际情况修改此值以进行不同条件下的测试
            target_x_velocity = max(min(target_x_velocity, max_target_velocity_x), min_target_velocity_x);

            % 2. 水平速度环 (X-Velocity PID)
            velocity_error_x = target_x_velocity - state(4);
            [pid_vel_x, target_pitch_angle_cmd] = updatePIDController(pid_vel_x, velocity_error_x);
            target_pitch_angle = -target_pitch_angle_cmd; % 正误差 -> 负俯仰角指令 (向右倾斜)
            % target_pitch_angle = 10; % 测试用：将目标俯仰角固定为10°，可根据实际情况修改此值以进行不同条件下的测试
            target_pitch_angle = max(min(target_pitch_angle, max_target_pitch), min_target_pitch);

            % 3. 姿态外环 (Attitude Angle PID)
            % 添加传感器噪声
            pitch_angle_measured = state(1) + sensor_noise_std * randn;
            pitch_rate_measured = state(2) + sensor_rate_noise_std * randn;

            theta_error = target_pitch_angle - pitch_angle_measured;
            [pid_att_outer, omega_ref] = updatePIDController(pid_att_outer, theta_error);
            omega_ref = max(min(omega_ref, max_omega_att), -max_omega_att);

            % 4. 姿态内环 (Attitude Rate PID)
            omega_error = omega_ref - pitch_rate_measured;
            [pid_att_inner, control] = updatePIDController(pid_att_inner, omega_error);
            current_gimbal_angle = max(min(control, max_gimbal_angle), -max_gimbal_angle);

            % 5. 执行机构模型 (延迟 + 滞后 + 噪声)
            gimbal_angle_buffer = [current_gimbal_angle, gimbal_angle_buffer(1:end - 1)];
            current_gimbal_angle_delayed = gimbal_angle_buffer(end);
            current_gimbal_angle_actual = (1 - alpha_actuator) * current_gimbal_angle_actual + ...
                alpha_actuator * current_gimbal_angle_delayed;
            gimbal_angle_noise = gimbal_angle_noise_std * randn;
            current_gimbal_angle_actual = current_gimbal_angle_actual + gimbal_angle_error + gimbal_angle_noise;

            % 存储控制周期数据
            time_list_control(step) = t_control; % 记录当前控制时间
            x_position_list_control(step) = state(3); % 记录当前水平位置
            x_velocity_list_control(step) = state(4); % 记录当前水平速度
            target_x_position_list_control(step) = target_x_position; % 记录当前目标水平位置
            target_x_velocity_list_control(step) = target_x_velocity; % 记录当前目标水平速度
            target_pitch_angle_list_control(step) = target_pitch_angle; % 记录当前目标俯仰角
            pitch_angle_list_control(step) = state(1); % 记录当前实际俯仰角
            pitch_rate_list_control(step) = state(2); % 记录当前俯仰角速度
            gimbal_angle_list_control(step) = current_gimbal_angle; % 记录当前发动机摆角指令
            gimbal_angle_actual_list_control(step) = current_gimbal_angle_actual; % 记录当前发动机实际摆角
            mass_list_control(step) = current_mass; % 记录当前火箭质量

            step = step + 1;
        end

        % 动力学更新（1kHz）
        t_next = min(t + dt_dynamics, time_duration);

        % 计算燃料消耗
        if fuel_mass > 0
            current_mass_flow_rate = max_mass_flow_rate * (manual_thrust / max_thrust);
            fuel_mass = max(fuel_mass - current_mass_flow_rate * dt_dynamics, 0);
            current_mass = empty_mass + fuel_mass;
        end

        % 根据燃料消耗插值质心位置和转动惯量
        mass_ratio = (initial_mass - current_mass) / (initial_mass - empty_mass);
        current_gimbal_to_cg = gimbal_to_cg_full - mass_ratio * (gimbal_to_cg_full - gimbal_to_cg_empty);
        current_inertia = inertia_full - mass_ratio * (inertia_full - inertia_empty);

        % 使用RK4更新动力学状态
        [state, pitch_acceleration] = updateDynamicsRK4(state, manual_thrust, ...
            current_gimbal_angle_actual, current_mass, current_inertia, current_gimbal_to_cg, dt_dynamics);

        % 简单的地面碰撞检测
        if state(5) < 0 && state(6) < 0 % 如果低于地面且向下运动
            state(5) = 0; % 停在地面
            state(6) = 0; % 垂直速度为0
        end

        % 存储动力学数据
        time_list(end + 1) = t;
        x_position_list(end + 1) = state(3);
        y_position_list(end + 1) = state(5);
        x_velocity_list(end + 1) = state(4);
        y_velocity_list(end + 1) = state(6);
        pitch_angle_list(end + 1) = state(1);
        pitch_rate_list(end + 1) = state(2);
        pitch_acceleration_list(end + 1) = pitch_acceleration;

        % 时间推进
        t = t_next;
    end

    %% 可视化结果
    figure('Name', '火箭水平位置控制仿真结果', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1400, 900], 'Color', 'w');

    % 1. 水平位置跟踪性能
    subplot(3, 3, 1);
    plot(time_list_control, x_position_list_control, 'b', 'LineWidth', 1.5);
    hold on;
    plot(time_list_control, target_x_position_list_control, '--r', 'LineWidth', 1.5);
    title('水平位置(X)跟踪性能');
    xlabel('时间 (s)'); ylabel('位置 (m)');
    legend('实际位置', '目标位置', 'Location', 'best');
    grid on;

    % 2. 水平速度跟踪
    subplot(3, 3, 2);
    plot(time_list_control, x_velocity_list_control, 'b', 'LineWidth', 1.5);
    hold on;
    plot(time_list_control, target_x_velocity_list_control, '--r', 'LineWidth', 1.5);
    title('水平速度(Vx)跟踪');
    xlabel('时间 (s)'); ylabel('速度 (m/s)');
    legend('实际速度', '目标速度', 'Location', 'best');
    grid on;

    % 3. 俯仰角跟踪
    subplot(3, 3, 3);
    plot(time_list_control, pitch_angle_list_control, 'b', 'LineWidth', 1.5);
    hold on;
    plot(time_list_control, target_pitch_angle_list_control, '--r', 'LineWidth', 1.5);
    title('俯仰角跟踪');
    xlabel('时间 (s)'); ylabel('角度 (°)');
    legend('实际角度', '目标角度', 'Location', 'best');
    grid on;

    % 4. 发动机摆角
    subplot(3, 3, 4);
    plot(time_list_control, gimbal_angle_list_control, 'g', 'LineWidth', 1.5);
    hold on;
    plot(time_list_control, gimbal_angle_actual_list_control, 'Color', [1 0.5 0], 'LineWidth', 1.5);
    title('发动机摆角');
    xlabel('时间 (s)'); ylabel('角度 (°)');
    legend('指令角度', '实际角度', 'Location', 'best');
    grid on;

    % 5. 角速度
    subplot(3, 3, 5);
    plot(time_list_control, pitch_rate_list_control, 'm', 'LineWidth', 1.5);
    title('俯仰角速度');
    xlabel('时间 (s)'); ylabel('角速度 (°/s)');
    grid on;

    % 6. 角加速度
    subplot(3, 3, 6);
    plot(time_list, pitch_acceleration_list, 'k', 'LineWidth', 1.5);
    title('俯仰角加速度');
    xlabel('时间 (s)'); ylabel('角加速度 (°/s²)');
    grid on;

    % 7. 垂直位置(Y)
    subplot(3, 3, 7);
    plot(time_list, y_position_list, 'Color', [0.5 0 0.5], 'LineWidth', 1.5);
    title('垂直位置(Y)');
    xlabel('时间 (s)'); ylabel('高度 (m)');
    grid on;

    % 8. 垂直速度(Vy)
    subplot(3, 3, 8);
    plot(time_list, y_velocity_list, 'Color', [0.5 0 0.5], 'LineWidth', 1.5);
    title('垂直速度(Vy)');
    xlabel('时间 (s)'); ylabel('速度 (m/s)');
    grid on;

    % 9. 质量变化
    subplot(3, 3, 9);
    plot(time_list_control, mass_list_control, 'Color', [0 0.5 0.5], 'LineWidth', 1.5);
    title('火箭质量变化');
    xlabel('时间 (s)'); ylabel('质量 (kg)');
    grid on;

    % 添加总标题
    sgtitle('火箭水平位置控制完整仿真结果', 'FontSize', 16);

    %% 计算并显示性能指标
    % 计算稳定时间 (到达并保持在目标位置±2%范围内的时间)
    target_reached = false;
    settling_time = time_duration;
    settling_threshold = 0.02 * 2; % 2 % of 2m target

    for i = 1:length(time_list_control)

        if ~target_reached && abs(x_position_list_control(i) - 2) <= settling_threshold
            target_reached = true;
            settling_time = time_list_control(i);
        elseif target_reached && abs(x_position_list_control(i) - 2) > settling_threshold
            target_reached = false;
        end

    end

    % 计算超调量
    overshoot = max(0, max(x_position_list_control) - 2);

    % 计算稳态误差
    steady_state_error = mean(abs(x_position_list_control(end - 100:end) - 2)); % 最后100个控制周期

    % 显示性能指标
    disp('=== 性能指标 ===');
    disp(['稳定时间 (2%准则): ', num2str(settling_time), ' s']);
    disp(['最大超调量: ', num2str(overshoot), ' m']);
    disp(['稳态误差: ', num2str(steady_state_error), ' m']);

    %% 辅助函数
    function pid = initPIDController(Kp, Ki, Kd, dt, integral_max, integral_min, alpha_d, alpha_output)
        pid = struct( ...
            'Kp', Kp, 'Ki', Ki, 'Kd', Kd, 'dt', dt, ...
            'integral', 0, 'prev_error', 0, ...
            'integral_max', integral_max, 'integral_min', integral_min, ...
            'lpf_d', initLowPassFilter(alpha_d), ...
            'lpf_output', initLowPassFilter(alpha_output));
    end

    function lpf = initLowPassFilter(alpha)
        lpf.alpha = alpha;
        lpf.y_prev = 0;
    end

    % 功能：更新低通滤波器的状态
    % 输入参数：
    %   lpf：低通滤波器结构体，包含alpha和y_prev
    %   x：当前输入信号
    % 输出参数：
    %   lpf：更新后的低通滤波器结构体
    function lpf = updateLowPassFilter(lpf, x)
        % 根据低通滤波器的公式更新输出值
        % lpf.alpha * x 表示当前输入信号的加权值
        % (1 - lpf.alpha) * lpf.y_prev 表示上一次输出值的加权值
        lpf.y_prev = lpf.alpha * x + (1 - lpf.alpha) * lpf.y_prev;
    end

    function [pid, control_filtered] = updatePIDController(pid, error)
        P = pid.Kp * error;
        pid.integral = pid.integral + error * pid.dt;
        pid.integral = max(min(pid.integral, pid.integral_max), pid.integral_min);
        I = pid.Ki * pid.integral;
        derivative = (error - pid.prev_error) / pid.dt;
        pid.lpf_d = updateLowPassFilter(pid.lpf_d, derivative);
        D = pid.Kd * pid.lpf_d.y_prev;
        control = P + I + D;
        pid.lpf_output = updateLowPassFilter(pid.lpf_output, control);
        control_filtered = pid.lpf_output.y_prev;
        pid.prev_error = error;
    end

    function [state_next, pitch_acceleration] = updateDynamicsRK4(state, thrust, gimbal_angle, mass, inertia, gimbal_to_cg, dt)

        function dxdt = dynamics(x, thrust, gimbal_angle, mass, inertia, gimbal_to_cg)
            theta = x(1); % 俯仰角 (°)
            omega = x(2); % 角速度 (°/s)
            x_vel = x(4); % x速度 (m/s)
            y_vel = x(6); % y速度 (m/s)

            % 推力方向（x轴正方向向左）
            thrust_angle = theta - gimbal_angle;
            thrust_x = -thrust * sind(thrust_angle); % 正为左
            thrust_y = thrust * cosd(thrust_angle); % 正为上

            % 加速度计算
            acc_x = thrust_x / mass;
            acc_y = (thrust_y - 9.81 * mass) / mass;

            % 力矩和角加速度
            torque = thrust * gimbal_to_cg * sind(gimbal_angle); % 正力矩
            alpha = rad2deg(torque / inertia); % 角加速度 deg/s²

            dxdt = [omega; alpha; x_vel; acc_x; y_vel; acc_y];
        end

        % RK4积分
        k1 = dynamics(state, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);
        k2 = dynamics(state + dt / 2 * k1, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);
        k3 = dynamics(state + dt / 2 * k2, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);
        k4 = dynamics(state + dt * k3, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);

        state_next = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
        torque = thrust * gimbal_to_cg * sind(gimbal_angle);
        pitch_acceleration = rad2deg(torque / inertia);
    end

end
