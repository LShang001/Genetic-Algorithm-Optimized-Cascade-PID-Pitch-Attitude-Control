% 文件名：rocket_simulation_fitness_position_x.m
% 作者：AI Assistant based on LShang's work
% 版本：1.0
% 功能：简化版火箭仿真，用于优化外层水平位置(X)和速度(Vx)环PID参数。
%       内层姿态环PID参数从文件加载，保持固定。垂直方向(Y)不受控。
%       计算适应度值，衡量水平位置跟踪误差、超调和姿态指令幅度。
% 输入：params - 水平位置和速度环PID参数数组 [Kp_pos_x, Ki_pos_x, Kd_pos_x, Kp_vel_x, Ki_vel_x, Kd_vel_x]
% 输出：fitness - 加权适应度值

function fitness = rocket_simulation_fitness_position_x(params)
    %% 加载已优化的内环（姿态）PID参数
    try
        loaded_data = load('optimal_adaptive_de_params.mat', 'optimal_params');
        att_params = loaded_data.optimal_params;
        Kp_outer_att = att_params(1); Ki_outer_att = att_params(2); Kd_outer_att = att_params(3);
        Kp_inner_att = att_params(4); Ki_inner_att = att_params(5); Kd_inner_att = att_params(6);
        % disp('成功加载固定的姿态PID参数。'); % Optional: Keep console clean during optimization
    catch
        warning('无法加载 optimal_adaptive_de_params.mat，将使用默认姿态PID参数。');
        Kp_outer_att = 4.9006; Ki_outer_att = 0; Kd_outer_att = 0;
        Kp_inner_att = 0.77848; Ki_inner_att = 2.45; Kd_inner_att = 0.026142;
    end

    %% 初始化参数
    % 从输入数组提取水平位置环和速度环PID控制器的增益参数
    Kp_pos_x = params(1); % 水平位置环 Kp (m error -> m/s command)
    Ki_pos_x = params(2); % 水平位置环 Ki
    Kd_pos_x = params(3); % 水平位置环 Kd
    Kp_vel_x = params(4); % 水平速度环 Kp (m/s error -> deg command)
    Ki_vel_x = params(5); % 水平速度环 Ki
    Kd_vel_x = params(6); % 水平速度环 Kd

    %% 子函数定义 (与之前类似)
    % 低通滤波器初始化函数
    function lpf = initLowPassFilter(alpha)
        lpf.alpha = alpha; lpf.y_prev = 0;
    end

    % 低通滤波器更新函数
    function lpf = updateLowPassFilter(lpf, x)
        lpf.y_prev = lpf.alpha * x + (1 - lpf.alpha) * lpf.y_prev;
    end

    % PID控制器初始化函数
    function pid = initPIDController(Kp, Ki, Kd, dt, integral_max, integral_min, alpha_d, alpha_output)
        pid = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd, 'dt', dt, 'integral', 0, 'prev_error', 0, ...
            'integral_max', integral_max, 'integral_min', integral_min, ...
            'lpf_d', initLowPassFilter(alpha_d), 'lpf_output', initLowPassFilter(alpha_output));
    end

    % PID控制器更新函数
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

    % RK4动力学更新函数 (不变)
    function state_next = updateDynamicsRK4(state, thrust, gimbal_angle, mass, inertia, gimbal_to_cg, dt)

        function dxdt = dynamics(x, thrust, gimbal_angle, mass, inertia, gimbal_to_cg)
            theta = x(1); omega = x(2); x_vel = x(4); y_vel = x(6);
            thrust_angle = theta - gimbal_angle;
            thrust_x = -thrust * sind(thrust_angle); % 正为左
            thrust_y = thrust * cosd(thrust_angle); % 正为上
            acc_x = thrust_x / mass;
            acc_y = (thrust_y - 9.81 * mass) / mass;
            torque = thrust * gimbal_to_cg * sind(gimbal_angle); % 正力矩 (顺时针摆角产生逆时针力矩)
            alpha = rad2deg(torque / inertia); % 角加速度 deg/s^2
            dxdt = [omega; alpha; x_vel; acc_x; y_vel; acc_y];
        end

        k1 = dynamics(state, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);
        k2 = dynamics(state + dt / 2 * k1, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);
        k3 = dynamics(state + dt / 2 * k2, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);
        k4 = dynamics(state + dt * k3, thrust, gimbal_angle, mass, inertia, gimbal_to_cg);
        state_next = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    end

    %% 主程序参数配置
    dt_control = 0.004; % 控制周期 (s)，250Hz
    dt_dynamics = 0.001; % 动力学更新步长 (s)，1kHz
    time_duration = 8; % 仿真总时长 (s)，调整以观察水平移动效果
    control_steps = ceil(time_duration / dt_control); % 总控制步数
    initial_mass = 15.8; % 初始质量 (kg)
    fuel_mass = 3.0; % 初始燃料质量 (kg)
    empty_mass = 12.8; % 空载质量 (kg)
    max_thrust = 200; % 最大推力 (N)
    max_mass_flow_rate = 0.12; % 最大质量流率 (kg/s)
    gimbal_to_cg_full = 0.58; % 满载质心距 (m)
    gimbal_to_cg_empty = 0.50; % 空载质心距 (m)
    inertia_full = 2.76; % 满载转动惯量 (kg·m²)
    inertia_empty = 2.1; % 空载转动惯量 (kg·m²)
    max_gimbal_angle = 15; % 最大摆角 (°)
    max_omega_att = 80; % 姿态内环输出最大角速度 (°/s) - 固定值
    manual_thrust = 160; % 固定推力 (N) - 注意: 需大于重力才能提供机动能力

    % 水平位置和速度环相关参数
    max_target_velocity_x = 3.0; % 水平位置环输出的最大目标速度 (m/s)
    min_target_velocity_x = -3.0; % 水平位置环输出的最小目标速度 (m/s)
    max_target_pitch = 20; % 速度环输出的最大目标俯仰角 (°) - 重要约束
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

    % 执行机构参数 (不变)
    delay_time = 0.01;
    delay_steps = round(delay_time / dt_control);
    tau_actuator = 0.08;
    alpha_actuator = dt_control / (tau_actuator + dt_control);

    % 噪声参数 (不变)
    gimbal_angle_error = 0.1;
    gimbal_angle_noise_std = 0.0;

    %% 初始化系统
    % 初始化所有PID控制器
    pid_pos_x = initPIDController(Kp_pos_x, Ki_pos_x, Kd_pos_x, dt_control, integral_max_pos_x, integral_min_pos_x, alpha_d_pos_x, alpha_output_pos_x);
    pid_vel_x = initPIDController(Kp_vel_x, Ki_vel_x, Kd_vel_x, dt_control, integral_max_vel_x, integral_min_vel_x, alpha_d_vel_x, alpha_output_vel_x);
    pid_att_outer = initPIDController(Kp_outer_att, Ki_outer_att, Kd_outer_att, dt_control, integral_max_att_outer, integral_min_att_outer, alpha_d_att_outer, alpha_output_att_outer); % 固定参数
    pid_att_inner = initPIDController(Kp_inner_att, Ki_inner_att, Kd_inner_att, dt_control, integral_max_att_inner, integral_min_att_inner, alpha_d_att_inner, alpha_output_att_inner); % 固定参数

    % 初始化执行机构状态
    gimbal_angle_buffer = zeros(1, delay_steps);
    current_gimbal_angle_delayed = 0;
    current_gimbal_angle_actual = 0;

    % 预分配存储数组
    time_list_control = zeros(1, control_steps);
    x_position_list_control = zeros(1, control_steps);
    target_x_position_list_control = zeros(1, control_steps);
    target_pitch_angle_list_control = zeros(1, control_steps); % 存储速度环输出的姿态指令

    % 系统初始状态
    state = [0; 0; 0; 0; 0; 0]; % [角,角速度, x,vx, y,vy] - 初始位置x=0, y=0
    current_mass = initial_mass;
    current_inertia = inertia_full;
    current_gimbal_to_cg = gimbal_to_cg_full;
    target_x_position = 0; % 初始目标X位置

    %% 主仿真循环
    t = 0; step = 1;

    while t < time_duration
        % 控制更新（250Hz）
        if step <= control_steps && abs(t - (step - 1) * dt_control) < 1e-10
            t_control = (step - 1) * dt_control;

            % --- 控制逻辑 ---
            % 1. 设定目标水平位置 (例如：0.5秒后开始移动到X=1m)
            if t_control < 0.5
                target_x_position = 0; % 保持初始位置
            else
                target_x_position = 1.0; % 目标位置 X=1m
            end

            % 2. 水平位置环 (X-Position PID)
            position_error_x = target_x_position - state(3); % 水平位置误差 (目标 - 实际X)
            [pid_pos_x, target_x_velocity] = updatePIDController(pid_pos_x, position_error_x);
            target_x_velocity = max(min(target_x_velocity, max_target_velocity_x), min_target_velocity_x); % 限制目标水平速度

            % 3. 水平速度环 (X-Velocity PID)
            velocity_error_x = target_x_velocity - state(4); % 水平速度误差 (目标 - 实际Vx)
            [pid_vel_x, target_pitch_angle_cmd] = updatePIDController(pid_vel_x, velocity_error_x);
            % 速度误差 -> 俯仰角指令。正Vx误差(需要加速向右) -> 负Pitch指令(向右倾斜)
            target_pitch_angle = -target_pitch_angle_cmd; % 取反
            target_pitch_angle = max(min(target_pitch_angle, max_target_pitch), min_target_pitch); % 限制目标俯仰角

            % 4. 姿态外环 (Attitude Angle PID - 固定参数)
            theta_error = target_pitch_angle - state(1); % 俯仰角误差
            [pid_att_outer, omega_ref] = updatePIDController(pid_att_outer, theta_error);
            omega_ref = max(min(omega_ref, max_omega_att), -max_omega_att); % 限制角速度指令

            % 5. 姿态内环 (Attitude Rate PID - 固定参数)
            omega_error = omega_ref - state(2); % 角速度误差
            [pid_att_inner, control] = updatePIDController(pid_att_inner, omega_error);
            current_gimbal_angle = max(min(control, max_gimbal_angle), -max_gimbal_angle); % 限制摆角指令

            % 6. 执行机构模型 (延迟 + 滞后 + 噪声)
            gimbal_angle_buffer = [current_gimbal_angle, gimbal_angle_buffer(1:end - 1)];
            current_gimbal_angle_delayed = gimbal_angle_buffer(end);
            current_gimbal_angle_actual = (1 - alpha_actuator) * current_gimbal_angle_actual + alpha_actuator * current_gimbal_angle_delayed;
            gimbal_angle_noise = gimbal_angle_noise_std * randn;
            current_gimbal_angle_actual = current_gimbal_angle_actual + gimbal_angle_error + gimbal_angle_noise;

            % 存储数据
            time_list_control(step) = t_control;
            x_position_list_control(step) = state(3); % 记录实际x位置
            target_x_position_list_control(step) = target_x_position; % 记录目标x位置
            target_pitch_angle_list_control(step) = target_pitch_angle; % 记录速度环输出的目标姿态角

            step = step + 1;
        end

        % 动力学更新（1kHz）
        if fuel_mass > 0
            current_mass_flow_rate = max_mass_flow_rate * (manual_thrust / max_thrust);
            fuel_mass = max(fuel_mass - current_mass_flow_rate * dt_dynamics, 0);
            current_mass = empty_mass + fuel_mass;
        end

        if initial_mass > empty_mass % 避免除零
            mass_ratio = (initial_mass - current_mass) / (initial_mass - empty_mass);
        else
            mass_ratio = 0;
        end

        current_gimbal_to_cg = gimbal_to_cg_full - mass_ratio * (gimbal_to_cg_full - gimbal_to_cg_empty);
        current_inertia = inertia_full - mass_ratio * (inertia_full - inertia_empty);
        state = updateDynamicsRK4(state, manual_thrust, current_gimbal_angle_actual, current_mass, current_inertia, current_gimbal_to_cg, dt_dynamics);

        % 简单的地面碰撞检测 (可选, Y方向)
        if state(5) < 0 && state(6) < 0 % 如果低于地面且向下运动
            state(5) = 0; % 停在地面
            state(6) = 0; % 垂直速度为0
        end

        t = t + dt_dynamics;
    end

    %% 计算适应度值
    % 1. 水平位置跟踪均方根误差 (RMSE)
    %    只计算目标设定后的误差 (例如，忽略前0.5秒)
    valid_indices = time_list_control >= 0.5;

    if ~any(valid_indices)
        position_rmse_x = 1e6; % 惩罚值
    else
        position_rmse_x = sqrt(mean((target_x_position_list_control(valid_indices) - x_position_list_control(valid_indices)) .^ 2));
    end

    % 2. 水平位置超调量 (Overshoot)
    position_overshoot_x = 0;
    start_idx = find(valid_indices, 1, 'first');

    if ~isempty(start_idx)

        for i = start_idx:control_steps
            target = target_x_position_list_control(i);
            actual = x_position_list_control(i);
            % 只计算向目标方向移动时的超调
            if target > 0 % 目标在正方向 (左)
                os = max(actual - target, 0);
            elseif target < 0 % 目标在负方向 (右)
                os = max(target - actual, 0); % 超调是 actual 比 target 更负
            else % 目标是0
                os = abs(actual); % 超调是偏离0的程度
            end

            position_overshoot_x = max(position_overshoot_x, os);
        end

    else
        position_overshoot_x = 1e6; % 惩罚值
    end

    % 3. 姿态指令幅度惩罚 (RMS of target pitch angle)
    if any(valid_indices)
        attitude_cmd_rms = sqrt(mean(target_pitch_angle_list_control(valid_indices) .^ 2));
    else
        attitude_cmd_rms = 1e6; % 惩罚值
    end

    % 4. 适应度函数权重 (可调整)
    w1 = 1.0; % 位置跟踪误差权重
    w2 = 0.2; % 超调惩罚权重
    w3 = 0.01; % 姿态指令幅度惩罚权重 (稍微降低，因为水平移动必然需要倾斜)

    % 5. 计算最终适应度值
    fitness = w1 * position_rmse_x + w2 * position_overshoot_x + w3 * attitude_cmd_rms;

    % 可选：增加对最终位置误差的惩罚
    final_error_penalty = 0;

    if ~isempty(x_position_list_control)
        final_error = abs(target_x_position_list_control(end) - x_position_list_control(end));
        final_error_penalty = 0.5 * final_error; % 增加最终稳态误差的权重
    end

    fitness = fitness + final_error_penalty;

    % 可选：增加对垂直位置变化的惩罚 (如果希望尽量保持高度)
    % y_deviation_penalty = 0.1 * sqrt(mean(state_history_y(valid_indices).^2)); % 惩罚偏离初始高度
    % fitness = fitness + y_deviation_penalty;

end
