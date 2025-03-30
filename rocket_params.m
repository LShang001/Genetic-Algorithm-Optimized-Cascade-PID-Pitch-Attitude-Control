% rocket_params.m - 火箭系统统一参数配置

%% 仿真参数
params.dt_control = 0.004; % 控制周期[s]
params.dt_dynamics = 0.001; % 动力学步长[s]
params.time_duration_full = 12; % 完整仿真时长[s]
params.time_duration_fitness = 4; % 适应度仿真时长[s]

%% 火箭物理参数
params.initial_mass = 15.8; % 初始质量[kg]
params.fuel_mass = 3.0; % 燃料质量[kg]
params.empty_mass = 12.8; % 空载质量[kg]
params.max_thrust = 200; % 最大推力[N]
params.max_mass_flow_rate = 0.12; % 最大质量流率[kg/s]
params.gimbal_to_cg_full = 0.58; % 满载质心距离[m]
params.gimbal_to_cg_empty = 0.50; % 空载质心距离[m]
params.inertia_full = 2.76; % 满载转动惯量[kg·m²]
params.inertia_empty = 2.1; % 空载转动惯量[kg·m²]
params.max_gimbal_angle = 15; % 最大摆角[°]
params.max_omega = 80; % 最大角速度[°/s]

%% 控制参数
params.integral_max_outer = 100; % 外环积分限幅
params.integral_min_outer = -100;
params.integral_max_inner = 100; % 内环积分限幅
params.integral_min_inner = -100;
params.alpha = 0.3; % 目标滤波系数
params.alpha_d_outer = 1; % 外环微分滤波
params.alpha_output_outer = 1; % 外环输出滤波
params.alpha_d_inner = 1; % 内环微分滤波
params.alpha_output_inner = 1; % 内环输出滤波

%% 执行机构参数
params.delay_time = 0.01; % 执行延迟[s]
params.tau_actuator = 0.05; % 执行时间常数[s]

%% 噪声参数
params.gimbal_angle_error = 0.0; % 摆角固定误差[°]
params.gimbal_angle_noise_std = 0.0; % 摆角噪声[°]
params.sensor_noise_std = 0.00; % 角度噪声[°]
params.sensor_rate_noise_std = 0.0; % 角速度噪声[°/s]

%% 默认PID参数（优化时会被覆盖）
params.default_pid = [3.5, 0.0, 0.0, 1.8, 1.2, 0.08];
