% IMU integrator Phi and covariance calculation to be used as ground-truth
% data for testing.

linear_acceleration = 0.3;
linear_velocity = 2.5;
angular_velocity = 0.2;

% define current state (16x1), 
current_q = [sqrt(2)/2 0 0 -sqrt(2)/2]';
current_w_bias = zeros(3, 1);
current_v = [0 linear_velocity 0]';
current_a_bias = zeros(3, 1);
current_p = zeros(3, 1);

current_state = [current_q; current_w_bias; current_v; ...
    current_a_bias; current_p];

% initial cov (15x15) and phi (15x15)
current_cov = zeros(15);
current_phi = eye(15);

% define gravity magnitude
gravity = 9.81;

% define imu readings (i, i+1), first 3 for acc, second 3 for gyro
imu_readings_begin = [linear_acceleration 0 gravity 0 angular_velocity 0]';
imu_readings_end = [linear_acceleration 0 gravity 0 angular_velocity 0]';
imu_readings = [imu_readings_begin, imu_readings_end];

% define dt
delta_time_seconds = 1.2;

% define sigmas
gyro_noise_sigma = 0.1;
gyro_bias_sigma = 0.2;
acc_noise_sigma = 0.3;
acc_bias_sigma = 0.4;
sigmas_squared = [gyro_noise_sigma^2 gyro_bias_sigma^2 ...
    acc_noise_sigma^2 acc_bias_sigma^2];

% interpolate imu readings
imu_readings_k1 = imu_readings(1:6, 1);
imu_readings_k23 = interpolate_imu(imu_readings(1:6, 1), ...
    imu_readings(1:6, 2), delta_time_seconds, 0.5 * delta_time_seconds);
imu_readings_k4 = imu_readings(1:6, 2);

% calculate state derivates
state_der1 = get_state_der(imu_readings_k1, current_state, gravity);
state_der2 = get_state_der(imu_readings_k23, current_state ...
    + 0.5 * delta_time_seconds * state_der1, gravity);
state_der3 = get_state_der(imu_readings_k23, current_state ...
    + 0.5 * delta_time_seconds * state_der2, gravity);
state_der4 = get_state_der(imu_readings_k4, current_state ...
    + delta_time_seconds * state_der3, gravity);

% calculate final state from RK4
new_state = current_state + delta_time_seconds * (state_der1 ...
      + 2 * state_der2 + 2 * state_der3 + state_der4) / 6;

% calculate covariance and state phi derivatives
[phi_der1, cov_der1] = get_phi_cov_der(imu_readings_k1, ...
    current_state, current_phi, current_cov, ...
    sigmas_squared);  
[phi_der2, cov_der2] = get_phi_cov_der(imu_readings_k23, ...
    current_state + 0.5 * delta_time_seconds * state_der1, ...
    current_phi + 0.5 * delta_time_seconds * phi_der1, ...
    current_cov + 0.5 * delta_time_seconds * cov_der1, ... 
      sigmas_squared);  
[phi_der3, cov_der3] = get_phi_cov_der(imu_readings_k23, ...
    current_state + 0.5 * delta_time_seconds * state_der2, ...
    current_phi + 0.5 * delta_time_seconds * phi_der2, ...
    current_cov + 0.5 * delta_time_seconds * cov_der2, ...  
    sigmas_squared);  
[phi_der4, cov_der4] = get_phi_cov_der(imu_readings_k4, ...
    current_state + delta_time_seconds * state_der3, ...
    current_phi + delta_time_seconds * phi_der3, ...
    current_cov + delta_time_seconds * cov_der3, ...
    sigmas_squared);  

% calculate final cov and phi from RK4
new_cov = current_cov + delta_time_seconds * (cov_der1 ...
      + 2 * cov_der2 + 2 * cov_der3 + cov_der4) / 6;
new_phi = current_phi + delta_time_seconds * (phi_der1 ...
      + 2 * phi_der2 + 2 * phi_der3 + phi_der4) / 6;

