function state_der = get_state_der(imu_readings, current_state, gravity)

state_der = zeros(16, 1);

acc_meas = imu_readings(1:3);
gyro_meas = imu_readings(4:6);

gyro_omega_matrix = gyro_omega(gyro_meas);

B_q_G = current_state(1:4);
B_q_G = B_q_G / norm(B_q_G);
B_R_G = q_to_rot_mat(B_q_G);
G_R_B = B_R_G';

q_dot = 0.5 * gyro_omega_matrix * B_q_G;
v_dot = G_R_B * acc_meas - [0 0 gravity]';
p_dot = current_state(8:10);

state_der(1:4) = q_dot;
state_der(8:10) = v_dot;
state_der(14:16) = p_dot;

end