function [phi_der, cov_der] = get_phi_cov_der(imu_readings, ... 
    current_state, current_phi, current_cov, sigmas_squared)

phi_cont = zeros(15);

acc_meas = imu_readings(1:3);
gyro_meas = imu_readings(4:6);

acc_skew = skew3(acc_meas);
gyro_skew = skew3(gyro_meas);

B_q_G = current_state(1:4);
B_q_G = B_q_G / norm(B_q_G);
G_R_B = q_to_rot_mat(B_q_G)';

phi_cont(1:3, 4:6) = - eye(3);
phi_cont(13:15, 7:9) =  eye(3);
phi_cont(1:3, 1:3) = - gyro_skew;
phi_cont(7:9, 10:12) = - G_R_B;
phi_cont(7:9, 1:3) = - G_R_B * acc_skew;

gyro_noise_sigma_squared = sigmas_squared(1);
gyro_bias_sigma_squared = sigmas_squared(2);
acc_noise_sigma_squared = sigmas_squared(3);
acc_bias_sigma_squared = sigmas_squared(4);

Q_noise = zeros(15);
Q_noise(1:3, 1:3) = gyro_noise_sigma_squared * eye(3);
Q_noise(4:6, 4:6) = gyro_bias_sigma_squared * eye(3);
Q_noise(7:9, 7:9) = acc_noise_sigma_squared * eye(3);
Q_noise(10:12, 10:12) = acc_bias_sigma_squared * eye(3);

cov_der = zeros(15);
phi_der = zeros(15);
cov_der = phi_cont * current_cov + current_cov * phi_cont' + Q_noise;
phi_der = phi_cont * current_phi;

end