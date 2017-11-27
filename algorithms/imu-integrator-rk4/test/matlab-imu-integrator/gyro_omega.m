function omega = gyro_omega(gyro_readings)

omega = zeros(4,4);

omega(2:4, 2:4) = - skew3(gyro_readings);
omega(1, 2:4) = - gyro_readings';
omega(2:4, 1) = gyro_readings;

end