function [roll_out, pitch_out, yaw_out] = attitude_pid(desired_angles, current_angles, current_rates, params)
% ATTITUDE_PID Cascaded Angle -> Rate PID Controller
%
% Inputs:
%   desired_angles : [desired_roll, desired_pitch, desired_yaw] (deg)
%   current_angles : [kalman_roll, kalman_pitch, kalman_yaw] (deg)
%   current_rates  : [gyro_x, gyro_y, gyro_z] (deg/s)
%   params         : Global parameter struct containing PID gains

% 1. Outer Loop: Angle Controller (Error -> Desired Rate)
error_roll_angle  = desired_angles(1) - current_angles(1);
error_pitch_angle = desired_angles(2) - current_angles(2);

desired_roll_rate  = params.PID.kp_roll_angle * error_roll_angle;
desired_pitch_rate = params.PID.kp_pitch_angle * error_pitch_angle;
desired_yaw_rate   = 0.0; % Heading lock

% 2. Inner Loop: Angular Rate Controller (Error -> Motor PWM Adjustment)
error_roll_rate  = desired_roll_rate - current_rates(1);
error_pitch_rate = desired_pitch_rate - current_rates(2);
error_yaw_rate   = desired_yaw_rate - current_rates(3);

roll_out  = params.PID.kp_roll_rate * error_roll_rate;
pitch_out = params.PID.kp_pitch_rate * error_pitch_rate;
yaw_out   = params.PID.kp_yaw_rate * error_yaw_rate;
end
