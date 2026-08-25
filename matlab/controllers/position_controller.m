function [desired_roll, desired_pitch] = position_controller(desired_xy, current_xy, current_vxy, params)
% POSITION_CONTROLLER Outer-Loop XY Position Controller
% Maps 2D position error (m) -> desired attitude tilt angles (deg)

error_x = desired_xy(1) - current_xy(1);
error_y = desired_xy(2) - current_xy(2);

if isfield(params, 'PID') && isfield(params.PID, 'kp_xy')
    kp_pos = params.PID.kp_xy;
    kd_pos = params.PID.kd_xy;
else
    kp_pos = 8.0;
    kd_pos = 5.0;
end

desired_pitch = kp_pos * error_x - kd_pos * current_vxy(1);  % X error drives pitch
desired_roll  = -kp_pos * error_y + kd_pos * current_vxy(2); % Y error drives roll

max_tilt = 12.0;
desired_pitch = min(max(desired_pitch, -max_tilt), max_tilt);
desired_roll  = min(max(desired_roll, -max_tilt), max_tilt);
end
