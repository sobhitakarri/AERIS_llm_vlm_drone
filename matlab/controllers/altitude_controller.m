function throttle_pwm = altitude_controller(desired_z, current_z, current_vz, params)
% ALTITUDE_CONTROLLER Altitude (Z) PID Controller
%
% Inputs:
%   desired_z  : Target altitude in meters
%   current_z  : Estimated altitude in meters (from ToF / Barometer)
%   current_vz : Vertical velocity in m/s
%   params     : Parameter struct

error_z = desired_z - current_z;
p_term = params.PID.kp_z * error_z;
d_term = -params.PID.kd_z * current_vz;

% Base hover throttle + PID correction
hover_base_pwm = 140; % Out of 255
throttle_pwm = hover_base_pwm + p_term + d_term;

% Saturation Limits
throttle_pwm = min(max(throttle_pwm, params.min_pwm), params.max_pwm);
end
