function [roll, pitch, yaw] = q2e(q0,q1,q2,q3)

% Convert quaternions to Euler angles 
% Euler angles rotation sequence: z-y'-x'' (yaw, pitch, roll)
% The Euler angles rotation is intrinsic, i.e., rotate along new axes 
% J. Chen, July, 2019

% Ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
% Verification https://www.andre-gaschler.com/rotationconverter/ 
% Note that quaternion to Euler is not unique but Euler to quaternionis unique 
% Also see https://www.mathworks.com/help/aerotbx/ug/quat2angle.html

% q = q0 + q1*i + q2*j + q3*k

% roll (x-axis rotation)
sinr_cosp = 2.0 * (q0 .* q1 + q2 .* q3);
cosr_cosp = 1.0 - 2.0 * (q1 .* q1 + q2 .* q2);
roll = atan2(sinr_cosp, cosr_cosp);

% pitch (y-axis rotation)
sinp = 2.0 * (q0 .* q2 - q3 .* q1);
if (abs(sinp) >= 1)
  pitch = pi/2*sign(sinp); % use 90 degrees if out of range
else
  pitch = asin(sinp);
end

% yaw (z-axis rotation)
siny_cosp = 2.0 * (q0 .* q3 + q1 .* q2);
cosy_cosp = 1.0 - 2.0 * (q2 .* q2 + q3 .* q3);  
yaw = atan2(siny_cosp, cosy_cosp);


% EulerAngles ToEulerAngles(Quaternion q)
% {
%     EulerAngles angles;
% 
%     // roll (x-axis rotation)
%     double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
%     double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
%     angles.roll = atan2(sinr_cosp, cosr_cosp);
% 
%     // pitch (y-axis rotation)
%     double sinp = +2.0 * (q.w * q.y - q.z * q.x);
%     if (fabs(sinp) >= 1)
%         angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
%     else
%         angles.pitch = asin(sinp);
% 
%     // yaw (z-axis rotation)
%     double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
%     double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
%     angles.yaw = atan2(siny_cosp, cosy_cosp);
% 
%     return angles;
% }