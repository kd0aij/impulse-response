function [roll, pitch, yaw] = quat2euler(q)
# convert column vectors of quaternions to 3 vectors of xyz-fixed Euler angles
nrows = size(q)(1);
roll = zeros(nrows,1);
pitch = zeros(nrows,1);
yaw = zeros(nrows,1);

for i = [1:nrows]
  roll(i) = atan2(2 * (q(i,1) * q(i,2) + q(i,3) * q(i,4)), 1 - 2 * (q(i,2) * q(i,2) + q(i,3) * q(i,3)));
  pitch(i) =	asin(2 * (q(i,1) * q(i,3) - q(i,4) * q(i,2)));
  yaw(i) = atan2(2 * (q(i,1) * q(i,4) + q(i,2) * q(i,3)), 1 - 2 * (q(i,3) * q(i,3) + q(i,4) * q(i,4)));
endfor
