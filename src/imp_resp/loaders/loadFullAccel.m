function [accel, at, s_interval] = loadFullAccel(basePath, prefix, sigRange=[])

filename = [basePath prefix "sensor_accel_0.csv"];
accdata = tdfread(filename);

if (length(sigRange) == 0)
  sigRange = [1 : length(gyrodata.int_count)-1];
%elseif (sigRange(end) > length(gyrodata.int_count)-1)
%  sigRange = [sigRange(1) : length(gyrodata.int_count)-1];
endif

nrecs = length(accdata.int_count(sigRange))
nsamp = sum(accdata.int_count(sigRange))
duration = accdata.timestamp(end) - accdata.timestamp(1);
s_interval = 1e-6 * duration / (nsamp - accdata.int_count(1))
accel = zeros(nsamp,3);
at = 1e-6 * accdata.timestamp(2) + s_interval * ([1:nsamp]' - accdata.int_count(1));

j = 1;
for i = sigRange
  if (accdata.int_count(i) >= 1)
    accel(j,1)   = accdata.x_in_0_(i);
    accel(j,2)   = accdata.y_in_0_(i);
    accel(j++,3) = accdata.z_in_0_(i);
  endif
  if (accdata.int_count(i) >= 2)
    accel(j,1)   = accdata.x_in_1_(i);
    accel(j,2)   = accdata.y_in_1_(i);
    accel(j++,3) = accdata.z_in_1_(i);
  endif
  if (accdata.int_count(i) >= 3)
    accel(j,1)   = accdata.x_in_2_(i);
    accel(j,2)   = accdata.y_in_2_(i);
    accel(j++,3) = accdata.z_in_2_(i);
  endif
  if (accdata.int_count(i) >= 4)
    accel(j,1)   = accdata.x_in_3_(i);
    accel(j,2)   = accdata.y_in_3_(i);
    accel(j++,3) = accdata.z_in_3_(i);
  endif
  if (accdata.int_count(i) >= 5)
    accel(j,1)   = accdata.x_in_4_(i);
    accel(j,2)   = accdata.y_in_4_(i);
    accel(j++,3) = accdata.z_in_4_(i);
  endif
endfor
