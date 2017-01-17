function [gyro, gt, s_interval] = loadFullGyro(basePath, prefix, sigRange=[])

filename = [basePath prefix "sensor_gyro_0.csv"];
gyrodata = tdfread(filename);

if (length(sigRange) == 0)
  sigRange = [1 : length(gyrodata.int_count)-1];
%elseif (sigRange(end) > length(gyrodata.int_count)-1)
%  sigRange = [sigRange(1) : length(gyrodata.int_count)-1];
endif

[average, variance, deltas, drop_lengths, drops] = interval_analysis(gyrodata.timestamp);
ndrops = length(drop_lengths)
droptimes = gyrodata.timestamp(drops) / 1e6
droplengths = drop_lengths / 1e6

if find(drops(sigRange))
  disp("error: dropout within sigRange");
endif

nrecs = length(gyrodata.int_count(sigRange))
nsamp = sum(gyrodata.int_count(sigRange))
duration = gyrodata.timestamp(sigRange(end)) - gyrodata.timestamp(sigRange(1));
s_interval = 1e-6 * duration / (nsamp - gyrodata.int_count(1))
gyro = zeros(nsamp,3);
gt = 1e-6 * gyrodata.timestamp(sigRange(1)) + s_interval * ([1:nsamp]' - gyrodata.int_count(1));

k = 1;
for i = sigRange
  if (gyrodata.int_count(i) >= 1)
    gyro(k,1) = gyrodata.x_in_0_(i);
    gyro(k,2) = gyrodata.y_in_0_(i);
    gyro(k++,3) = gyrodata.z_in_0_(i);
  endif
  if (gyrodata.int_count(i) >= 2)
    gyro(k,1) = gyrodata.x_in_1_(i);
    gyro(k,2) = gyrodata.y_in_1_(i);
    gyro(k++,3) = gyrodata.z_in_1_(i);
  endif
  if (gyrodata.int_count(i) >= 3)
    gyro(k,1) = gyrodata.x_in_2_(i);
    gyro(k,2) = gyrodata.y_in_2_(i);
    gyro(k++,3) = gyrodata.z_in_2_(i);
  endif
  if (gyrodata.int_count(i) >= 4)
    gyro(k,1) = gyrodata.x_in_3_(i);
    gyro(k,2) = gyrodata.y_in_3_(i);
    gyro(k++,3) = gyrodata.z_in_3_(i);
  endif
  if (gyrodata.int_count(i) >= 5)
    gyro(k,1) = gyrodata.x_in_4_(i);
    gyro(k,2) = gyrodata.y_in_4_(i);
    gyro(k++,3) = gyrodata.z_in_4_(i);
  endif
endfor
