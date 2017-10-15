pkg load optim
pkg load signal
pkg load mapping
addpath("./loaders:./lib");
close all;
clear all;

%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_2K_1inst/flight1/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_2K_1inst/bench2/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/200hz/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/60hz/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/chirp/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/chirp_50_1000_30/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/chirp_50_250_30/log3/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/200hz/weight_pad/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/180hz/weight_pad/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/180hz/orig_weight_pad/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/180hz/orig_new/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/125hz/weight_pad/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/130hz/weight_pad/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/135hz/weight_pad/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/1100hz/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/60hz/weight_pad/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/5hz/weight_pad/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/30hz/weight_pad/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/30hz/orig_new/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/click/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/chirp_orig_20_180_new/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/chirp_20_180_new/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/chirp_150_200/orig/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/chirp_150_200/uniform/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/noise/uniform/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1KHz_synth/audacity/noise/orig/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1Khz/all_samp/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1Khz/all_samp/180hz-2/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1Khz/all_samp/chirp/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/500Hz/180hz/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/500Hz/450hz/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/250Hz/180hz/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/250Hz/100hz/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/250Hz/noise_floor/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/250Hz/noise_floor/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/S250/flight1/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/S250/flight3/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/AeroQuad/2017-01-21-3/";
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/AeroQuad/bench1/";
default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/fast_sensor_log_rebased/180Hz/";

%startTime = 40
%endTime = 60
%startTime = 40
%endTime = 60
%default_basePath = "/home/markw/gdrive/flightlogs/fast_gyro/1Khz/flt1/";
%startTime = 40
%endTime = 60

rad2deg = 180/pi;

# set default run parameters
# desired uniform sample rate in Hz; should be less than average sample rate
sampRate = 1000

# set to true when start/endTime are correct
rangeSelected = 0;
# length in seconds of impulse response plot
impLen = 1.5

# path to CSV files
fflush (stdout);
basePath = default_basePath;

# CSV files generated by u2log2csv.py are prefixed by "logname_"
# process the first ".ulg" file we find in the specified directory
files = readdir(basePath);
for i = [1:size(files)(1)]
  r = cell2mat(strfind(files(i), ".ulg"));
  if (r > 0) 
    prefix = [cell2mat(files(i))(1:r-1) '_']
    [info,err,msg] = stat([basePath files{i}]);
    logsize = info.size
    break 
  endif
endfor

# load the signal data
choice = input("select accel or gyro data (0/1):");
if (choice == 0 || choice == [])
  sigType = "accel"
  [s0t, signal, data] = loadAccel_filt(prefix, 0, basePath);
  #[s0t, signal, data] = loadAccel_int(prefix, 0, basePath);
else
  sigType = "gyro"
  [s0t, signal, data] = loadGyro_filt(prefix, 0, basePath);
  #[s0t, signal, data] = loadGyro_int(prefix, 0, basePath);
endif

nsamples = size(signal)(1);
sigRange = [1:nsamples];
logduration = (s0t(end) - s0t(1))
printf("log data rate: %6.1f Kbytes/sec\n", 1e-3 * logsize / logduration);

[average, variance, deltas, drop_lengths, drops] = interval_analysis(data.timestamp);
ndrops = length(drop_lengths)
droptimes = s0t(drops);
droplengths = drop_lengths / 1e6;

# find timespan of data
if (!exist('startTime') | !exist('endTime'))
  startTime = s0t(1)
  endTime = s0t(end)
else
  # find index span of gyro data
  startOffset = 1;
  while (s0t(startOffset) < startTime) startOffset++; endwhile
  endOffset = startOffset;
  while (s0t(endOffset) < endTime) endOffset++; endwhile
  sigRange = [startOffset:endOffset];
endif

figNum = 1;
figure(figNum++, "Position", [1,200,1200,480]);
subplot(2,1,1);
plot(s0t, signal(:,1), "-r", s0t, signal(:,2), "-g", s0t, signal(:,3), "-b");
limits = axis;
hold on
for i = [1:ndrops]
  plot([droptimes(i) droptimes(i)], [limits(3) limits(4)], 'mx-');
endfor 
hold off
axis("tight"); title(["filtered " sigType " data extents"]);
xlabel("seconds");
grid("on"); grid("minor");

while (true)
  subplot(2,1,2);
  plot(s0t(sigRange), signal(sigRange,1), "-r", s0t(sigRange), signal(sigRange,2), "-g", s0t(sigRange), signal(sigRange,3), "-b");
  drawnow;
  # this works in debug mode, but axis limits are wrong otherwise
  axis("tight")
  limits2 = axis

  hold on
  for i = [1:ndrops]
    if (droptimes(i) > limits2(1) && droptimes(i) < limits2(2))
      plot([droptimes(i) droptimes(i)], [limits2(3) limits2(4)], 'mx-');
    endif
  endfor 
  hold off
  axis("tight"); title(["filtered " sigType " data subset"]);
  xlabel("seconds");
  grid("on"); grid("minor");

  newStart = input("enter new startTime (return when done): ", "s");
  if length(newStart) == 0
    break; 
  else
    startTime = str2num(newStart);
  endif;
  endTime = input("new endTime: ");

  # find index span of gyro data
  startOffset = 1;
  while (s0t(startOffset) < startTime) startOffset++; endwhile
  startOffset
  endOffset = startOffset;
  while (s0t(endOffset) < endTime) endOffset++; endwhile
  endOffset
  sigRange = [startOffset:endOffset];
endwhile

# load 1KHz sample sequence for selected time range
if (choice == 0)
  [samp_seq, at, a_interval] = loadFullAccel(basePath, prefix, sigRange);
else
  [samp_seq, at, a_interval] = loadFullGyro(basePath, prefix, sigRange);
endif

nsamp = length(samp_seq);

# get FFT and calculate frequency span
fftLen = size(samp_seq)(1)
accel_fft = fft(samp_seq - mean(samp_seq));
nyquist_freq = sampRate/2
freq_res = sampRate / fftLen
# ignore DC term
index_range = [2:ceil(fftLen/2)];
freq_range = -1 + index_range * sampRate / fftLen;

figure(figNum++, "Position", [1,300,1200,300]);
plot(at, samp_seq, '.-');
legend(["x";"y";"z"]);
axis("tight"); grid on;
title([sigType " signals"]);
xlabel("seconds");
ylabel("m/sec/sec");

zindex_range = [1:ceil(nsamp/2)]';
zfreq_range = zindex_range * 1000 / nsamp;
zseq_fft = fft(samp_seq - mean(samp_seq));
zseq_fft = abs(zseq_fft(zindex_range,:));

x_fft = abs(accel_fft(index_range,1));
y_fft = abs(accel_fft(index_range,2));
z_fft = abs(accel_fft(index_range,3));
fscl = max(z_fft);

figure(figNum++, "Position", [1,300,1200,300]);
plot(
     freq_range, 20 * log10(x_fft / fscl), 'r',
     freq_range, 20 * log10(y_fft / fscl), 'g',
     freq_range, 20 * log10(z_fft / fscl), 'b'
     );
title([sigType " vibration spectrum"]);
legend("x", "y", "z", "Location", "northeast");
xlabel("Hz");
axis("tight"); grid on;
hgsave([basePath "/vibration_spectrum.ofig"])

absmax = max(max(zseq_fft));
figure(figNum++, "Position", [1,300,1200,900]);
subplot(3,1,1);
plot(zfreq_range, 20 * log10(zseq_fft(:,1) / absmax), 'r');
axis([0 500 -60 0]);
grid on;
xlabel("Hz"); ylabel("dB");
title([sigType " vibration spectrum x"]);
subplot(3,1,2);
plot(zfreq_range, 20 * log10(zseq_fft(:,2) / absmax), 'g');
axis([0 500 -60 0]);
grid on;
xlabel("Hz"); ylabel("dB");
title([sigType " vibration spectrum y"]);
subplot(3,1,3);
plot(zfreq_range, 20 * log10(zseq_fft(:,3) / absmax), 'b');
axis([0 500 -60 0]);
grid on;
xlabel("Hz"); ylabel("dB");
title([sigType " vibration spectrum z"]);
hgsave([basePath sigType "_vibration_spectrum.ofig"])

save ("-binary", [basePath "workspace.bin"])