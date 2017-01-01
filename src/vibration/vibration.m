pkg load optim
pkg load signal
pkg load mapping
addpath("src/imp_resp/loaders:src/imp_resp/lib");
close all;
clear all;

default_basePath = "/home/markw/gdrive/flightlogs";

rad2deg = 180/pi;

# set default run parameters
# desired uniform sample rate in Hz; should be less than average sample rate
sampRate = 400

# set to true when start/endTime are correct
rangeSelected = 0;
# length in seconds of impulse response plot
impLen = 1.5

# path to CSV files
fflush (stdout);
choice = input("load example dataset? (Y/n):", "s");
if length(choice) == 0 || choice == 'Y' || choice == 'y'
  choice = input("choose example [1,2]: ");
  if choice == 1
    basePath = "sample_ulogs/S250AQ/2016-12-05/log002/"
    startTime = 148
    endTime = 160
  elseif choice == 2
    basePath = "sample_ulogs/AquaQuad/2016-09-17/"
    startTime = 167
    endTime = 178
  else
    basePath = "ulogs/S250_pixracer/2016-08-18/sess001/log3/"
  endif
else
  basePath = default_basePath;
endif

if strcmp(basePath, default_basePath)
  basePath = [uigetdir(basePath) "/"];
endif

# CSV files generated by u2log2csv.py are prefixed by "logname_"
files = readdir(basePath);
for i = [1:size(files)(1)]
  r = cell2mat(strfind(files(i), "sensor_combined"));
  if (r > 0) 
    prefix = cell2mat(files(i))(1:r-1)
    break 
  endif
endfor

# load the measured rates, rate setpoints and rate controls (actuator input)
[a0t, accel0] = loadAccel_xyz(prefix, 0, basePath);
[af0t, accelf0] = loadAccel_filt(prefix, 0, basePath);

nsamples = size(accel0)(1);
sigRange = [1:nsamples];

# find timespan of accel data
if (!exist('startTime') | !exist('endTime'))
  startTime = a0t(1)
  endTime = a0t(end)
else
  # find index span of accel data
  startOffset = 1;
  while (a0t(startOffset) < startTime) startOffset++; endwhile
  endOffset = startOffset;
  while (a0t(endOffset) < endTime) endOffset++; endwhile
  sigRange = [startOffset:endOffset];
endif

figNum = 1;
figure(figNum++, "Position", [1,200,1200,480]);
subplot(2,1,1);
# ticks, rollRate, pitchRate
plot(a0t, accel0(:,1), "-b", a0t, accel0(:,2), "-r");
axis("tight"); title("raw roll and pitch rate data extents");
xlabel("seconds");
grid("on"); grid("minor");

while (true)
  subplot(2,1,2);
  plot(a0t(sigRange), accel0(sigRange,1), "-b", a0t(sigRange), accel0(sigRange,2), "-r");
  axis("tight"); title("raw roll and pitch rate data subset");
  xlabel("seconds");
  grid("on"); grid("minor");

  newStart = input("enter new startTime (return when done): ", "s");
  if length(newStart) == 0
    break; 
  else
    startTime = str2num(newStart);
  endif;
  endTime = input("new endTime: ");

  # find index span of accel data
  startOffset = 1;
  while (a0t(startOffset) < startTime) startOffset++; endwhile
  endOffset = startOffset;
  while (a0t(endOffset) < endTime) endOffset++; endwhile
  sigRange = [startOffset:endOffset];
endwhile

# resample to uniform rate, over the time range [startTime, endTime]
[a0tu, accel0u] = resample2(startTime, endTime, a0t, accel0, sampRate);
[af0tu, accelf0u] = resample2(startTime, endTime, af0t, accelf0, sampRate);

# get FFT and calculate frequency span
dataLen = size(accel0u)(1)
fftLen = 2*ceil((dataLen-2)/2) + 1
accel_fft = fft(accel0u);
accelf_fft = fft(accelf0u);
nyquist_freq = sampRate/2
index_range = [2:ceil(fftLen/2)];
freq_range = index_range * sampRate / fftLen;

figure(figNum++, "Position", [1,300,1200,480]);
plot(freq_range, abs(accelf_fft(index_range,1)), 'r',
      freq_range, abs(accelf_fft(index_range,2)), 'g',
      freq_range, abs(accelf_fft(index_range,3)), 'b');
title("vibration spectrum: filtered");
legend("x", "y", "z", "Location", "northwest");
xlabel("Hz");
axis("tight"); grid on;
hgsave([basePath "/vibration_spectrum.ofig"])

figure(figNum++, "Position", [1,400,1200,480]);
plot(freq_range, abs(accel_fft(index_range,1)), 'r',
      freq_range, abs(accel_fft(index_range,2)), 'g',
      freq_range, abs(accel_fft(index_range,3)), 'b');
title("vibration spectrum");
legend("x", "y", "z", "Location", "northwest");
xlabel("Hz");
axis("tight"); grid on;
hgsave([basePath "/vibration_spectrum.ofig"])

figure(figNum++, "Position", [1,500,1200,480]);
subplot(3,1,1);
plot(freq_range, abs(accel_fft(index_range,1)), 'r');
axis("tight"); grid on;
title("vibration spectrum");
subplot(3,1,2);
plot(freq_range, abs(accel_fft(index_range,2)), 'g');
axis("tight"); grid on;
subplot(3,1,3);
plot(freq_range, abs(accel_fft(index_range,3)), 'b');
axis("tight"); grid on;
xlabel("Hz");
hgsave([basePath "/vibration_spectrum2.ofig"])
