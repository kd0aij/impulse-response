pkg load optim
pkg load signal
pkg load mapping
addpath("./loaders:./lib");
close all;
clear all;

default_basePath = "/home/markw/Desktop/flightLogs_Desktop/ArduCopter_logs/s250_ardu/2017-03-21/";

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

# CSV files generated by mavlogdump.py
# load the signal data
choice = input("select accel or gyro data (0/1):");
if (choice == 0 || choice == [])
  sigType = "accel"
  [s0t, signal, data] = loadAcc1(basePath);
else
  sigType = "gyro"
  [s0t, signal, data] = loadGyr1(basePath);
endif

nsamples = size(signal)(1);
sigRange = [1:nsamples];
logduration = (s0t(end) - s0t(1))

[average, variance, deltas, drop_lengths, drops] = interval_analysis(data.timestamp);
ndrops = length(drop_lengths)
droptimes = s0t(drops);

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
axis("tight"); title(["raw " sigType " data extents"]);
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
  axis("tight"); title(["raw " sigType " data subset"]);
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

nsamp = length(signal);

# get FFT and calculate frequency span
fftLen = size(signal)(1)
accel_fft = fft(signal - mean(signal));
nyquist_freq = sampRate/2
freq_res = sampRate / fftLen
# ignore DC term
index_range = [2:ceil(fftLen/2)];
freq_range = -1 + index_range * sampRate / fftLen;

figure(figNum++, "Position", [1,300,1200,300]);
plot(s0t, signal, '.-');
legend(["x";"y";"z"]);
axis("tight"); grid on;
title([sigType " signals"]);
xlabel("seconds");
ylabel("m/sec/sec");

zindex_range = [1:ceil(nsamp/2)]';
zfreq_range = zindex_range * 1000 / nsamp;
zseq_fft = fft(signal - mean(signal));
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
hgsave([basePath "/" sigType "_vibration_spectrum.ofig"])

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
hgsave([basePath "/" sigType "_vibration_spectrum2.ofig"])

save ("-binary", [basePath "workspace.bin"])
