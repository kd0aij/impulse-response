pkg load optim
pkg load signal
pkg load mapping
addpath("./loaders:./lib");
close all;
clear all;

rad2deg = 180/pi;

# set default run parameters
# desired uniform sample rate in Hz; should be less than average sample rate
sampRate = 1e3

# synthesize 1KHz sample data
freq = 180
a0t = [0:1/sampRate:1]';
accel0 = sin(freq * 2 * pi * a0t);
nsamp = length(a0t)

# add false sample jitter
%jitter = zeros(size(a0t));
%jitter = awgn(jitter, 120);
%jitter_std = sqrt(var(jitter))
%a0t += jitter;

# integrate and decimate
n_int = 2;
decRate = sampRate / n_int
ndec = floor(nsamp/n_int)
a0td = zeros(ndec, 1);
accel0d = zeros(ndec, 1);
n = 1;
for i = [1:n_int:nsamp-1]
  a0td(n) = a0t(i);
  accel0d(n) = (accel0(i) + accel0(i+1)) / 2;
  n++;
end
jitter = zeros(size(a0td));
jitter([1:2:end]) = .0004;
jitter([2:2:end]) = -.0004;
jitter_std = sqrt(var(jitter))
a0td += jitter;

# resample to 500Hz
sampRate /= 2
[a0tu, accel0u] = resample2(a0td(1), a0td(end), a0td, accel0d, sampRate);
a0tu = a0tu';
accel0u = accel0u';

xvar = var(accel0u)

# get FFT and calculate frequency span
fftLen = size(accel0u)(1)
%fftLen = 2*ceil((dataLen-2)/2)
accel_fft = fft(accel0u);
nyquist_freq = sampRate/2
freq_res = sampRate / fftLen
# ignore DC term
index_range = [2:ceil(fftLen/2)];
freq_range = -1 + index_range * sampRate / fftLen;

accel_mag = abs(accel_fft(index_range));
accel_mag /= max(accel_mag);
accel_db = 20 * log10(accel_mag);

figNum = 1;
figure(figNum++, "Position", [1,300,1200,480]);
plot(freq_range, accel_db);
title("vibration spectrum");
ylabel('dB');
xlabel('Hz');
xlabel("Hz");
axis("tight"); 
set(gca, "xgrid", "on");
set(gca, "xminorgrid", "on");
%hgsave([basePath "/vibration_spectrum.ofig"])
