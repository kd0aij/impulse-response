pkg load optim
pkg load signal
pkg load mapping

# this script must be run from its location in the local repo
addpath("./loaders:./lib");
close all;
clear all;

rad2deg = 180/pi;

# set default run parameters
# actual uniform sample rate in Hz
rateOnly = true;
sampRate = 1000
nplots = 2

# set to true when start/endTime are correct
rangeSelected = 0;
# length in seconds of impulse response plot
impLen = 1.5

# path to CSV files
fflush (stdout);
default_basePath = "/home/markw/Desktop/flightLogs_Desktop/ArduCopter_logs/s250_ardu/2017-03-21/";

# load the measured rates, rate setpoints and rate controls (actuator input)
%[accelf, a0t, a_interval] = loadFullAccel(basePath, prefix);
%[gyrof, g0t, g_interval] = loadFullGyro(basePath, prefix);
[g0t, gyrof, gyrodata] = loadGyro_filt(prefix, 0, basePath);
[s0t, setpoint0] = loadVratesSP(prefix, 0, basePath);

# rotate gyro readings into body frame
rtheta = pi/4;
rotMatrix = [cos(rtheta) -sin(rtheta); sin(rtheta) cos(rtheta)]
gyrof(:,1:2) = gyrof(:,1:2) * rotMatrix;

[average, variance, deltas, drop_lengths, drops] = interval_analysis(gyrodata.timestamp);
ndrops = length(drop_lengths)
droptimes = g0t(drops);
droplengths = drop_lengths / 1e6;

nsamples = size(gyrof)(1);
sigRange = [1:nsamples];

# find timespan of gyro data
if (!exist('startTime') | !exist('endTime'))
  startTime = g0t(1)
  endTime = g0t(end)
else
  # find index span of gyro data
  startOffset = 1;
  while (g0t(startOffset) < startTime) startOffset++; endwhile
  endOffset = startOffset;
  while (g0t(endOffset) < endTime) endOffset++; endwhile
  sigRange = [startOffset:endOffset];
endif

figNum = 1;
figure(figNum++, "Position", [1,200,1200,480]);
subplot(nplots,1,1);
# ticks, rollRate, pitchRate
plot(g0t, gyrof(:,1), "-b", g0t, gyrof(:,2), "-r");
limits = axis;
hold on
for i = [1:ndrops]
  plot([droptimes(i) droptimes(i)], [limits(3) limits(4)], 'mx-');
endfor 
hold off
axis("tight"); title("raw roll and pitch rate data extents");
xlabel("seconds");
grid("on"); grid("minor");

while (true)
  subplot(nplots,1,2);
  plot(g0t(sigRange), gyrof(sigRange,1), "-b", g0t(sigRange), gyrof(sigRange,2), "-r");
  axis("tight"); title("raw roll and pitch rate data subset");
  xlabel("seconds");
  grid("on"); grid("minor");
  limits2 = axis
  hold on
  for i = [1:ndrops]
    if (droptimes(i) > limits2(1) && droptimes(i) < limits2(2))
      plot([droptimes(i) droptimes(i)], [limits2(3) limits2(4)], 'mx-');
    endif
  endfor 
  hold off

  newStart = input("enter new startTime (return when done): ", "s");
  if length(newStart) == 0
    break; 
  else
    startTime = str2num(newStart);
  endif;
  endTime = input("new endTime: ");

  # find index span of gyro data
  startOffset = 1;
  while (g0t(startOffset) < startTime) startOffset++; endwhile
  endOffset = startOffset;
  while (g0t(endOffset) < endTime) endOffset++; endwhile
  sigRange = [startOffset:endOffset-1];
endwhile

# reload gyro data with specified sigRange (avoid dropouts)
[gyrof, g0t, g_interval] = loadFullGyro(basePath, prefix, sigRange);
sampRate = 1 / g_interval

# rotate gyro readings into body frame
rtheta = pi/4;
rotMatrix = [cos(rtheta) -sin(rtheta); sin(rtheta) cos(rtheta)]
gyrof(:,1:2) = gyrof(:,1:2) * rotMatrix;

# resample to uniform rate, over the time range [startTime, endTime]
[s0tu, setpoint0u] = resample2(startTime, endTime, s0t, setpoint0, sampRate, false);
nsamp = min(length(gyrof), length(setpoint0u));

rollRateSig = gyrof(1:nsamp,1);
rollSetpoint = setpoint0u(:,1);

pitchRateSig = gyrof(1:nsamp,2);
pitchSetpoint = setpoint0u(:,2);

# lowpass filter signal streams
[b,a]=butter ( 1, 100 / (sampRate/2) );
rollRateSig = filter(b,a,rollRateSig);
pitchRateSig = filter(b,a,pitchRateSig);
setpoint0u = filter(b,a,setpoint0u);

# sample interval in seconds
sampInt = 1 / sampRate;

# these globals are used by function hstarStim which is invoked
# by function leasqr()
# the "signal" is modeled as h * gStim and leasqr finds the parameters
# p1,p2,p3 which result in the best match to roll/pitchStim
global gfitrng = [0:sampInt:impLen]';

label = "Angular rate";
fprefix = "rate_";
rollStim = rollSetpoint(1:nsamp);
rollSig = rollRateSig;
pitchStim = pitchSetpoint(1:nsamp);
pitchSig = pitchRateSig;

# perform a least-squares fit to find the 3 parameters for the impulse response model
# the model is h(t) = p3 exp(-p1 t) sin(p2 t)
pin=[2, 15, .1];
stol = .0001;
niter = 50;
fit2rng = [1:nsamp]';
global gstim = rollStim;
[f,pr,cvg,iter,corp,covpr]=leasqr(fit2rng,rollSig,pin,"hstarStim",stol,niter);
if (cvg) 
  disp("roll fit converged"); 
else
  disp("roll fit diverged"); 
endif

fit2rng = [1:nsamp]';
# note that keyword "global" MUST NOT be used this time
gstim = pitchStim;
[f,pp,cvg,iter,corp,covpr]=leasqr(fit2rng,pitchSig,pin,"hstarStim",stol,niter);

rollw0 = pr(2);
pitchw0 = pp(2);
rollF0 = rollw0 / 2 / pi;
pitchF0 = pitchw0 / 2 / pi;
rollZeta = pr(1) / rollw0;
pitchZeta = pp(1) / pitchw0;

rollImpFit = pr(3) * decaysin(gfitrng, pr);
pitchImpFit = pp(3) * decaysin(gfitrng, pp);
if (cvg) 
  disp("pitch fit converged"); 
else
  disp("pitch fit diverged"); 
endif

# compute transfer functions
Nfft = 2048;
TFrollSig = fft(rollImpFit, Nfft);
TFpitchSig = fft(pitchImpFit, Nfft);
freqInt = (1/sampInt) / Nfft;
xRange = [1:20 / freqInt];
fRange = [0:xRange(end)-1] * freqInt;

rLimit = find(arg(TFrollSig)<(-pi/2));
if ((size(rLimit,1)>0) && rLimit(1) < length(fRange)) 
	rLimit = fRange(rLimit(1));
else 
	rLimit = 0; 
endif
pLimit = find(arg(TFpitchSig)<(-pi/2));
if ((size(pLimit,1)>0) && pLimit(1) < length(fRange)) 
	pLimit = fRange(pLimit(1));
else 
	pLimit = 0; 
endif

# load PX4 parameters
load([basePath "/parameters.octave"],
      "MC_ROLLRATE_P",
      "MC_ROLLRATE_I",
      "MC_ROLLRATE_D",
      "MC_PITCHRATE_P",
      "MC_PITCHRATE_I",
      "MC_PITCHRATE_D");

disp(["test " label " Response"]);

peakImp = max(rollImpFit);
lagRollImp = find(rollImpFit==peakImp)
rollModel = fftfilt(rollImpFit, rollStim);

rollStimMean = mean(rollStim);
Nlags = 500;
[RrollModel, lag] = xcorr(rollModel-mean(rollModel), rollStim-rollStimMean, Nlags);
RrollModel /= max(RrollModel);
lagRoll = findPeakLag(RrollModel, lag)

peakImp = max(pitchImpFit);
lagPitchImp = find(pitchImpFit==peakImp)
pitchModel = fftfilt(pitchImpFit, pitchStim);

pitchStimMean = mean(pitchStim);
[RpitchModel, lag] = xcorr(pitchModel-mean(pitchModel), pitchStim-pitchStimMean, Nlags);
RpitchModel /= max(RpitchModel);
lagPitch = findPeakLag(RpitchModel, lag)

dlyRoll = lagRollImp * sampInt;
dlyPitch = lagPitchImp * sampInt;

lagx = sampInt * lag;

# more robust method of measuring response latency
global gresponse = rollModel;
global gstimulus = rollStim;
global gmaxDelay = 4 * lagRollImp;
dlyRoll = round(fminsearch("delaySSQ", [lagRoll])) * sampInt

gresponse = pitchModel;
gstimulus = pitchStim;
gmaxDelay = 4 * lagPitchImp;
dlyPitch = round(fminsearch("delaySSQ", [lagPitch])) * sampInt

figPos = [200,350,512,512];
figure(figNum++, "Position", figPos);
plot( 
	[dlyRoll,dlyRoll],[.8,1],"-b", [dlyPitch,dlyPitch],[.8,1],"-r",
	lagx, RrollModel, "-bo", lagx, RpitchModel, "-ro"
	);
legend("roll", "pitch", "Location", "southeast");
title(sprintf("%s crosscorr. roll lag: %i, pitch lag: %i", label, lagRoll, lagPitch));
axis([0,2*lagPitch*sampInt,.8,1]);
xlabel("seconds");
axis("tight"); grid on;
hgsave([basePath "/" fprefix "crossCorr.ofig"])

disp(sprintf("max rollSig: %5.2f, max roll Stimulus: %5.2f, max rollModel: %5.2f", max(rollSig), max(rollStim), max(rollModel)));

varRoll = var(rollSig);
varRollError = var(rollSig - rollModel);
varRollPred = var(rollModel);
varPitch = var(pitchSig);
varPitchError = var(pitchSig - pitchModel);
varPitchPred = var(pitchModel);
varFracRoll = 1 - varRollError/varRoll;
varFracPitch = 1 - varPitchError/varPitch;

disp(sprintf("modeled variance:: roll: %5.2f, pitch: %5.2f", varFracRoll, varFracPitch));
yunits = "degrees/sec";

g0ts = g0t(1:nsamp);
figPos = [200,300,1200,512];
figure(figNum++, "Position", figPos);
subplot(2,1,1);
cScale = sum(rollImpFit)
plot(g0ts, rollModel*rad2deg, g0ts, rollSig*rad2deg, '.-', g0ts+dlyRoll, cScale * rollStim*rad2deg);
legend("rollModel", "roll", "delayed Control");
axis("tight")
title(sprintf("%s Control, model and actual: roll delay=%4.3f, pitch delay=%4.3f, modeled variance roll %4.2f, pitch %4.2f", label, dlyRoll, dlyPitch, varFracRoll, varFracPitch));
ylabel(yunits);


subplot(2,1,2);
cScale = sum(pitchImpFit)
plot(g0ts, pitchModel*rad2deg, g0ts, pitchSig*rad2deg, '.-', g0ts+dlyPitch, cScale * pitchStim*rad2deg);
axis("tight")
legend("pitchModel", "pitch", "delayed Control");
ylabel(yunits);
grid on;
%print([basePath "/" fprefix "tracking.png"], "-S1200,512")
hgsave([basePath "/" fprefix "tracking.ofig"])

%input("hit return to continue");

figPos = [200,400,1024,900];
figure(figNum++, "Position", figPos);
subplot(3,1,1);
imprng = [0:sampInt:impLen];
ximp = [1:1+impLen/sampInt];
plot(imprng, rollImpFit(ximp), "-b.", imprng, pitchImpFit(ximp), "-r.");
tLine1 = [label " impulse response --- " sprintf("Roll control latency: %5.3f --- Pitch control latency: %5.3f\n", dlyRoll, dlyPitch)];
      
tLine2 = sprintf(" roll damping: %4.2f, f0: %4.2f Hz, 90deg lag at %4.1f Hz  --- pitch damping: %4.2f, f0: %4.2f Hz, 90deg lag at %4.1f Hz\n",
          rollZeta, rollF0, rLimit, pitchZeta, pitchF0, pLimit);

tLine3 = sprintf("time range: [%5.3f,%5.3f] sec, RollRate P: %5.3f, I: %5.3f, D: %5.4f --- PitchRate P: %5.3f, I: %5.3f, D: %5.4f\n",
      startTime, endTime,
      MC_ROLLRATE_P,
      MC_ROLLRATE_I,
      MC_ROLLRATE_D,
      MC_PITCHRATE_P,
      MC_PITCHRATE_I,
      MC_PITCHRATE_D);
      
title([tLine1 tLine2 tLine3]);
legend("roll", "pitch","location","northeast");
legend("boxoff");
xlabel("seconds");
axis("tight"); grid on;

subplot(3,1,2);
semilogx(fRange, 20*log10(abs(TFrollSig(xRange))), "-b", fRange, 20*log10(abs(TFpitchSig(xRange))), "-r",
     [rLimit,rLimit], [-20,20], "-b", [pLimit,pLimit], [-20,20], "-r");
title([label " transfer function magnitude"]);
legend("roll", "pitch","location","northeast");
legend("boxoff");
xlabel("Hz");
ylabel("dB");
axis([fRange(2) 20 -20 20]); grid on;

subplot(3,1,3);
semilogx(fRange, unwrap(arg(TFrollSig(xRange)))*rad2deg, "-b", fRange, unwrap(arg(TFpitchSig(xRange)))*rad2deg, "-r",
     [rLimit,rLimit], [-180,0], "-b", [pLimit,pLimit], [-180,0], "-r");
title([label " transfer function phase"]);
legend("roll", "pitch","location","northeast");
legend("boxoff");
xlabel("Hz"); ylabel("degrees");
axis([fRange(2) 20 -180 0]); grid on;
set(gca, 'YTick', [0 -45 -90 -135 -180])
%print([basePath "/" fprefix "impulse.png"], "-S1200,900")
hgsave([basePath "/" fprefix "impulse.ofig"])

input("hit return to exit");
