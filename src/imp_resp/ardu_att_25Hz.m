pkg load optim
pkg load signal
pkg load mapping

# this script must be run from its location in the local repo
addpath("./loaders:./lib");
close all;
clear all;

%default_basePath = "/home/markw/Desktop/flightlogs_Desktop/arducopter/aquaquad/oneshot";
default_basePath = "/home/markw/Desktop/flightlogs_Desktop/arduplane/Stryker/digitalServos/compensation";

rad2deg = 180/pi;

# set default run parameters
# desired uniform sample rate in Hz; should be less than average sample rate
rateOnly = false;
sampRate = 25
nplots = 2

# set to true when start/endTime are correct
rangeSelected = 0;
# length in seconds of impulse response plot
impLen = 1.5

# path to CSV files
fflush (stdout);
choice = input("load example dataset? (N/y):", "s");
if length(choice) == 0 || (choice != 'Y' && choice != 'n')
  basePath = default_basePath;
else
  choice = input("choose example [1,2]: ");
  if choice == 1
    basePath = "../../sample_ulogs/S250AQ/2016-12-05/log002/"
    startTime = 148
    endTime = 160
  elseif choice == 2
    basePath = "../../sample_ulogs/AquaQuad/2016-09-17/"
    startTime = 167
    endTime = 178
  else
    basePath = "ulogs/S250_pixracer/2016-08-18/sess001/log3/"
  endif
endif

if strcmp(basePath, default_basePath)
  basePath = [uigetdir(basePath) "/"];
endif

# load the measured angles and setpoints
[ts, att0, att_sp0, data] = loadATT(basePath);

nsamples = size(att0)(1);
sigRange = [1:nsamples];

# find timespan of data
if (!exist('startTime') | !exist('endTime'))
  startTime = ts(1)
  endTime = ts(end)
else
  # find index span of gyro data
  startOffset = 1;
  while (ts(startOffset) < startTime) startOffset++; endwhile
  endOffset = startOffset;
  while (ts(endOffset) < endTime) endOffset++; endwhile
  sigRange = [startOffset:endOffset];
endif

figNum = 1;
figure(figNum++, "Position", [1,200,1200,480]);
subplot(nplots,1,1);
# ticks, rollRate, pitchRate
plot(ts, att0(:,1), "-b", ts, att0(:,2), "-r");
axis("tight"); title("raw roll and pitch rate data extents");
xlabel("seconds");
grid("on"); grid("minor");

while (true)
  subplot(nplots,1,2);
    plot(ts(sigRange), att0(sigRange,1), "-b", ts(sigRange), att0(sigRange,2), "-r");
    axis("tight"); title("roll and pitch angle data subset");
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
  while (ts(startOffset) < startTime) startOffset++; endwhile
  endOffset = startOffset;
  while (ts(endOffset) < endTime) endOffset++; endwhile
  sigRange = [startOffset:endOffset];
endwhile

# resample to uniform rate, over the time range [startTime, endTime]
  [att0tu, att0u] = resample2(startTime, endTime, ts, att0, sampRate);
  [as0tu, att_sp0u] = resample2(startTime, endTime, ts, att_sp0, sampRate);

  nsamples = size(att_sp0u)(1);
  sigRange = [1:nsamples];

  # unwrap the Euler angles for analysis
  att_sp0u = unwrap(att_sp0u);
  att0u = unwrap(att0u);

  rollAttSP = att_sp0u(sigRange,1);
  rollAngleSig = att0u(sigRange,1);

  pitchAttSP = att_sp0u(sigRange,2);
  pitchAngleSig = att0u(sigRange,2);

nsamples = size(att0u)(1);
sigRange = [1:nsamples];

# sample interval in seconds
sampInt = 1 / sampRate;

# these globals are used by function hstarStim which is invoked
# by function leasqr()
# the "signal" is modeled as h * gStim and leasqr finds the parameters
# p1,p2,p3 which result in the best match to roll/pitchStim
global gfitrng = [0:sampInt:impLen]';

  label = "Angle";
  fprefix = "tilt_";
  rollStim = rollAttSP(:);
  rollSig = rollAngleSig;
  pitchStim = pitchAttSP(:);
  pitchSig = pitchAngleSig;

# perform a least-squares fit to find the 3 parameters for the impulse response model
# the model is h(t) = p3 exp(-p1 t) sin(p2 t)
pin=[2, 8, .1];
stol = .0001;
niter = 50;
fit2rng = [1:length(rollSig)]';
global gstim = rollStim;
[f,pr,cvg,iter,corp,covpr]=leasqr(fit2rng,rollSig,pin,"hstarStim",stol,niter);
if (cvg) 
  disp("roll fit converged"); 
else
  disp("roll fit diverged"); 
endif

fit2rng = [1:length(pitchSig)]';
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
%load([basePath "/parameters.octave"],
%      "MC_ROLLRATE_P",
%      "MC_ROLLRATE_I",
%      "MC_ROLLRATE_D",
%      "MC_PITCHRATE_P",
%      "MC_PITCHRATE_I",
%      "MC_PITCHRATE_D");

disp(["test " label " Response"]);

peakImp = max(rollImpFit);
lagRollImp = find(rollImpFit==peakImp)
rollModel = fftfilt(rollImpFit, rollStim);

rollStimMean = mean(rollStim);
Nlags = 100;
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
dlyRoll = round(fminsearch("delaySSQ", [lagRollImp])) * sampInt

global gresponse = pitchModel;
global gstimulus = pitchStim;
global gmaxDelay = 4 * lagPitchImp;
dlyPitch = round(fminsearch("delaySSQ", [lagPitchImp])) * sampInt

#rewrap Euler angles
  rollStim = wrapToPi(rollStim);
  rollSig = wrapToPi(rollSig);
  pitchStim = wrapToPi(pitchStim);
  pitchSig = wrapToPi(pitchSig);
  rollModel = wrapToPi(rollModel);
  rollStimMean = mean(rollStim);
  pitchStimMean = mean(pitchStim);

figPos = [200,350,512,512];
figure(figNum++, "Position", figPos);
plot( 
	[dlyRoll,dlyRoll],[.8,1],"-b", [dlyPitch,dlyPitch],[.8,1],"-r",
	lagx, RrollModel, "-bo", lagx, RpitchModel, "-ro"
	);
legend("roll", "pitch", "Location", "southeast");
title(sprintf("%s crosscorr. roll lag: %i, pitch lag: %i", label, lagRoll, lagPitch));
%axis([0,2*lagPitch*sampInt,.8,1]);
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
  yunits = "degrees";

figPos = [200,300,1200,512];
figure(figNum++, "Position", figPos);
subplot(2,1,1);
cScale = sum(rollImpFit)
plot(att0tu, rollModel*rad2deg, att0tu, rollSig*rad2deg, '.-', att0tu+dlyRoll, cScale * rollStim*rad2deg);
legend("rollModel", "roll", "delayed Control");
axis("tight")
title(sprintf("%s Control, model and actual: roll delay=%4.3f, pitch delay=%4.3f, modeled variance roll %4.2f, pitch %4.2f", label, dlyRoll, dlyPitch, varFracRoll, varFracPitch));
ylabel(yunits);


subplot(2,1,2);
cScale = sum(pitchImpFit)
plot(att0tu, pitchModel*rad2deg, att0tu, pitchSig*rad2deg, '.-', att0tu+dlyPitch, cScale * pitchStim*rad2deg);
axis("tight")
legend("pitchModel", "pitch", "delayed Control");
ylabel(yunits);
grid on;
%print([basePath "/" fprefix "tracking.png"], "-S1200,512")
hgsave([basePath "/" fprefix "tracking.ofig"])

input("hit return to continue");

figPos = [200,400,1024,900];
figure(figNum++, "Position", figPos);
subplot(3,1,1);
imprng = [0:sampInt:impLen];
ximp = [1:1+impLen/sampInt];
plot(imprng, rollImpFit(ximp), "-b.", imprng, pitchImpFit(ximp), "-r.");
tLine1 = [label " impulse response --- " sprintf("Roll control latency: %5.3f --- Pitch control latency: %5.3f\n", dlyRoll, dlyPitch)];
      
tLine2 = sprintf(" roll damping: %4.2f, f0: %4.2f Hz, 90deg lag at %4.1f Hz  --- pitch damping: %4.2f, f0: %4.2f Hz, 90deg lag at %4.1f Hz\n",
          rollZeta, rollF0, rLimit, pitchZeta, pitchF0, pLimit);

%tLine3 = sprintf(" RollRate P: %5.3f, I: %5.3f, D: %5.4f --- PitchRate P: %5.3f, I: %5.3f, D: %5.4f\n",
%      MC_ROLLRATE_P,
%      MC_ROLLRATE_I,
%      MC_ROLLRATE_D,
%      MC_PITCHRATE_P,
%      MC_PITCHRATE_I,
%      MC_PITCHRATE_D);
      
title([tLine1 tLine2]);
legend("roll", "pitch","location","southwest");
legend("boxoff");
xlabel("seconds");
axis("tight"); grid on;

subplot(3,1,2);
semilogx(fRange, 20*log10(abs(TFrollSig(xRange))), "-b", fRange, 20*log10(abs(TFpitchSig(xRange))), "-r",
     [rLimit,rLimit], [-20,20], "-b", [pLimit,pLimit], [-20,20], "-r");
title([label " transfer function magnitude"]);
legend("roll", "pitch","location","southwest");
legend("boxoff");
xlabel("Hz");
ylabel("dB");
axis([0.1 20 -20 20]); grid on;

subplot(3,1,3);
semilogx(fRange, unwrap(arg(TFrollSig(xRange)))*rad2deg, "-b", fRange, unwrap(arg(TFpitchSig(xRange)))*rad2deg, "-r",
     [rLimit,rLimit], [-180,0], "-b", [pLimit,pLimit], [-180,0], "-r");
title([label " transfer function phase"]);
legend("roll", "pitch","location","southwest");
legend("boxoff");
xlabel("Hz"); ylabel("degrees");
axis([0.1 20 -180 0]); grid on;
set(gca, 'YTick', [0 -45 -90 -135 -180])
%print([basePath "/" fprefix "impulse.png"], "-S1200,900")
hgsave([basePath "/" fprefix "impulse.ofig"])

input("hit return to exit");
