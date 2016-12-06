function [ut, uvec, sample_interval, jitter, drop_idx, drop_len] = resample(time, yvec, sample_rate=0, doplot=false)

start_time = time(1);
end_time = time(end);

# report dropouts
dt = diff(time);
sample_interval = median(dt);
if (sample_rate != 0)
  sample_interval = 1 / sample_rate;
endif
jitter = std(dt);
drop_idx = find((dt-sample_interval) > (1.5*sample_interval));
drop_len = dt(drop_idx);

# resample to uniform rate in Hz
ut = linspace(start_time, end_time, 1 + sample_rate * (end_time - start_time));
uvec = interp1(time, yvec, ut, 'pchip'); 

if (doplot)
  plot(time, yvec, '-bd', ut, uvec, 'ko');
  legend("nonuniform", "resampled");
  xlabel("seconds");
  ylabel("rad/sec");
endif