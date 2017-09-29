# calculate the sum of squared differences between x and y for an integer
# delay of "delay".
# Uses only the first M-100 samples of x (M is length(gresponse)-gmaxDelay )
# Limitation: will fail if delay < -100 or delay > gmaxDelay

function s = delaySSQ(delay)
  global gresponse;
  global gstimulus;
  global gmaxDelay;
  M = length(gresponse) - gmaxDelay - 100;
  d = min(round(delay), gmaxDelay)
  s = sum((gresponse(d+100:d+M+100) - gstimulus(100:M+100)).^2); 
endfunction

