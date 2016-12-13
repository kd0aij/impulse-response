# calculate the sum of squared differences between x and y for an integer
# delay of "delay".
# Uses only the first M samples of x (M is length(grollModel)-gmaxDelay )
# Limitation: will fail if delay > gmaxDelay

function s = delaySSQ(delay)
  global gresponse;
  global gstimulus;
  global gmaxDelay;
  M = length(gresponse) - gmaxDelay;
  d = round(delay);
  s = sum((gresponse(d+1:d+M) - gstimulus(1:M)).^2); 
endfunction

