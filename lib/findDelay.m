# find the delay (integer number of samples) which minimizes the sum of squares
# of (x-y). Test delays from zero to N-1 samples.
# Uses only the first M-N samples of x (where M is the length of x and y)
function dly = findDelay(x, y, N)
  M = length(x) - N;
  r = zeros(N,1);
  for d = [0:N-1]
    r(d+1) = sum((y(d+1:d+M) - x(1:M)).^2); 
  endfor
  minError = r(1);
  dly = 1;
  for i = [2:N-1]
    if r(i) < minError
      minError = r(i);
      dly = i;
    endif
  endfor 
endfunction

