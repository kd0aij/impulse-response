# analyze logging intervals using timestamps "x"
function [average, variance, deltas, drop_lengths, drops] = interval_analysis(x)
  nsamples = size(x)(1)
  duration = x(end) - x(1)
	deltas = (advanceOne(x) .- x)(1:end-1);
  average = mean(deltas);
  variance = var(deltas);
  drops = deltas > (2 * average);
  ndrops = nnz(drops);
  drop_lengths = deltas(drops);
endfunction

# shift one sample to left: x(2), x(3), x(4), ... x(n+1)
function y = advanceOne(x)
	y =  cat(1, x(2:end,:), zeros(1, size(x)(2)));
endfunction
