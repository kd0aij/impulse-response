# derivative w.r.t. x is a vector of first differences: a_n = .5((a_n+1 - a_n) + (a_n - a_n-1))
# centered on sample n
function y = ddt(x)
	y = 0.5 * ((advanceOne(x) .- x) + (x .- delayOne(x)));
endfunction

# shift one sample to left: x(2), x(3), x(4), ... x(n+1)
function y = advanceOne(x)
	y =  cat(1, x(2:end,:), zeros(1, size(x)(2)));
endfunction
# shift one sample to right: x(-1), x(0), x(1), ... x(n-1)
function y = delayOne(x)
	y =  cat(1, zeros(1, size(x)(2)), x(1:end-1,:));
endfunction

