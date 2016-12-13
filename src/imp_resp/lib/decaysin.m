function y = decaysin(x, par)
	y = exp(-par(1)*x) .* sin(par(2)*x);
endfunction

