function y = attenuator(x, par)
	y = 1-exp(-par(1)*(1-x));
endfunction

