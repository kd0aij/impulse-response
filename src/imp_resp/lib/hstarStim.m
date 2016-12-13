function y = hstarStim(x, par)
	global gfitrng;
	global gstim;
	h = par(3) * decaysin(gfitrng, par);
	y = fftfilt(h, gstim);
endfunction
