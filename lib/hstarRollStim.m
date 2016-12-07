function y = hstarRollStim(x, par)
	global gfitrng;
	global grollStim;
	h = par(3) * decaysin(gfitrng, par);
	y = fftfilt(h, grollStim);
endfunction
