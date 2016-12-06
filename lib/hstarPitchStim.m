function y = hstarPitchStim(x, par)
	global gfitrng;
	global gpitchStim;
	h = par(3) * decaysin(gfitrng, par);
	y = fftfilt(h, gpitchStim);
endfunction

