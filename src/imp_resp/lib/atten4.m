function y = atten4(x)
  x2 = x.*x;
  x4 = x2.*x2;
	y = 1 - x4 / 2;
endfunction

