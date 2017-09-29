function y = expo(x, par)
  if (mod(par(2),2) == 0)
	  y = par(1) * sign(x) .* x.^par(2) + (1 - par(1)) * x;
  else
	  y = par(1) * x.^par(2) + (1 - par(1)) * x;
  endif
endfunction
