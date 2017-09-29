function y = boost4(x)
  a = .01^.25;
  x = ((1-a)*x)+a;
  x2 = x.*x;
  y = x2.*x2;
endfunction

