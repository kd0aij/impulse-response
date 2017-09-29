function y = unk_filter(n, a)
  y = zeros(1,n);
  for i = 2:n
    y(i) = a * (y(i-1) + 1);
  endfor
endfunction