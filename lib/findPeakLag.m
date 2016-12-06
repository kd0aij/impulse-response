function pkLag = findPeakLag(Rx, lag)
    index = find(Rx == 1);
    if (length(index) == 1)
        pkLag = lag(index);
    else
        pkLag = lag(end);
    endif
endfunction
