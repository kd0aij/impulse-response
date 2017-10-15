basePath = "/home/markw/Desktop/RFexplorer/test1"
flist=dir(basePath);

nfreqs=0;
nscans=0;
fulldata=[];
for i=[1:size(flist)]
  if (findstr(flist(i).name, '.csv') > 10)
    disp(flist(i).name);
    [startdate, starttime, enddate, endtime, data]=plotRF2(basePath, flist(i).name);
    if (nfreqs == 0)
      nfreqs=size(data)(2)
      startd=startdate
      startt=starttime
    endif
    nscans+=size(data)(1)
    fulldata = [fulldata; data];
    fflush(stdout());
  endif
endfor
endd=enddate
endt=endtime
figure()
waterfall(fulldata)
title("waterfall")
xlabel("freq bin")
ylabel("scan")
zlabel("dBm")
