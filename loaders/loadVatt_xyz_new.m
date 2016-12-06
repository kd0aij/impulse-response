function [t, vec] = loadVatt_xyz(instance, basePath)
# load CSV file generated by ULogReader

filename = [basePath "/log002_vehicle_attitude_" num2str(instance) ".csv"];
printf ("input file: %s\n", filename);

A = importdata(filename, ',', 1);
fieldNames = strsplit(A.textdata{1}, ',');
data = csvread(filename);
printf("number of data records: %d\n", size(data)(1));

# using A.colheaders, construct timestamp and x,y,z vectors
tname = "timestamp";
xname = ["roll"];
yname = ["pitch"];
zname = ["yaw"];
for i=1:size(A.colheaders)(2)
  if (strcmp(tname, A.colheaders(i))) tindex = i; endif
  if (strcmp(xname, A.colheaders(i))) xindex = i; endif
  if (strcmp(yname, A.colheaders(i))) yindex = i; endif
  if (strcmp(zname, A.colheaders(i))) zindex = i; endif
endfor

t = A.data(:,tindex) / 1e6;
vec = [A.data(:,xindex) A.data(:,yindex) A.data(:,zindex)];

end