function vec = tdfread(filename)
# load CSV file generated by ULogReader

A = importdata(filename, ',', 1);
fieldNames = strsplit(A.textdata{1}, ',');
data = csvread(filename);
printf("number of data records: %d\n", size(data)(1));

# using field names, construct data vectors
for i=1:size(A.colheaders)(2)
  vec.(genvarname(fieldNames{i})) = A.data(:,i);
endfor

end