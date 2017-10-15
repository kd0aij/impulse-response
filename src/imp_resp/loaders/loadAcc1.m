function [t, vec, data] = loadAcc1(basePath)
# load CSV file generated by mavlogdump.py

data = tdfread([basePath "/acc1.csv"]);
t = data.timestamp;
vec = [data.AccX, data.AccY, data.AccZ];

end