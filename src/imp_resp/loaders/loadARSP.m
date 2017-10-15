function [t, diffp, airspeed, data] = loadARSP(basePath)
# load CSV file generated by mavlogdump.py

data = tdfread([basePath "/arsp.csv"]);
t = 1e-6 * data.TimeUS;
diffp = [data.DiffPress];
airspeed = [data.Airspeed];

end