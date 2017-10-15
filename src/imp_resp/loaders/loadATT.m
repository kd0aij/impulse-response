function [t, rpy, asp, data] = loadATT(basePath)
# load CSV file generated by mavlogdump.py

data = tdfread([basePath "/att.csv"]);
t = 1e-6 * data.TimeUS;
rpy = [data.Roll/100, data.Pitch/100, data.Yaw/100];
asp = [data.DesRoll/100, data.DesPitch/100, data.DesYaw/100];

end