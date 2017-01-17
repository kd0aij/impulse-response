function [t, vec] = loadAttSP(prefix, instance, basePath)
# load CSV file generated by ULogReader

filename = [basePath prefix "vehicle_attitude_setpoint_" num2str(instance) ".csv"];
printf ("input file: %s\n", filename);

data = tdfread(filename);
t = data.timestamp / 1e6;
vec = [data.roll_body, data.pitch_body, data.yaw_body];

end