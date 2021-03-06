function [t, vec, data] = loadAccel_filt(prefix, instance, basePath)
# load CSV file generated by ULogReader

filename = [basePath prefix "sensor_accel_" num2str(instance) ".csv"];
printf ("input file: %s\n", filename);

data = tdfread(filename);
t = data.timestamp / 1e6;
vec = [data.x, data.y, data.z];

end
