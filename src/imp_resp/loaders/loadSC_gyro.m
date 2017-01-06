function [t, vec] = loadSC_gyro(prefix, instance, basePath)
# load CSV file generated by ULogReader

filename = [basePath prefix "sensor_combined_" num2str(instance) ".csv"];
printf ("input file: %s\n", filename);

data = tdfread(filename);
t = data.timestamp / 1e6;
vec = [data.gyro_rad_0_, data.gyro_rad_1_, data.gyro_rad_2_];

end