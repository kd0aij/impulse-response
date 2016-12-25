# script to identify the motor time constant given the true measured
# motor speed and the motor speed setpoint
# the motor is modelled as a first order system as follows:
# speed_dot = 1/time_const * (speed_setpoint - speed)

clear all; close all;clc;

source("rotation.m")

# get a struct of parameters
params = init_params();

# create a random signal of motor speed setpoint and measured motor speed
# TODO: replace this with real data
T_rpm = 0:0.01:10;
T_rpm = T_rpm;
rpm_meas = 5000 + 3000.*sin(T_rpm);
rpm_setpoint = rpm_meas;

# low pass filter the measured motor speed to simulate lag
rpm_meas(1) = rpm_setpoint(1);
for i = 2: length(rpm_setpoint)
    rpm_meas(i) = 0.9 * rpm_meas(i-1) + 0.1 * rpm_setpoint(i);
end

# run fminsearch in order to find the motor time constant
res = fminsearch(@(x)cost_motor_time_const(x,T_rpm, rpm_meas, rpm_setpoint, params), 0.1);

# assign the optimal motor time constant and run a simulation for visual verification
params.tau_rot = res;
rpm_sim = sim_motor_speed(T_rpm, rpm_setpoint, params);

plot(T_rpm, rpm_meas, 'r');
hold on;
plot(T_rpm, rpm_sim);

