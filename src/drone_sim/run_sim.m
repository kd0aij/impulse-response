# simulate a drone using pyhsical first pricinples
clear all; close all;clc;
source("rotation.m");
source("attitude_controller.m")

params = init_params();

# control parameters
params.ctrl.att_p = [4;4;4];
params.ctrl.rate_p = [0.005;0.005;0.005];
params.ctrl.rate_d = [0.00;0.000;0.000];

# state vector
state = zeros(params.num_states, 1);
state(params.mp.q) = [1;0;0;0];
state(params.mp.omg_rot(1)) = 45;
state(params.mp.omg_rot(2)) = 45;
state(params.mp.omg_rot(3)) = 45;
state(params.mp.omg_rot(4)) = 45;
state(params.mp.omg(1)) = 0.5;
state(params.mp.omg(2)) = -0.5;
state(params.mp.omg(3)) = 0.1;


# input vector (desired motor rotational speeds) in rad/s
input = ones(params.mot.num_motors,1).*50;

# desired attitude quaternion
q_sp = [cos(0.1);sin(0.1);0;0];


# simulation
sim_time = 5;
step_size = 0.005;

STATE = zeros(params.num_states, sim_time / step_size);
omega_prev = state(params.mp.omg);
RATE_SP = zeros(3, sim_time / step_size);
OMG_ROT_SP = zeros(4, sim_time / step_size);

index = 1;
for i=0:step_size:sim_time
    STATE(:, index) = state;
    
    # attitude control
    rate_des = control_attitude(state(params.mp.q), q_sp, params);
    RATE_SP(:, index) = rate_des;
    control = control_rates(state(params.mp.omg), rate_des, omega_prev, step_size, params);
    omega_prev = state(params.mp.omg);
    input = control_to_omega(control, params) + ones(4,1).*45;
    OMG_ROT_SP(:, index) = input;

    # most simple integration, in future use ode solver here
    state_new = state + eval_dynamics(state, input, params) * step_size;
    state_new(params.mp.q) = state_new(params.mp.q)./(norm(state_new(params.mp.q)));
    state = state_new;
    index = index + 1;
end


# generate plots
sim_time = 0:step_size:sim_time;

plot(sim_time, STATE(params.mp.omg(1), :));
hold on;
plot(sim_time, RATE_SP(1,:), 'r');
xlabel("time in seconds");
ylabel("body x rate in rad/s");

figure();
plot(sim_time, STATE(params.mp.omg(2), :));
hold on;
plot(sim_time, RATE_SP(2,:), 'r');
xlabel("time in seconds");
ylabel("body y rate in rad/s");

figure();
plot(sim_time, STATE(params.mp.omg(3), :));
hold on;
plot(sim_time, RATE_SP(3,:), 'r');
xlabel("time in seconds");
ylabel("body z rate in rad/s");

figure();
plot(sim_time, STATE(params.mp.p(1), :));
hold on;
plot(sim_time, STATE(params.mp.p(2), :));
plot(sim_time, STATE(params.mp.p(3), :));
xlabel("time in seconds");
ylabel("local position in m");

figure();
plot(sim_time, STATE(params.mp.omg_rot(1), :));
hold on;
plot(sim_time, STATE(params.mp.omg_rot(2), :));
plot(sim_time, STATE(params.mp.omg_rot(3), :));
plot(sim_time, STATE(params.mp.omg_rot(3), :));
xlabel("time in seconds");
ylabel("rotor angular velocities in rad/s");

figure()
plot(sim_time, STATE(params.mp.omg_rot(1), :));
hold on;
plot(sim_time, OMG_ROT_SP(1,:));
