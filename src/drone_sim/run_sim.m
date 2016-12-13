# simulate a drone using pyhsical first pricinples
clear all; close all;clc;
source("rotation.m");
source("attitude_controller.m")

params.num_states = 17; # 3 pos, 3 vel, 4 quat, 3 omega, 4 omega_rot

# define indexes how states are ordered in the state vector
params.mp.q = [1:4];          # unit attitude quaternion
params.mp.p = [5:7];          # local position NED
params.mp.v = [8:10];         # velocity NED
params.mp.omg = [11:13];      # body angular velocity
params.mp.omg_rot = [14:17];   # rotor angular velocity

# define motor configuration
# x is forward axis, y axis points to right
# the motor numbering follows the PX4 convention
params.mot.m1_pos_x = 0.15;
params.mot.m1_pos_y = 0.15;

params.mot.m2_pos_x = -0.15;
params.mot.m2_pos_y = -0.15;

params.mot.m3_pos_x = 0.15;
params.mot.m3_pos_y = -0.15;

params.mot.m4_pos_x = -0.15;
params.mot.m4_pos_y = 0.15;

params.mot.x_pos = [params.mot.m1_pos_x, params.mot.m2_pos_x, params.mot.m3_pos_x, params.mot.m4_pos_x];
params.mot.y_pos = [params.mot.m1_pos_y, params.mot.m2_pos_y, params.mot.m3_pos_y, params.mot.m4_pos_y];

params.mot.num_motors = 4;

# define parameters
params.m = 1.0;   # mass of drone
params.g = 9.81;  # gravity
params.I = zeros(3);  # inertia matrix
params.I(1,1) = 0.001;
params.I(2,2) = 0.001;
params.I(3,3) = 0.003;

params.I_p = zeros(3);  # inertia matrix of propeller
params.I_p(1,1) = 0;
params.I_p(2,2) = 0;
params.I_p(3,3) = 1e-6;

params.tau_rot = 0.1;   # motor time constant (from desired rpm to acual)
params.mot.k_mot = 1e-3;  # motor constant, thrust = k_mot * rmp^2
params.mot.k_t = 0.001;   # motor torque constant, torque = k_t * thrust
params.mot.omega_max = 80;

params.k_drag = 0.01;     # drag constant (linear relationship between body rate z and drag torque)

# control parameters
params.ctrl.att_p = [4;4;4];
params.ctrl.rate_p = [0.005;0.005;0.005];

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
q_sp = [cos(-0.2); sin(-0.2); 0;0];


# simulation
sim_time = 5;
step_size = 0.01;

STATE = zeros(params.num_states, sim_time / step_size);

index = 1;
for i=0:0.01:sim_time
    STATE(:, index) = state;
    
    # attitude conrol
    rate_des = control_attitude(state(params.mp.q), q_sp, params);
    control = control_rates(state(params.mp.omg), rate_des, params);
    input = control_to_omega(control, params) + ones(4,1).*45;

    # most simple integration, in future use ode solver here
    state_new = state + eval_dynamics(state, input, params) * step_size;
    state_new(params.mp.q) = state_new(params.mp.q)./(norm(state_new(params.mp.q)));
    state = state_new;
    index = index + 1;
end


# generate plots
sim_time = 0:step_size:sim_time;

plot(sim_time, STATE(params.mp.omg(1), :));
xlabel("time in seconds");
ylabel("body x rate in rad/s");

figure();
plot(sim_time, STATE(params.mp.omg(2), :));
xlabel("time in seconds");
ylabel("body y rate in rad/s");

figure();
plot(sim_time, STATE(params.mp.omg(3), :));
xlabel("time in seconds");
ylabel("body z rate in rad/s");