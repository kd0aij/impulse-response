## Copyright (C) 2016 Roman
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {Function File} {@var{retval} =} init_params (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Roman <roman@roman-ThinkPad-T560>
## Created: 2016-12-25

function [params] = init_params ()

params.num_states = 17; # 3 pos, 3 vel, 4 quat, 3 omega, 4 omega_rot

# define indexes how states are ordered in the state vector
params.mp.q = [1:4];          # unit attitude quaternion
params.mp.p = [5:7];          # local position NED
params.mp.v = [8:10];         # velocity NED
params.mp.omg = [11:13];      # body angular velocity
params.mp.omg_rot = [14:17];   # rotor angular velocity

params.sim.p = true;
params.sim.v = true;
params.sim.q = true;
params.sim.omg = true;
params.sim.omg_rot = true;

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

params.tau_rot = 0.2;   # motor time constant (from desired rpm to acual)
params.mot.k_mot = 1e-3;  # motor constant, thrust = k_mot * rmp^2
params.mot.k_t = 0.001;   # motor torque constant, torque = k_t * thrust
params.mot.omega_max = 80;

params.k_drag = 0.01;     # drag constant (linear relationship between body rate z and drag torque)

endfunction
