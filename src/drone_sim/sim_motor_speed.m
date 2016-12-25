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
## @deftypefn {Function File} {@var{retval} =} sim_motor_speed (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Roman <roman@roman-ThinkPad-T560>
## Created: 2016-12-25

function [RPM] = sim_motor_speed (time, rpm_setpoint, params)
    rpm_to_rad = 2*pi/60;

    # we only want motor dynamics
    params.sim.q = false;
    params.sim.omg = false;
    params.sim.v = false;
    params.sim.p = false;
    parms.sim.omg_rot = true;

    RPM = zeros(length(time), 1);

    # initialise state
    rpm_sim = ones(params.mot.num_motors,1) .*rpm_setpoint(1) * rpm_to_rad;

    for i = 1:length(time)-1
        RPM(i) = rpm_sim(1)./rpm_to_rad;
        
        dt = time(i+1) - time(i);
        
        input = ones(params.mot.num_motors,1) .* (rpm_setpoint(i) * rpm_to_rad);
        state = zeros(params.num_states, 1);
        state(params.mp.omg_rot) = rpm_sim;
        
        rpm_dot = eval_dynamics(state, input, params);
        
        rpm_sim = rpm_sim + rpm_dot.*dt;

    end

    RPM(end) = rpm_sim(1)./rpm_to_rad;
endfunction
