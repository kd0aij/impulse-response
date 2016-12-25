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
## @deftypefn {Function File} {@var{retval} =} cost_motor_time_const (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Roman <roman@roman-ThinkPad-T560>
## Created: 2016-12-24

function [cost] = cost_motor_time_const (x, time_meas, rpm_meas, rpm_setpoint, params )
    # assign the parameter we want to estimate
    params.tau_rot = x;

    RPM = sim_motor_speed(time_meas, rpm_setpoint, params);

    cost = (RPM - rpm_meas')' * (RPM - rpm_meas');


endfunction
