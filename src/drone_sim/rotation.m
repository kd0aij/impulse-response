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
## @deftypefn {Function File} {@var{retval} =} rotation (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Roman <roman@roman-ThinkPad-T560>
## Created: 2016-12-10

function [R] = quat2rot (q)
  q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
    
    R = [q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
           2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1);
           2*(q1*q3-q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2];
endfunction

function [q_dot] = quat_dot(q, omega)

    wx = omega(1);
    wy = omega(2);
    wz = omega(3);
    
    W =  [0, -wx, -wy, -wz; wx, 0, -wz, wy; wy, wz, 0, -wx; wz, -wy, wx, 0] .* 0.5;
    q_dot = W * q;
    

endfunction
