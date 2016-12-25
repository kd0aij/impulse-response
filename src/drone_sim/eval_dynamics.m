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
## @deftypefn {Function File} {@var{retval} =} eval_dynamics (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Roman <roman@roman-ThinkPad-T560>
## Created: 2016-12-09


# calculate the time derivative of the state vector according to our
# non-linear model f = f(state, input, params)
function [f] = eval_dynamics (state, input, params)
    # assign local vectors
    pos = state(params.mp.p);
    vel = state(params.mp.v);
    omega = state(params.mp.omg);
    omega_rot = state(params.mp.omg_rot);
    quat = state(params.mp.q);
    tau_rot = params.tau_rot;

    I = params.I;
    I_p = params.I_p;

    # get motor thrust vector in body frame, rotate to earth frame
    thrust = calc_motor_thrust_body(omega_rot, params);
    thrust = quat2rot(quat) * thrust;

    # derivative of position is current velocity
    if params.sim.p
        pos_dot = vel;
    else
         pos_dot = [];
    end
    
    if params.sim.v
        vel_dot = 1 / params.m .* (thrust + [0;0;params.g]);
    else
        vel_dot = [];
    end

    # in the referrenced paper the term omega_rot_tot corresponds to
    # sum(I_p * (omega_b + omega_mot(i)))
    omega_rot_tot = zeros(3,1);
    for i=1:params.mot.num_motors
        
        sign = -1;
        
        if i > 2
          sign = 1;
         end
        omega_rot_tot = omega_rot_tot + I_p * (omega + [0;0;omega_rot(i)*sign]);
    
    end
    
    # calculate torque created by the propellers and the drag torque due to body z rate
    if params.sim.omg
        moment = calc_motor_moment(omega_rot, params) + calc_drag_torque(omega, params);
        omega_dot = inv(I) * (moment - cross(omega, I*omega + omega_rot_tot));
    else
        omega_dot = [];
    end
    
    # motor rotational dynamics are modelled as first order system with time constant
    if params.sim.omg_rot
        omega_rot_dot = 1/tau_rot * (input - omega_rot);
    else
        omega_rot_dot = [];
    end

    # quaternion derivative
    if params.sim.q
      quat_dot = quat_dot(quat, omega);
    else
      quat_dot = [];
    end

    # return time derivative of state vector
    f = [quat_dot;pos_dot;vel_dot;omega_dot;omega_rot_dot];


# calculate the total thrust force produced by the motors in body frame
function [out] = calc_motor_thrust_body(omega_rot, params)
    out = zeros(3, 1);

    # thrust is modelled as thrust = k_mot * rmp^2
    for i = 1:params.mot.num_motors
      out(3) = out(3) - params.mot.k_mot * omega_rot(i) * omega_rot(i);
    end


endfunction

# calculate the total moment created by the motors in body frame
function [out] = calc_motor_moment(omega_rot, params)
    out = zeros(3,1);
    for i = 1:params.mot.num_motors
        thrust = [0;0;-params.mot.k_mot * omega_rot(i) * omega_rot(i)];
        arm = [params.mot.x_pos(i);params.mot.y_pos(i);0];
        out = out + cross(arm, thrust);
        
        # the torque produced by the propeller is proportional to the thrust
        # it produces
        sign = 1;
        if i > 2
            sign = -1;
        end
        out(3) = out(3) + sign * params.mot.k_t * params.mot.k_mot * omega_rot(i) * omega_rot(i);
        
    end

endfunction

# calculate the drag torque generated when the drone is rotating around it's z
# body axis
function [out] = calc_drag_torque(omega, params)

    out = zeros(3,1);
    out(3) = -params.k_drag * omega(3); 

endfunction

endfunction

