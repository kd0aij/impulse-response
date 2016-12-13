source("rotation.m");

# compute desired body rates given vehicle attitude and attitude
# setpoint. rates are calculated such that attitude behaves like
# first order system with time constant 1/params.ctrl.att_p
function [rate_des] = control_attitude(q, q_sp, params)

q_sp_inv = [q_sp(1);-q_sp(2);-q_sp(3);-q_sp(4)];

q_error = mult_quat(q, q_sp_inv);

rate_des = -sign(q_error(1)) .* q_error(2:end) .* params.ctrl.att_p;

endfunction

# compute control signal for the three body axes given
# vehicle body rates and desired body rates
function [out] = control_rates(rates, rates_des, params)

    out = params.ctrl.rate_p .* (rates_des - rates);

endfunction

# compute desired motor speeds diven moment control vectorize
# this does not handle thrust yet
function [out] = control_to_omega(control, params)

    mixer = [-0.707107, 0.707107, 1.000000; 0.707107, -0.707107,  1.000000;0.707107,  0.707107, -1.000000;-0.707107, -0.707107, -1.000000];
    out = mixer * control .*params.mot.omega_max;
    for i = 1:length(out)
        if out(i) < 0
            out(i) = 0;
        end
        
        if (out(i) > params.mot.omega_max)
            out(i) = params.mot.omega_max;
        end
    end
endfunction