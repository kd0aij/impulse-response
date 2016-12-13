# impulse-response

[![Join the chat at https://gitter.im/kd0aij/impulse-response](https://badges.gitter.im/kd0aij/impulse-response.svg)](https://gitter.im/kd0aij/impulse-response?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

The src/drone_sim folder contains Octave code for simulating a vehicle.

The src/imp_resp folder contains Octave code for blackbox impulse response estimation.

imp_resp/Impulse_response.m estimates impulse response functions (angle, angular rate, angular acceleration) from PX4 logfiles in the .ulg format.

The sample_ulogs folder contains some sample logs which have been preprocessed using pyulog tool ulog2csv.py and ulog_params.py
and may be used to test Impulse-response.m by selecting example 1 or 2 at the prompts.