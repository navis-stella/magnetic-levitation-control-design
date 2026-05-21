% #########################################################################
%
%    Subscript:
%       Physical Constants and System Constraints
%
% #########################################################################

% Current limits for the electromagnets [A]
% Defines the operational range of the power electronics
I_min = 0;
I_max = 15; 

% Rated operating current value
I_work = 10;

% Definition of the gravitational acceleration vector [m/s²]
% Convention: The Y-axis is vertical, with the positive direction pointing upwards.
% Therefore, gravity acts in the negative Y-direction.
grav = [0; -9.80665; 0];

% Definition of magnet identifiers based on their installation position
% Format: [E/S: Side] [L/R: Left/Right] [O/U: Top(Oben)/Bottom(Unten)]
elMaglabels = {'ELO', 'ELU', 'ERO', 'ERU', 'SLO', 'SLU', 'SRO', 'SRU'};

% Sampling time for noise generation and discrete simulation [s]
% Corresponds to a system frequency of 10 kHz (Ts = 1e-4)
Ts = 1e-4; 

% Standard deviation of the Gaussian white noise [mm]
% Example: 0.002 mm represents 2 micrometers (μm) of sensor uncertainty
sensor_noise_std = 0.002;  

% Noise power for the "Band-Limited White Noise" block in Simulink/Simulation
% Calculation: Power Spectral Density (PSD) = Variance * Sampling Time
% Formula: P = (std_dev^2) * Ts
noise_power = sensor_noise_std^2 * Ts;