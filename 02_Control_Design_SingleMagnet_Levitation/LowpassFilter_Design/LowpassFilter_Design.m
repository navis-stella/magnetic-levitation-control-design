
% Lowpass filter design (requires Signal Processing Toolbox; if not, use manual IIR)
order = 4;  % Filter order (2-4 for good tradeoff)
fc = 200;    % Cutoff frequency [Hz] 
Ts = 1e-4;  % Sample time [s] (match simulation step size)
fs = 1 / Ts;  % Sampling frequency

% Normalized cutoff
Wn = fc / (fs / 2);

% Butterworth coefficients
[b, a] = butter(order, Wn, 'low');

% Save to .mat file
save('lpf_params.mat', 'a', 'b');
disp('LPF parameters saved to lpf_params.mat');