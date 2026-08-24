%% BMW i3 IPMSM Parameters
% Electrical parameters
Rs      = 5.3e-3;        % Stator phase resistance [Ohm]
Ld      = 0.090e-3;      % d-axis inductance [H]
Lq      = 0.255e-3;      % q-axis inductance [H]
lambda_m = 0.0385;       % PM flux linkage [Wb]
p       = 6;             % Pole pairs
% DC link
Vdc     = 360;           % DC-link voltage [V]
% Current limit used for our simplified model
Imax    = 430;           % Peak phase current [A]
% Mechanical parameters
J       = 0.06;          % Rotor inertia [kg*m^2] - ASSUMED
B       = 0.001;         % Viscous friction [N*m*s/rad] - ASSUMED
% Motor operating limits
Tmax    = 250;           % Maximum torque [N*m]
Pmax    = 125e3;         % Maximum power [W]
n_base  = 4800;          % Base/operating speed [rpm]
n_max   = 11400;         % Maximum speed [rpm]
% Derived quantities
DeltaL  = Lq - Ld;        % Saliency difference [H]
omega_base = 2*pi*n_base/60;  % Mechanical base speed [rad/s]
omega_max  = 2*pi*n_max/60;   % Mechanical maximum speed [rad/s]
f_e_base = p*n_base/60;       % Electrical base frequency [Hz]
f_e_max  = p*n_max/60;        % Electrical maximum frequency [Hz]
% SVPWM voltage limit
Vmax = Vdc/sqrt(3);           % Maximum fundamental phase voltage [V]

fprintf('Delta L = %.6f mH\n', DeltaL*1e3);
fprintf('Base speed = %.2f rad/s\n', omega_base);
fprintf('Maximum speed = %.2f rad/s\n', omega_max);
fprintf('Base electrical frequency = %.2f Hz\n', f_e_base);
fprintf('Maximum electrical frequency = %.2f Hz\n', f_e_max);
fprintf('Approx. voltage limit = %.2f V\n', Vmax);

%% STEP 2B: Basic Electromagnetic Torque Validation
% Test operating point
id_test = 0;      % d-axis current, A
iq_test = 100;     % q-axis current, A

Te_test = (3/2) * p * (lambda_m*iq_test + (Ld - Lq)*id_test*iq_test);

fprintf('\n--- Torque Validation ---\n');
fprintf('id = %.2f A\n', id_test);
fprintf('iq = %.2f A\n', iq_test);
fprintf('Te = %.4f Nm\n', Te_test);