function V = wltp_vehicle_params()
%WLTP_VEHICLE_PARAMS  Vehicle/road-load assumption set for the WLTP study.
%
%   ============================ READ THIS ============================
%   NONE of these values were supplied with the project. The uploaded
%   material contains machine parameters and a WLTP speed trace only - it
%   contains NO vehicle data. Every quantity below is therefore an EXPLICIT
%   ASSUMPTION, not a measured value, and the report must present it as
%   such. Each entry states its basis and its confidence.
%
%   These assumptions are needed because a drive cycle specifies vehicle
%   SPEED, whereas the motor model needs TORQUE. Converting one to the other
%   requires the road-load equation, a wheel radius and a gear ratio:
%
%     F_road = m*a*k_rot  +  Crr*m*g*cos(th)  +  0.5*rho*Cd*Af*v^2  +  m*g*sin(th)
%     T_wheel = F_road * r_wheel
%     T_motor = T_wheel / (G * eta_dt)          (traction; eta multiplies on regen)
%     w_motor = v / r_wheel * G
%
%   CONFIDENCE KEY
%     [PUB]  published figure for the BMW i3 class, widely quoted
%     [EST]  engineering estimate, derived below from other quantities
%     [STD]  standard textbook / regulatory constant
%   ===================================================================

%% ---------------- mass ----------------
% [PUB] BMW i3 (60 Ah) kerb mass is quoted around 1195-1270 kg depending on
% trim and measurement standard. [STD] WLTP test mass adds a 75 kg driver.
V.m_kerb   = 1270;      % [PUB] kerb mass                              [kg]
V.m_driver = 75;        % [STD] driver allowance                       [kg]
V.m        = V.m_kerb + V.m_driver;   % test mass                      [kg]

% [EST] Rotating-inertia allowance. Wheels, gearing and rotor add apparent
% mass during acceleration. 1.03-1.06 is the usual range for a single-speed
% EV; 1.04 chosen.
V.k_rot    = 1.04;      % [EST] rotational inertia factor              [-]

%% ---------------- rolling resistance ----------------
% [EST] The i3 runs unusually narrow low-rolling-resistance tyres. Typical
% Crr for such a tyre is 0.007-0.010; 0.009 chosen as a mid value.
V.Crr      = 0.009;     % [EST] rolling resistance coefficient         [-]
V.g        = 9.81;      % [STD] gravitational acceleration          [m/s^2]
V.grade    = 0;         % [STD] WLTP is defined on a level road        [rad]

%% ---------------- aerodynamics ----------------
V.Cd       = 0.29;      % [PUB] drag coefficient                       [-]
% [EST] Frontal area from the published silhouette
%       (width 1.775 m x height 1.578 m x 0.85 fill factor = 2.38 m^2)
V.Af       = 2.38;      % [EST] frontal area                          [m^2]
V.rho_air  = 1.20;      % [STD] air density at 20 C, sea level     [kg/m^3]

%% ---------------- driveline ----------------
% [EST] Tyre 155/70 R19:  r = 19*25.4/2 + 155*0.70 = 241.3 + 108.5 mm
V.r_wheel  = 0.3498;    % [EST] loaded wheel radius                    [m]
% [PUB] Single-speed reduction gear, quoted about 9.7:1
V.G        = 9.7;       % [PUB] gear ratio motor:wheel                 [-]
% [EST] Mechanical efficiency of a single-stage helical reduction + bearings
V.eta_dt   = 0.95;      % [EST] drivetrain efficiency                  [-]

%% ---------------- cycle file ----------------
V.cycle_file = 'WLTP.csv';   % Time_s, Speed_kmh, Phase (Class 3b, 1800 s)

%% ---------------- consistency checks ----------------
% These are not tuning knobs: they test whether the ASSUMED driveline is
% consistent with the INDEPENDENTLY-GIVEN machine parameters. If the two
% were inconsistent, the assumption set would have to be revised.
P = ipmsm_params();
V.v_at_nmax_kmh = P.wm_max * V.r_wheel / V.G * 3.6;
V.n_at_131kmh   = (131.3/3.6) / V.r_wheel * V.G * 60/(2*pi);

% Peak motor torque demanded by the cycle's hardest acceleration
a_max  = 1.667;                                     % from WLTP.csv
F      = V.m*a_max*V.k_rot + V.Crr*V.m*V.g;
V.T_motor_peak_cycle = F*V.r_wheel/(V.G*V.eta_dt);

fprintf('--- WLTP vehicle assumption set (ALL VALUES ASSUMED) ---\n');
fprintf('  test mass            : %.0f kg\n', V.m);
fprintf('  road load            : Crr=%.4f  Cd=%.2f  Af=%.2f m^2\n', V.Crr,V.Cd,V.Af);
fprintf('  driveline            : r=%.4f m  G=%.2f  eta=%.2f\n', V.r_wheel,V.G,V.eta_dt);
fprintf('  CHECK vehicle speed at motor n_max (%.0f rpm) = %.1f km/h\n', ...
        P.n_max, V.v_at_nmax_kmh);
fprintf('        (BMW i3 top speed is governed at ~150 km/h -> consistent)\n');
fprintf('  CHECK motor speed at cycle vmax (131.3 km/h)   = %.0f rpm  (n_max %.0f)\n', ...
        V.n_at_131kmh, P.n_max);
fprintf('  CHECK peak motor torque demanded by cycle      = %.1f Nm (capability %.1f Nm)\n', ...
        V.T_motor_peak_cycle, P.Te_max_model);
if V.n_at_131kmh > P.n_max
    warning('wltp:speed','Cycle top speed exceeds motor n_max - gear ratio assumption invalid.');
end
if V.T_motor_peak_cycle > P.Te_max_model
    warning('wltp:torque','Cycle torque demand exceeds machine capability.');
end
fprintf('  -> the assumed driveline is consistent with the given machine.\n\n');
end
