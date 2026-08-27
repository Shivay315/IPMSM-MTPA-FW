function P = ipmsm_params()
%IPMSM_PARAMS  Single authoritative parameter set for the MTPA / FW study.
%
%   P = IPMSM_PARAMS() returns a struct with every machine, inverter and
%   simulation parameter used anywhere in this project. Nothing else in the
%   project may hard-code a machine parameter: the Simulink model is
%   generated from this file by BUILD_FOC_MODEL, and every analysis script
%   calls this function.
%
%   PARAMETER PROVENANCE -- IMPORTANT FOR THE REPORT
%   ------------------------------------------------
%   This is a SIMPLIFIED, BMW-i3-INSPIRED parameter set. It is NOT a set of
%   measured BMW i3 machine parameters and must not be presented as one.
%   Rated torque/power/speed figures are taken from public BMW i3 (60 Ah,
%   IB1 drive) specifications; Rs, Ld, Lq and lambda_m are representative
%   values chosen to be consistent with a machine of that class. The model
%   is linear-magnetic: constant Ld, Lq, lambda_m, no saturation, no
%   cross-coupling, no iron/PM loss.
%
%   See also BUILD_FOC_MODEL, IPMSM_REF_GEN.

%% ---------------- Machine electrical parameters ----------------
P.Rs     = 5.3e-3;      % Stator phase resistance                     [Ohm]
P.Ld     = 0.090e-3;    % d-axis inductance                           [H]
P.Lq     = 0.255e-3;    % q-axis inductance                           [H]
P.lam    = 0.0385;      % PM flux linkage (lambda_m)                  [Wb]
P.p      = 6;           % Pole pairs                                  [-]

%% ---------------- Mechanical parameters ------------------------
P.J      = 0.06;        % Rotor + reflected inertia   [kg m^2]  ASSUMED
P.B      = 0.001;       % Viscous friction coefficient [N m s/rad] ASSUMED

%% ---------------- Inverter / DC link ---------------------------
P.Vdc    = 360;         % DC-link voltage                             [V]
P.Is_max = 430;         % Peak phase current limit                    [A]

% Maximum fundamental peak phase voltage in the LINEAR SVPWM range.
% Space-vector modulation reaches a peak phase voltage of Vdc/sqrt(3)
% before over-modulation begins.
P.Vmax   = P.Vdc/sqrt(3);                    % = 207.846 V

%% ---------------- Nameplate figures (reference only) -----------
% Quoted class figures, used ONLY for sanity-checking the model.
P.Tmax_spec  = 250;     % Peak torque                                [N m]
P.Pmax_spec  = 125e3;   % Peak power                                 [W]
P.n_base_spec= 4800;    % Quoted base speed                          [rpm]
P.n_max      = 11400;   % Maximum speed                              [rpm]

%% ---------------- Numerical guards -----------------------------
P.we_min = 1.0;         % Floor on |we| to keep Vmax/we finite  [rad/s elec]

%% ---------------- Derived quantities ---------------------------
P.dL      = P.Lq - P.Ld;              % Saliency  Lq-Ld > 0          [H]
P.I_ch    = P.lam/P.Ld;               % Characteristic current       [A]
P.wm_max  = 2*pi*P.n_max/60;          % Max mechanical speed     [rad/s]
P.we_max  = P.p*P.wm_max;             % Max electrical speed     [rad/s]

% Voltage-limited base speed at full current, from the CORRECT MTPA point.
[id_b, iq_b] = local_mtpa(P.Is_max, P.Ld, P.Lq, P.lam);
P.id_mtpa_max = id_b;
P.iq_mtpa_max = iq_b;
P.Te_max_model = 1.5*P.p*(P.lam*iq_b + (P.Ld-P.Lq)*id_b*iq_b);
P.we_base = P.Vmax/hypot(P.lam + P.Ld*id_b, P.Lq*iq_b);
P.wm_base = P.we_base/P.p;
P.n_base  = P.wm_base*60/(2*pi);
P.P_base  = P.Te_max_model*P.wm_base;

%% ---------------- Current-loop PI design -----------------------
% Pole-zero cancellation ("internal model") tuning. Plant on each axis is
% first order, L*di/dt = v - Rs*i, so choosing
%       Kp = wc*L ,  Ki = wc*Rs
% cancels the plant pole and gives a closed-loop bandwidth of exactly wc.
P.wc     = 2*pi*500;              % Current-loop bandwidth   [rad/s]  (500 Hz)
P.Kp_d   = P.wc*P.Ld;
P.Kp_q   = P.wc*P.Lq;
P.Ki_d   = P.wc*P.Rs;
P.Ki_q   = P.wc*P.Rs;
P.Kt     = 5*P.Ki_d;              % Anti-windup back-calculation / tracking gain

%% ---------------- Solver settings ------------------------------
P.solver  = 'ode23tb';   % Stiff-capable: we can reach 7000 rad/s elec
P.reltol  = 1e-6;
P.abstol  = 1e-8;
P.maxstep = 1e-4;

end

% -------------------------------------------------------------------------
function [id, iq] = local_mtpa(Is, Ld, Lq, lam)
% MTPA point, from  2*dL*id^2 - lam*id - dL*Is^2 = 0  (see IPMSM_REF_GEN).
dL = Lq - Ld;
id = (lam - sqrt(lam^2 + 8*dL^2*Is^2))/(4*dL);
iq = sqrt(max(Is^2 - id^2, 0));
end
