function D = wltp_preprocess(csvfile, V, P, doplot)
%WLTP_PREPROCESS  Convert the WLTP speed trace into motor speed/torque demand.
%
%   D = WLTP_PREPROCESS(CSVFILE, V, P, DOPLOT)
%
%   Backward-facing (quasi-static) vehicle model: the cycle prescribes vehicle
%   speed, from which the required wheel force, and hence the required motor
%   torque and motor speed, are computed second by second.
%
%   ROAD-LOAD EQUATION
%     F_road = m*a*k_rot + Crr*m*g*cos(th) + 0.5*rho*Cd*Af*v^2 + m*g*sin(th)
%     T_wheel = F_road * r_wheel
%     w_motor = (v / r_wheel) * G
%     T_motor = T_wheel / (G*eta_dt)     when tractive  (F_road > 0)
%             = T_wheel * eta_dt / G     when braking   (F_road < 0)
%
%   The efficiency term inverts on braking because the loss is then taken out
%   of the energy flowing back from the wheels to the motor.
%
%   ALL vehicle parameters are ASSUMPTIONS - see WLTP_VEHICLE_PARAMS.
%
%   Returns struct D with fields t, v_ms, a_ms2, F_road, T_wheel, wm, rpm,
%   T_motor, phase, dist_km.

if nargin < 1 || isempty(csvfile), csvfile = 'WLTP.csv'; end
if nargin < 2 || isempty(V), V = wltp_vehicle_params(); end
if nargin < 3 || isempty(P), P = ipmsm_params(); end
if nargin < 4, doplot = true; end

%% ---------------- read the cycle ----------------
% Kept deliberately low-level (fopen/textscan) so it works on R2015a without
% readtable/detectImportOptions and without the Statistics toolbox.
% fopen() does NOT search the MATLAB path, so resolve the file first.
if exist(csvfile,'file') ~= 2
    error('wltp:noFile','Cannot find %s.', csvfile);
end
resolved = which(csvfile);
if isempty(resolved), resolved = csvfile; end
fid = fopen(resolved,'r');
if fid < 0
    error('wltp:noFile', ['Cannot open %s. Copy WLTP.csv into this folder ' ...
          '(it ships one level up in "Major Project").'], csvfile);
end
hdr = fgetl(fid);                                     %#ok<NASGU>  header line
Craw = textscan(fid, '%f%f%s', 'Delimiter', ',');
fclose(fid);

t     = Craw{1}(:);
v_kmh = Craw{2}(:);
phase = Craw{3}(:);

if numel(t) < 2, error('wltp:tooShort','Cycle file has too few rows.'); end

%% ---------------- kinematics ----------------
v  = v_kmh/3.6;                        % m/s
dt = [diff(t); t(end)-t(end-1)];
a  = [diff(v)./diff(t); 0];            % m/s^2, forward difference

%% ---------------- road load ----------------
F_in   = V.m * a * V.k_rot;                                  % inertia
F_roll = V.Crr * V.m * V.g * cos(V.grade) * (v > 0.01);      % no rolling at rest
F_aero = 0.5 * V.rho_air * V.Cd * V.Af * v.^2;               % drag
F_grad = V.m * V.g * sin(V.grade);                           % zero on WLTP
F_road = F_in + F_roll + F_aero + F_grad;

T_wheel = F_road * V.r_wheel;
wm      = (v / V.r_wheel) * V.G;                             % rad/s mechanical
rpm     = wm * 60/(2*pi);

T_motor = zeros(size(T_wheel));
trac    = T_wheel >= 0;
T_motor(trac)  = T_wheel(trac) / (V.G * V.eta_dt);
T_motor(~trac) = T_wheel(~trac) * V.eta_dt / V.G;

dist_km = sum(v .* dt)/1000;

%% ---------------- feasibility against the machine ----------------
% Maximum torque the machine can produce at each cycle speed, using the full
% MTPA+FW strategy at the current limit.
T_cap = zeros(size(wm));
for k = 1:numel(wm)
    [a_,b_] = ipmsm_strategies(4, P.Is_max, P.p*wm(k), P);
    T_cap(k) = 1.5*P.p*(P.lam*b_ + (P.Ld-P.Lq)*a_*b_);
end
over_T = sum(abs(T_motor) > T_cap + 1e-9);
over_w = sum(rpm > P.n_max + 1e-9);

fprintf('--- WLTP preprocessing ---\n');
fprintf('  samples             : %d   duration %.0f s   distance %.3f km\n', ...
        numel(t), t(end), dist_km);
fprintf('  vehicle speed       : max %.1f km/h   mean %.2f km/h\n', ...
        max(v_kmh), mean(v_kmh));
fprintf('  motor speed         : max %.0f rpm   (n_max %.0f rpm)\n', max(rpm), P.n_max);
fprintf('  motor torque demand : max %.1f Nm  min %.1f Nm  (capability %.1f Nm)\n', ...
        max(T_motor), min(T_motor), P.Te_max_model);
fprintf('  samples over torque capability : %d\n', over_T);
fprintf('  samples over speed  limit      : %d\n', over_w);
if over_T == 0 && over_w == 0
    fprintf('  -> the whole cycle is inside the machine envelope.\n');
else
    fprintf('  -> cycle exceeds the envelope at some points; shortfall is reported.\n');
end

if doplot
    figure(30); clf;
    subplot(3,1,1); plot(t, v_kmh, 'LineWidth',1.1); grid on;
    ylabel('v [km/h]'); title('WLTP Class 3b cycle and derived motor demand');
    subplot(3,1,2); plot(t, rpm, 'LineWidth',1.1); grid on; hold on;
    plot([t(1) t(end)], [P.n_max P.n_max], 'k--');
    ylabel('motor [rpm]'); legend('demand','n_{max}');
    subplot(3,1,3); plot(t, T_motor, 'LineWidth',1.1); grid on; hold on;
    plot(t, T_cap, 'r--'); plot(t, -T_cap, 'r--');
    ylabel('T_{motor} [N m]'); xlabel('time [s]'); legend('demand','capability');
end

D = struct('t',t,'v_kmh',v_kmh,'v_ms',v,'a_ms2',a,'dt',dt,'F_road',F_road, ...
           'T_wheel',T_wheel,'wm',wm,'rpm',rpm,'T_motor',T_motor, ...
           'T_cap',T_cap,'phase',{phase},'dist_km',dist_km,'V',V);
end
