function W = run_wltp_study(doplot)
%RUN_WLTP_STUDY  Compare the four strategies over the WLTP Class 3b cycle.
%
%   W = RUN_WLTP_STUDY(DOPLOT)  runs the quasi-static (backward-facing)
%   drive-cycle study and prints the energy / loss comparison table.
%
%   METHOD
%   At every cycle second the required motor torque and speed are known from
%   WLTP_PREPROCESS. For each strategy we find the SMALLEST current demand
%   Is_ref that delivers the required torque at that speed:
%
%       Is_ref* = min { Is : Te(strategy, Is, we) >= |T_demand| }
%
%   found by a coarse scan plus local refinement (no monotonicity assumed).
%   A candidate is admissible only if it is PHYSICALLY REALISABLE, i.e. the
%   strategy did not return region 4. Without that test, "MTPA only" would be
%   credited above base speed with torque it cannot produce inside the
%   voltage limit and would wrongly appear to be the most efficient strategy.
%
%   If no admissible current delivers the torque, the shortfall is recorded.
%   That is how "tracking error" appears in a quasi-static study.
%
%   Negative (regenerative) torque is handled by symmetry: replacing iq by
%   -iq negates Te while leaving |Is|, and therefore copper loss, unchanged.
%
%   ENERGY ACCOUNTING (peak-amplitude convention)
%       P_elec = 1.5*(vd*id + vq*iq)        terminal electrical power
%       P_mech = Te*wm                      shaft power
%       P_cu   = 1.5*Rs*|Is|^2              copper loss
%   In steady state P_elec = P_mech + P_cu; this identity is checked and
%   reported as a validation of the energy bookkeeping.
%
%   LIMITATION: copper loss only. Iron loss, PM loss, mechanical drag and
%   inverter loss are NOT modelled, so consumption figures are optimistic
%   and must be labelled "copper-loss-only" in the report.

if nargin < 1, doplot = true; end

P = ipmsm_params();
V = wltp_vehicle_params();
D = wltp_preprocess('WLTP.csv', V, P, false);

names = {'S1  FOC id=0','S2  MTPA only','S3  FW only','S4  MTPA+FW'};
NS = 4;  N = numel(D.t);

Is_scan = 0:5:P.Is_max;
Te_a  = zeros(NS,N);  Is_a  = zeros(NS,N);
Pcu_a = zeros(NS,N);  Pel_a = zeros(NS,N);  Pme_a = zeros(NS,N);
short = zeros(NS,N);

for s = 1:NS
    for k = 1:N
        we    = P.p*D.wm(k);
        T_req = abs(D.T_motor(k));
        [Te_use, id_u, iq_u] = solve_current(s, T_req, we, P, Is_scan);

        sgn = 1;
        if D.T_motor(k) < 0, sgn = -1; end
        iq_u = sgn*iq_u;  Te_use = sgn*Te_use;

        vd = P.Rs*id_u - we*P.Lq*iq_u;
        vq = P.Rs*iq_u + we*(P.Ld*id_u + P.lam);

        Te_a(s,k)  = Te_use;
        Is_a(s,k)  = hypot(id_u, iq_u);
        Pcu_a(s,k) = 1.5*P.Rs*Is_a(s,k)^2;
        Pel_a(s,k) = 1.5*(vd*id_u + vq*iq_u);
        Pme_a(s,k) = Te_use*D.wm(k);
        short(s,k) = max(T_req - abs(Te_use), 0);
    end
end

%% ---------------- energy integration ----------------
dt   = D.dt(:)';
DT   = repmat(dt, NS, 1);
E_el = sum(Pel_a .* DT, 2) / 3.6e6;
E_cu = sum(Pcu_a .* DT, 2) / 3.6e6;
E_me = sum(Pme_a .* DT, 2) / 3.6e6;

bal        = max(abs(Pel_a - (Pme_a + Pcu_a)), [], 2);
rms_short  = sqrt(mean(short.^2, 2));
max_short  = max(short, [], 2);
unmet      = sum(short > 1e-6, 2);

%% ---------------- report ----------------
line1 = '===============================================================================';
line2 = '-------------------------------------------------------------------------------';
fprintf('\n%s\n', line1);
fprintf(' WLTP CLASS 3b DRIVE-CYCLE COMPARISON   (%.3f km, %.0f s)\n', D.dist_km, D.t(end));
fprintf(' Quasi-static backward-facing vehicle model. Vehicle parameters ASSUMED.\n');
fprintf(' Losses: COPPER ONLY - figures are an upper bound on efficiency.\n');
fprintf('%s\n', line1);
fprintf('%-15s %11s %10s %8s %10s %8s %9s\n', 'strategy', 'E_mech kWh', ...
        'E_net kWh', 'Wh/km', 'E_cu kWh', 'unmet s', 'complete');
for s = 1:NS
    if unmet(s) == 0, cflag = 'YES'; else, cflag = 'NO'; end
    fprintf('%-15s %11.4f %10.4f %8.2f %10.4f %8d %9s\n', names{s}, ...
            E_me(s), E_el(s), 1000*E_el(s)/D.dist_km, E_cu(s), unmet(s), cflag);
end
fprintf('\n  A strategy with unmet > 0 does NOT complete the cycle. Its energy\n');
fprintf('  figures are then NOT directly comparable, because it performs less\n');
fprintf('  mechanical work - compare E_mech before comparing E_net or E_cu.\n');
fprintf('%s\n', line2);
fprintf('%-15s %11s %11s %14s\n', 'strategy', 'max|dT| Nm', 'rms dT Nm', 'E-balance W');
for s = 1:NS
    fprintf('%-15s %11.3f %11.4f %14.2e\n', names{s}, ...
            max_short(s), rms_short(s), bal(s));
end
fprintf('%s\n', line2);
ref = 4;
for s = 1:NS
    if s == ref, continue; end
    if unmet(s) > 0
        tag = '   (INCOMPLETE - not comparable)';
    else
        tag = '';
    end
    fprintf('  %s vs %s : copper %+.1f%%, work %+.1f%%%s\n', names{s}, names{ref}, ...
            100*(E_cu(s)-E_cu(ref))/E_cu(ref), ...
            100*(E_me(s)-E_me(ref))/E_me(ref), tag);
end
fprintf('%s\n\n', line1);

%% ---------------- plots ----------------
if doplot
    figure(31); clf;
    subplot(3,1,1); plot(D.t, D.v_kmh, 'LineWidth', 1.1); grid on;
    ylabel('v [km/h]'); title('WLTP: strategy comparison');
    subplot(3,1,2); plot(D.t, Is_a', 'LineWidth', 1.0); grid on;
    ylabel('|I_s| [A]'); legend(names, 'Location', 'best');
    subplot(3,1,3); plot(D.t, Pcu_a'/1e3, 'LineWidth', 1.0); grid on;
    ylabel('P_{cu} [kW]'); xlabel('time [s]'); legend(names, 'Location', 'best');

    figure(32); clf;
    subplot(2,1,1); bar(E_cu); grid on; ylabel('cycle copper loss [kWh]');
    set(gca, 'XTickLabel', names); title('WLTP energy comparison');
    subplot(2,1,2); bar(1000*E_el/D.dist_km); grid on; ylabel('consumption [Wh/km]');
    set(gca, 'XTickLabel', names);
end

outdir = fullfile(pwd, 'results');
if ~exist(outdir, 'dir'), mkdir(outdir); end
W = struct('names', {names}, 'E_elec_kWh', E_el, 'E_copper_kWh', E_cu, ...
           'E_mech_kWh', E_me, 'Wh_per_km', 1000*E_el/D.dist_km, ...
           'rms_shortfall', rms_short, 'max_shortfall', max_short, ...
           'unmet_samples', unmet, 'energy_balance_W', bal);
save(fullfile(outdir, 'wltp_study.mat'), '-struct', 'W');
fprintf('Saved %s\n', fullfile(outdir, 'wltp_study.mat'));
end

% =========================================================================
function [Te_use, id_u, iq_u] = solve_current(s, T_req, we, P, Is_scan)
%SOLVE_CURRENT  Smallest ADMISSIBLE current demand meeting T_req.
best_Is = P.Is_max;
found   = false;
for j = 1:numel(Is_scan)
    [a_, b_, rg_] = ipmsm_strategies(s, Is_scan(j), we, P);
    if rg_ ~= 4 && 1.5*P.p*(P.lam*b_ + (P.Ld-P.Lq)*a_*b_) >= T_req
        best_Is = Is_scan(j);  found = true;  break
    end
end
if found && best_Is > 0
    grid_lo = max(best_Is - 5, 0);
    refine  = linspace(grid_lo, best_Is, 21);
    for j = 1:numel(refine)
        [a_, b_, rg_] = ipmsm_strategies(s, refine(j), we, P);
        if rg_ ~= 4 && 1.5*P.p*(P.lam*b_ + (P.Ld-P.Lq)*a_*b_) >= T_req
            best_Is = refine(j);  break
        end
    end
end
[id_u, iq_u, rg_] = ipmsm_strategies(s, best_Is, we, P);
if rg_ == 4
    id_u = 0;  iq_u = 0;            % demand unrealisable at this speed
end
Te_use = 1.5*P.p*(P.lam*iq_u + (P.Ld-P.Lq)*id_u*iq_u);
end
