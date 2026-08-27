function R = run_strategy_comparison(Is_ref, savefigs, doplot)
%RUN_STRATEGY_COMPARISON  Prescribed-speed comparison of the four strategies.
%
%   R = RUN_STRATEGY_COMPARISON(IS_REF, SAVEFIGS, DOPLOT)
%
%   DOPLOT = false suppresses all figures (useful for batch/headless runs and
%   for regression-checking the numbers without a display). Default true.
%
%   Sweeps mechanical speed from standstill to n_max and evaluates the
%   steady-state operating point selected by each of the four strategies on
%   the IDENTICAL plant, IDENTICAL current limit and IDENTICAL voltage limit.
%
%   This is the ANALYTICAL envelope study: speed is an independent variable,
%   which is the only way to compare strategies fairly. The dynamic Simulink
%   confirmation of these curves is RUN_PRESCRIBED_SPEED_SIM.
%
%   Strategies
%     S1  Conventional FOC, id = 0
%     S2  MTPA only (no voltage awareness)
%     S3  Flux weakening only (no MTPA optimisation)
%     S4  Combined MTPA + FW
%
%   Loss model: COPPER LOSS ONLY,  Pcu = 1.5*Rs*|Is|^2  (peak-amplitude
%   convention: |Is| is the peak of the phase current, so the RMS phase
%   current is |Is|/sqrt(2) and Pcu = 3*Rs*(|Is|/sqrt2)^2 = 1.5*Rs*|Is|^2).
%   Iron loss, PM loss, mechanical loss and inverter loss are NOT modelled;
%   the efficiency figures below are therefore an UPPER BOUND and must be
%   reported as "copper-loss-only efficiency".

if nargin < 1 || isempty(Is_ref),  P0 = ipmsm_params(); Is_ref = P0.Is_max; end
if nargin < 2, savefigs = true; end
if nargin < 3, doplot   = true; end

P    = ipmsm_params();
name = {'S1  FOC id=0', 'S2  MTPA only', 'S3  FW only', 'S4  MTPA+FW'};
rpm  = linspace(0, P.n_max, 601);
NS   = 4;  NR = numel(rpm);

[id,iq,Ism,Te,Vm,reg,Pcu,Psh,eta,NmA] = deal(nan(NS,NR));

for s = 1:NS
    for k = 1:NR
        wm = rpm(k)*2*pi/60;  we = P.p*wm;
        [a,b,rg] = ipmsm_strategies(s, Is_ref, we, P);
        id(s,k)=a; iq(s,k)=b; reg(s,k)=rg;
        Ism(s,k) = hypot(a,b);
        Te(s,k)  = 1.5*P.p*(P.lam*b + (P.Ld-P.Lq)*a*b);
        vd = P.Rs*a - we*P.Lq*b;
        vq = P.Rs*b + we*(P.Ld*a + P.lam);
        Vm(s,k)  = hypot(vd,vq);
        Pcu(s,k) = 1.5*P.Rs*Ism(s,k)^2;
        Psh(s,k) = Te(s,k)*wm;
        eta(s,k) = Psh(s,k)/max(Psh(s,k)+Pcu(s,k), eps);
        NmA(s,k) = Te(s,k)/max(Ism(s,k), eps);
    end
end

feasible = reg ~= 4;                 % physically realisable operating points
TeF = Te; TeF(~feasible) = NaN;      % masked copies for plotting
PshF = Psh; PshF(~feasible) = NaN;
VmF  = Vm;  VmF(~feasible) = NaN;

%% ---------------- headline table ----------------
fprintf('\n=================================================================================\n');
fprintf(' FOUR-STRATEGY COMPARISON   Is_ref = %.0f A   (Is_cmd = %.0f A)\n', ...
        Is_ref, min(Is_ref,P.Is_max));
fprintf(' Machine: simplified BMW-i3-INSPIRED parameter set (not measured data)\n');
fprintf('=================================================================================\n');
fprintf('%-15s %9s %8s %10s %12s %10s %10s\n', ...
        'strategy','Te@0 Nm','Nm/A@0','max kW','max rpm','Te@nmax','Pcu@0 kW');
for s = 1:NS
    f = find(feasible(s,:) & Te(s,:) > 1e-6);
    if isempty(f), f = 1; end
    rmax = rpm(f(end));
    tmax = Te(s,end); if ~feasible(s,end), tmax = 0; end
    fprintf('%-15s %9.2f %8.3f %10.2f %12.0f %10.2f %10.2f\n', ...
            name{s}, Te(s,1), NmA(s,1), max(PshF(s,:))/1e3, rmax, tmax, Pcu(s,1)/1e3);
end
fprintf('---------------------------------------------------------------------------------\n');
fprintf(' Base speed (voltage-limited, at full current) : %.0f rpm\n', P.n_base);
fprintf(' Characteristic current lam/Ld                 : %.1f A  (Is_max = %.0f A)\n', ...
        P.I_ch, P.Is_max);
fprintf(' MTPV active below n_max?                      : NO (requires ~100,000 rpm)\n');
fprintf('=================================================================================\n\n');

%% ---------------- plots ----------------
figs = {};                 % cell array: portable across MATLAB and Octave
fignames = {};
if doplot
fignames{end+1}='torque_vs_speed';      figs{end+1} = mkplot(1, rpm, TeF,  name, 'Torque vs speed',            'Torque [N m]');
fignames{end+1}='current_vs_speed';     figs{end+1} = mkplot(2, rpm, Ism,  name, 'Current magnitude vs speed', '|I_s| [A]');
fignames{end+1}='id_vs_speed';          figs{end+1} = mkplot(3, rpm, id,   name, 'd-axis current vs speed',    'i_d [A]');
fignames{end+1}='iq_vs_speed';          figs{end+1} = mkplot(4, rpm, iq,   name, 'q-axis current vs speed',    'i_q [A]');
fignames{end+1}='voltage_vs_speed';     figs{end+1} = mkplot(5, rpm, VmF,  name, 'Voltage magnitude vs speed', '|V| [V]');
yline_safe(P.Vmax, 'V_{max}');
fignames{end+1}='power_vs_speed';       figs{end+1} = mkplot(6, rpm, PshF/1e3, name, 'Shaft power vs speed',   'P_{shaft} [kW]');
fignames{end+1}='copperloss_vs_speed';  figs{end+1} = mkplot(7, rpm, Pcu/1e3,  name, 'Copper loss vs speed',   'P_{cu} [kW]');
fignames{end+1}='torque_per_amp';       figs{end+1} = mkplot(8, rpm, NmA,  name, 'Torque per ampere vs speed', 'T_e/|I_s| [N m/A]');
fignames{end+1}='efficiency_vs_speed';  figs{end+1} = mkplot(9, rpm, 100*eta, name, ...
                     'Copper-loss-only efficiency vs speed', '\eta_{Cu} [%]');

% id-iq trajectory with the constraint geometry
fignames{end+1}='id_iq_trajectories';
figs{end+1} = figure(10); clf; hold on; grid on; box on;
th = linspace(pi/2, pi, 400);
hC = plot(P.Is_max*cos(th), P.Is_max*sin(th), 'k--', 'LineWidth',1.2);
hE = [];
for w = [1 2 3 5 8 11.4]*1000
    we = P.p*w*2*pi/60;
    Psi = max(P.Vmax - P.Rs*min(Is_ref,P.Is_max), 0.05*P.Vmax)/max(we,P.we_min);
    t = linspace(0, pi, 400);
    hh = plot((-P.lam + Psi*cos(t))/P.Ld, Psi*sin(t)/P.Lq, ':', 'Color',[.6 .6 .6]);
    if isempty(hE), hE = hh; end
end
co = lines(NS);
hS = zeros(1,NS);
for s = 1:NS
    m = feasible(s,:);
    hS(s) = plot(id(s,m), iq(s,m), 'LineWidth',1.8, 'Color',co(s,:));
end
xlabel('i_d [A]'); ylabel('i_q [A]');
title('Operating-point trajectories with current circle and voltage ellipses');
% legend explicit handles: 11 lines are drawn but only 6 are labelled
legend([hC hE hS], [{'current limit'},{'voltage ellipses'}, name], 'Location','best');
xlim([-P.Is_max*1.05 20]); ylim([0 P.Is_max*1.05]);
end   % if doplot

if savefigs && doplot
    outdir = fullfile(pwd,'results'); if ~exist(outdir,'dir'), mkdir(outdir); end
    for k = 1:numel(figs)
        try
            saveas(figs{k}, fullfile(outdir, ['cmp_' fignames{k} '.png']));
        catch
        end
    end
    save(fullfile(outdir,'strategy_comparison.mat'), ...
         'rpm','id','iq','Ism','Te','Vm','reg','Pcu','Psh','eta','NmA','name','Is_ref','P');
    fprintf('Figures and data written to %s\n', outdir);
end

R = struct('rpm',rpm,'id',id,'iq',iq,'Is',Ism,'Te',Te,'Vmag',Vm,'region',reg, ...
           'Pcu',Pcu,'Pshaft',Psh,'eta_cu',eta,'NmPerA',NmA,'names',{name},'P',P);
end

% -------------------------------------------------------------------------
function f = mkplot(n, x, Y, name, ttl, yl)
f = figure(n); clf; hold on; grid on; box on;
co = lines(size(Y,1));
for s = 1:size(Y,1)
    plot(x, Y(s,:), 'LineWidth', 1.8, 'Color', co(s,:));
end
xlabel('Mechanical speed [rpm]'); ylabel(yl); title(ttl);
legend(name, 'Location','best');
end

function yline_safe(v, lbl)
% R2015a-compatible horizontal reference line (yline is R2018b).
xl = xlim; hold on;
plot(xl, [v v], 'k--');
text(xl(1) + 0.02*diff(xl), v, lbl, 'VerticalAlignment','bottom');
end
