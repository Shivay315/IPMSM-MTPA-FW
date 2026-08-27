function OUT = run_prescribed_speed_sim(whichTests)
%RUN_PRESCRIBED_SPEED_SIM  Dynamic Simulink validation, Tests A-D.
%
%   OUT = RUN_PRESCRIBED_SPEED_SIM              runs A, B, C and D
%   OUT = RUN_PRESCRIBED_SPEED_SIM('AC')        runs only A and C
%
%   Speed is PRESCRIBED (an independent input), not integrated. That is the
%   correct rig for validating a current-control strategy: it isolates the
%   controller from the unbounded free-acceleration behaviour that made the
%   original model impossible to interpret.
%
%   TEST A  Low-speed MTPA          1000 rpm, step current demand
%   TEST B  Transition through base speed, slow ramp 2000 -> 6000 rpm
%   TEST C  High-speed field weakening, 10000 rpm
%   TEST D  Full-range sweep 0 -> n_max, compared against the analytical
%           envelope produced by RUN_STRATEGY_COMPARISON
%
%   Each test is judged NUMERICALLY, not visually:
%     - steady-state |id - id*| and |iq - iq*| below tolerance
%     - |V| <= Vmax at all times
%     - |Is| <= Is_max at all times
%     - Te consistent with the torque equation evaluated on the measured
%       currents
%
%   Log column order (set by BUILD_FOC_MODEL):
%     1 t   2 wm  3 we  4 Is_ref  5 id*  6 iq*  7 region  8 id  9 iq
%    10 Is_mag  11 Te  12 vd*  13 vq*  14 vd  15 vq  16 Vmag  17 m  18 sat

if nargin < 1, whichTests = 'ABCD'; end
P   = ipmsm_params();
mdl = 'foc_prescribed_v2';
if ~exist([mdl '.slx'],'file'), build_foc_model('prescribed', mdl); end
load_system(mdl);

C = struct('t',1,'wm',2,'we',3,'Isref',4,'ids',5,'iqs',6,'reg',7,'id',8, ...
           'iq',9,'Ism',10,'Te',11,'vds',12,'vqs',13,'vd',14,'vq',15, ...
           'Vmag',16,'m',17,'sat',18);
OUT = struct(); allpass = true;

%% ---------------- TEST A : low-speed MTPA ----------------
if any(whichTests == 'A')
    T = 0.3;  rpm = 1000;
    wm_ts    = [0 rpm*2*pi/60; T rpm*2*pi/60];                 %#ok<NASGU>
    Is_ref_ts= [0 0; 0.0199 0; 0.02 P.Is_max; T P.Is_max];     %#ok<NASGU>
    L = runsim(mdl, T, wm_ts, Is_ref_ts);
    [ok, rep] = judge(L, C, P, 'A  low-speed MTPA (1000 rpm)', 0.25*T);
    % additionally confirm we really are on the MTPA trajectory
    ss = L(end,:);
    id_m = (P.lam - sqrt(P.lam^2 + 8*P.dL^2*P.Is_max^2))/(4*P.dL);
    iq_m = sqrt(P.Is_max^2 - id_m^2);
    dM = hypot(ss(C.id)-id_m, ss(C.iq)-iq_m);
    fprintf('    on MTPA trajectory      : |i - i_MTPA| = %.3f A  -> %s\n', ...
            dM, passfail(dM < 1.0));
    fprintf('    voltage saturation used : %.1f%% of samples\n', 100*mean(L(:,C.sat)));
    ok = ok && dM < 1.0;  allpass = allpass && ok;
    OUT.A = struct('log',L,'pass',ok,'report',{rep});
end

%% ---------------- TEST B : transition through base speed ----------------
if any(whichTests == 'B')
    T = 1.0;
    wm_ts     = [0 2000*2*pi/60; T 6000*2*pi/60];              %#ok<NASGU>
    Is_ref_ts = [0 P.Is_max; T P.Is_max];                      %#ok<NASGU>
    L = runsim(mdl, T, wm_ts, Is_ref_ts);
    [ok, rep] = judge(L, C, P, 'B  MTPA -> FW transition (2000-6000 rpm)', 0.05*T);
    r = L(:,C.reg);
    fprintf('    regions visited         : %s\n', mat2str(unique(round(r))'));
    ntr = sum(abs(diff(round(r))) > 0);
    fprintf('    region changes          : %d (expect 1: MTPA->FW)\n', ntr);
    % transition must be continuous: no jump in the reference
    jmp = max(abs(diff(L(:,C.ids))));
    fprintf('    max step in id*         : %.3f A per solver step -> %s\n', ...
            jmp, passfail(jmp < 25));
    ok = ok && jmp < 25;  allpass = allpass && ok;
    OUT.B = struct('log',L,'pass',ok,'report',{rep});
end

%% ---------------- TEST C : high-speed FW ----------------
if any(whichTests == 'C')
    T = 0.3;  rpm = 10000;
    wm_ts     = [0 rpm*2*pi/60; T rpm*2*pi/60];                %#ok<NASGU>
    Is_ref_ts = [0 P.Is_max; T P.Is_max];                      %#ok<NASGU>
    L = runsim(mdl, T, wm_ts, Is_ref_ts);
    [ok, rep] = judge(L, C, P, 'C  high-speed field weakening (10000 rpm)', 0.3*T);
    ss = L(end,:);
    fprintf('    id* = %.1f A (must be strongly negative)   -> %s\n', ...
            ss(C.ids), passfail(ss(C.ids) < -300));
    fprintf('    iq* = %.1f A (reduced from MTPA value)     -> %s\n', ...
            ss(C.iqs), passfail(ss(C.iqs) < 200));
    fprintf('    region = %d (expect 2 = FW Region I)       -> %s\n', ...
            round(ss(C.reg)), passfail(round(ss(C.reg)) == 2));
    ok = ok && ss(C.ids) < -300 && round(ss(C.reg)) == 2;
    allpass = allpass && ok;
    OUT.C = struct('log',L,'pass',ok,'report',{rep});
end

%% ---------------- TEST D : full-range sweep ----------------
if any(whichTests == 'D')
    T = 4.0;
    wm_ts     = [0 0; T P.wm_max];                             %#ok<NASGU>
    Is_ref_ts = [0 P.Is_max; T P.Is_max];                      %#ok<NASGU>
    L = runsim(mdl, T, wm_ts, Is_ref_ts);
    [ok, rep] = judge(L, C, P, 'D  full-range sweep (0 - n_max)', 0.02*T);

    % compare the dynamic result against the analytical envelope
    rpmL = L(:,C.wm)*60/(2*pi);
    TeA  = nan(size(rpmL));
    for k = 1:numel(rpmL)
        [a,b] = ipmsm_strategies(4, P.Is_max, P.p*L(k,C.wm), P);
        TeA(k) = 1.5*P.p*(P.lam*b + (P.Ld-P.Lq)*a*b);
    end
    sel = rpmL > 200;                       % skip the initial transient
    eT  = max(abs(L(sel,C.Te) - TeA(sel)));
    fprintf('    max |Te_sim - Te_analytical| = %.3f Nm -> %s\n', ...
            eT, passfail(eT < 5));
    ok = ok && eT < 5;  allpass = allpass && ok;

    figure(20); clf;
    subplot(3,1,1); plot(rpmL, L(:,C.Te), 'LineWidth',1.5); hold on;
    plot(rpmL, TeA, 'r--','LineWidth',1.2); grid on;
    ylabel('T_e [N m]'); legend('Simulink','analytical'); title('Test D: full-range sweep');
    subplot(3,1,2); plot(rpmL, L(:,[C.id C.ids C.iq C.iqs]), 'LineWidth',1.2); grid on;
    ylabel('current [A]'); legend('i_d','i_d^*','i_q','i_q^*');
    subplot(3,1,3); plot(rpmL, L(:,C.Vmag), 'LineWidth',1.5); grid on; hold on;
    plot(rpmL([1 end]), [P.Vmax P.Vmax],'k--');
    ylabel('|V| [V]'); xlabel('speed [rpm]'); legend('|V|','V_{max}');

    OUT.D = struct('log',L,'pass',ok,'report',{rep},'Te_analytical',TeA);
end

fprintf('\n================================================================\n');
fprintf(' PRESCRIBED-SPEED VALIDATION OVERALL: %s\n', passfail(allpass));
fprintf('================================================================\n\n');
OUT.pass = allpass;
end

% =========================================================================
function L = runsim(mdl, T, wm_ts, Is_ref_ts)
assignin('base','wm_ts',      wm_ts);
assignin('base','Is_ref_ts',  Is_ref_ts);
set_param(mdl,'StopTime',num2str(T));

% Depending on release, sim() either returns a SimulationOutput carrying the
% To Workspace variable or leaves it in the base workspace. Handle both.
L = [];
try
    so = sim(mdl);
    if isa(so,'Simulink.SimulationOutput')
        try, L = so.get('simlog'); catch, L = []; end
    end
catch ME
    if ~strcmp(ME.identifier,'MATLAB:TooManyOutputs'), rethrow(ME); end
    sim(mdl);
end
if isempty(L) && exist('simlog','var')
    L = simlog;
end
if isempty(L)
    try
        L = evalin('base','simlog');
    catch
    end
end
if isempty(L)
    error('runsim:noLog','Simulation produced no ''simlog'' data.');
end
end

% =========================================================================
function [ok, rep] = judge(L, C, P, ttl, t_settle)
fprintf('\n---- TEST %s ----\n', ttl);
ss   = L(:,C.t) >= t_settle;
eid  = max(abs(L(ss,C.id) - L(ss,C.ids)));
eiq  = max(abs(L(ss,C.iq) - L(ss,C.iqs)));
vmax = max(sqrt(L(:,C.vd).^2 + L(:,C.vq).^2));
imax = max(L(:,C.Ism));
TeChk= max(abs(L(:,C.Te) - 1.5*P.p*(P.lam*L(:,C.iq) + (P.Ld-P.Lq).*L(:,C.id).*L(:,C.iq))));

c1 = eid  < 2.0;
c2 = eiq  < 2.0;
c3 = vmax <= P.Vmax*1.001;
c4 = imax <= P.Is_max*1.001;
c5 = TeChk < 1e-6;

fprintf('    max |id - id*| (steady)  : %8.3f A     -> %s\n', eid,  passfail(c1));
fprintf('    max |iq - iq*| (steady)  : %8.3f A     -> %s\n', eiq,  passfail(c2));
fprintf('    max |V|                  : %8.3f V     (Vmax = %.3f) -> %s\n', ...
        vmax, P.Vmax, passfail(c3));
fprintf('    max |Is|                 : %8.3f A     (Imax = %.0f)  -> %s\n', ...
        imax, P.Is_max, passfail(c4));
fprintf('    torque-equation residual : %8.2e Nm    -> %s\n', TeChk, passfail(c5));
ok  = c1 && c2 && c3 && c4 && c5;
rep = struct('eid',eid,'eiq',eiq,'vmax',vmax,'imax',imax,'Teres',TeChk,'pass',ok);
fprintf('    TEST RESULT              : %s\n', passfail(ok));
end

function s = passfail(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end
