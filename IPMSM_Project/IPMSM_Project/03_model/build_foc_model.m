function mdl = build_foc_model(mode, mdl)
%BUILD_FOC_MODEL  Programmatically construct the corrected FOC/MTPA/FW model.
%
%   MDL = BUILD_FOC_MODEL(MODE)  builds and saves a Simulink model.
%     MODE = 'prescribed'  speed wm is an INPUT (From Workspace 'wm_ts').
%                          Used for the torque-speed envelope / strategy
%                          comparison, where speed must be an independent
%                          variable.
%     MODE = 'dynamic'     speed comes from the mechanical integrator driven
%                          by (Te - TL). Used for WLTP / closed-loop work.
%
%   The model is GENERATED, never hand-edited. Every machine parameter is
%   injected here from IPMSM_PARAMS, so the model can never drift out of
%   sync with the parameter file. Re-run this function after any parameter
%   change.
%
%   DESIGN NOTES (see report):
%
%   1. PI controllers are built from primitive blocks (Gain / Integrator /
%      Sum) rather than the library PID Controller block. This removes any
%      toolbox or release dependency and makes the anti-windup path explicit.
%      Each axis implements
%           u     = Kp*e + I
%           dI/dt = Ki*e + Kaw*(TR - u)
%      where TR is the ACTUALLY APPLIED PI voltage (v_applied - v_comp).
%      Back-calculation against the applied voltage is what prevents windup
%      when the SVPWM limiter is active.
%
%   2. The voltage limiter acts directly in the dq frame. Limiting in dq is
%      mathematically identical to limiting in alpha-beta, because the
%      inverse Park transform is a pure rotation and rotations preserve
%      vector magnitude:  |[valpha;vbeta]| = |[vd;vq]|. The original model
%      performed InvPark -> limit -> Park, which is an identity operation
%      around the same clip. Removing it eliminates two blocks and the
%      spurious theta dependence without changing any result.
%
%   3. There is exactly ONE source of id* and iq*: the RefGen block. The
%      Cart2Polar/RateLimiter/Polar2Cart/iq_star_calc algebraic loop of the
%      original model is gone.
%
%   See also IPMSM_PARAMS, IPMSM_REF_GEN, RUN_STRATEGY_COMPARISON.

if nargin < 1 || isempty(mode), mode = 'prescribed'; end
mode = validatestring(mode, {'prescribed','dynamic'});
if nargin < 2 || isempty(mdl)
    mdl = ['foc_' mode '_v2'];
end

P = ipmsm_params();

%% ---------------- create a clean model ----------------
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);

set_param(mdl, 'Solver',     P.solver, ...
               'SolverType', 'Variable-step', ...
               'RelTol',     num2str(P.reltol), ...
               'AbsTol',     num2str(P.abstol), ...
               'MaxStep',    num2str(P.maxstep), ...
               'StartTime',  '0', ...
               'StopTime',   '1.0', ...
               'SaveOutput', 'off', ...
               'SaveTime',   'off');

% Local helper functions are used rather than anonymous handles: an anonymous
% function body must be an expression, and wrapping a void call to add_block in
% one is fragile across releases.

%% ---------------- sources ----------------
add_block('simulink/Sources/From Workspace', [mdl '/IsRefSrc'], ...
          'Position', [30 60 100 90]);
set_param([mdl '/IsRefSrc'], 'VariableName','Is_ref_ts', 'SampleTime','0', ...
          'Interpolate','on', 'OutputAfterFinalValue','Holding final value');

switch mode
    case 'prescribed'
        add_block('simulink/Sources/From Workspace', [mdl '/SpeedSrc'], ...
                  'Position', [30 380 100 410]);
        set_param([mdl '/SpeedSrc'], 'VariableName','wm_ts', 'SampleTime','0', ...
                  'Interpolate','on', 'OutputAfterFinalValue','Holding final value');
    case 'dynamic'
        add_block('simulink/Sources/From Workspace', [mdl '/TLSrc'], ...
                  'Position', [700 700 770 730]);
        set_param([mdl '/TLSrc'], 'VariableName','TL_ts', 'SampleTime','0', ...
                  'Interpolate','on', 'OutputAfterFinalValue','Holding final value');
end

%% ---------------- MATLAB Function blocks ----------------
% Parameters are injected as literals from P: one source of truth.
n = @(x) sprintf('%.10g', x);
NL = sprintf('\n');

fcn_refgen = strjoin({ ...
 'function [id_star, iq_star, region] = RefGen(Is_ref, we)', ...
 '%#codegen', ...
 '% Single authoritative source of the current references.', ...
 '% Delegates to the validated IPMSM_REF_GEN; machine parameters are', ...
 '% injected by BUILD_FOC_MODEL from IPMSM_PARAMS.', ...
 sprintf('[id_star, iq_star, region] = ipmsm_ref_gen(Is_ref, we, %s, %s, %s, %s, %s, %s, %s);', ...
         n(P.Rs), n(P.Ld), n(P.Lq), n(P.lam), n(P.Vmax), n(P.Is_max), n(P.we_min)), ...
 'end'}, NL);

fcn_decouple = strjoin({ ...
 'function [vd_comp, vq_comp] = Decouple(id, iq, we)', ...
 '%#codegen', ...
 '% Feed-forward cancellation of the speed-dependent cross-coupling terms', ...
 '%   vd = Rs*id + Ld*did/dt - we*Lq*iq', ...
 '%   vq = Rs*iq + Lq*diq/dt + we*(Ld*id + lam)', ...
 '% so that each axis presents a first-order plant to its PI controller.', ...
 sprintf('vd_comp = -we*%s*iq;', n(P.Lq)), ...
 sprintf('vq_comp =  we*(%s*id + %s);', n(P.Ld), n(P.lam)), ...
 'end'}, NL);

fcn_limit = strjoin({ ...
 'function [vd, vq, Vmag, m, sat_flag] = VoltLimit(vd_star, vq_star)', ...
 '%#codegen', ...
 '% SVPWM linear-range limit, applied in dq. The inverse Park transform is', ...
 '% a rotation, so |[valpha;vbeta]| = |[vd;vq]| and clipping here is', ...
 '% identical to clipping in the stationary frame.', ...
 '% The direction of the voltage vector is preserved (magnitude scaling),', ...
 '% which is the standard minimum-distortion choice.', ...
 sprintf('Vmax = %s;', n(P.Vmax)), ...
 'Vmag = sqrt(vd_star^2 + vq_star^2);', ...
 'm    = Vmag/Vmax;', ...
 'if Vmag <= Vmax', ...
 '    vd = vd_star; vq = vq_star; sat_flag = 0;', ...
 'else', ...
 '    s = Vmax/Vmag; vd = vd_star*s; vq = vq_star*s; sat_flag = 1;', ...
 'end', ...
 'end'}, NL);

fcn_motor = strjoin({ ...
 'function [did_dt, diq_dt] = MotorElec(vd, vq, we, id, iq)', ...
 '%#codegen', ...
 '% IPMSM dq electrical dynamics (magnetically linear).', ...
 sprintf('did_dt = (vd - %s*id + we*%s*iq) / %s;', n(P.Rs), n(P.Lq), n(P.Ld)), ...
 sprintf('diq_dt = (vq - %s*iq - we*(%s*id + %s)) / %s;', ...
         n(P.Rs), n(P.Ld), n(P.lam), n(P.Lq)), ...
 'end'}, NL);

fcn_torque = strjoin({ ...
 'function [Te, Is_mag] = TorqueCalc(id, iq)', ...
 '%#codegen', ...
 '% Te = 1.5*p*[ lam*iq + (Ld-Lq)*id*iq ]   (PM term + reluctance term)', ...
 sprintf('Te     = 1.5*%s*(%s*iq + (%s - %s)*id*iq);', ...
         n(P.p), n(P.lam), n(P.Ld), n(P.Lq)), ...
 'Is_mag = sqrt(id^2 + iq^2);', ...
 'end'}, NL);

fcn_mech = strjoin({ ...
 'function dwm_dt = MechDyn(Te, TL, wm)', ...
 '%#codegen', ...
 sprintf('dwm_dt = (Te - TL - %s*wm) / %s;', n(P.B), n(P.J)), ...
 'end'}, NL);

mkfcn(mdl, 'RefGen',     fcn_refgen,   [150   40  240  110]);
mkfcn(mdl, 'Decouple',   fcn_decouple, [640  260  730  330]);
mkfcn(mdl, 'VoltLimit',  fcn_limit,    [880  150  980  240]);
mkfcn(mdl, 'MotorElec',  fcn_motor,    [1030 380 1130  470]);
mkfcn(mdl, 'TorqueCalc', fcn_torque,   [1230 520 1330  580]);
if strcmp(mode,'dynamic')
    mkfcn(mdl, 'MechDyn', fcn_mech,    [800  660  890  720]);
end

%% ---------------- speed path ----------------
addGain(mdl,'we_calc', n(P.p), [200 380 240 410]);

%% ---------------- current error ----------------
addSum(mdl,'Sum_ed','|+-',[330  55 350  75]);
addSum(mdl,'Sum_eq','|+-',[330 115 350 135]);

%% ---------------- d-axis PI (primitive, with back-calculation) ----------
addGain(mdl,'Kp_d',  n(P.Kp_d), [400  40 440  70]);
addGain(mdl,'Ki_d',  n(P.Ki_d), [400  90 440 120]);
addGain(mdl,'Kaw_d', n(P.Kt),   [400 140 440 170]);
addSum(mdl,'Sum_Id','|++',[490  95 510 115]);
addInt(mdl,'Int_d','0',   [540  90 570 120]);
addSum(mdl,'Sum_ud','|++',[620  45 640  65]);
addSum(mdl,'Sum_awd','|+-',[560 145 580 165]);

%% ---------------- q-axis PI ----------------
addGain(mdl,'Kp_q',  n(P.Kp_q), [400 200 440 230]);
addGain(mdl,'Ki_q',  n(P.Ki_q), [400 250 440 280]);
addGain(mdl,'Kaw_q', n(P.Kt),   [400 300 440 330]);
addSum(mdl,'Sum_Iq','|++',[490 255 510 275]);
addInt(mdl,'Int_q','0',   [540 250 570 280]);
addSum(mdl,'Sum_uq','|++',[620 205 640 225]);
addSum(mdl,'Sum_awq','|+-',[560 305 580 325]);

%% ---------------- voltage assembly and limiting ----------------
addSum(mdl,'Sum_vd','|++',[790 155 810 175]);
addSum(mdl,'Sum_vq','|++',[790 215 810 235]);
addSum(mdl,'Sum_trd','|+-',[790 300 810 320]);   % TR_d = vd - vd_comp
addSum(mdl,'Sum_trq','|+-',[790 350 810 370]);   % TR_q = vq - vq_comp

%% ---------------- state integrators ----------------
addInt(mdl,'Int_id','0',[1180 380 1210 410]);
addInt(mdl,'Int_iq','0',[1180 430 1210 460]);
if strcmp(mode,'dynamic')
    addInt(mdl,'Int_wm','0',[930 660 960 690]);
end

%% ---------------- logging ----------------
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [1380 640 1410 670]);
add_block('simulink/Signal Routing/Mux', [mdl '/LogMux'], ...
          'Inputs','18', 'Position',[1460 60 1470 700]);
add_block('simulink/Sinks/To Workspace', [mdl '/SimLog'], 'Position',[1520 360 1590 400]);
set_param([mdl '/SimLog'], 'VariableName','simlog', 'SaveFormat','Array', 'SampleTime','-1');

%% ================= wiring =================
% (line helper is the local function addL below)

% speed
switch mode
    case 'prescribed', addL(mdl,'SpeedSrc/1','we_calc/1');
    case 'dynamic',    addL(mdl,'Int_wm/1','we_calc/1');
end

% references
addL(mdl,'IsRefSrc/1','RefGen/1');
addL(mdl,'we_calc/1','RefGen/2');

% errors
addL(mdl,'RefGen/1','Sum_ed/1');   addL(mdl,'Int_id/1','Sum_ed/2');
addL(mdl,'RefGen/2','Sum_eq/1');   addL(mdl,'Int_iq/1','Sum_eq/2');

% d-axis PI
addL(mdl,'Sum_ed/1','Kp_d/1');  addL(mdl,'Sum_ed/1','Ki_d/1');
addL(mdl,'Kp_d/1','Sum_ud/1');  addL(mdl,'Int_d/1','Sum_ud/2');
addL(mdl,'Ki_d/1','Sum_Id/1');  addL(mdl,'Kaw_d/1','Sum_Id/2');  addL(mdl,'Sum_Id/1','Int_d/1');
addL(mdl,'Sum_trd/1','Sum_awd/1'); addL(mdl,'Sum_ud/1','Sum_awd/2'); addL(mdl,'Sum_awd/1','Kaw_d/1');

% q-axis PI
addL(mdl,'Sum_eq/1','Kp_q/1');  addL(mdl,'Sum_eq/1','Ki_q/1');
addL(mdl,'Kp_q/1','Sum_uq/1');  addL(mdl,'Int_q/1','Sum_uq/2');
addL(mdl,'Ki_q/1','Sum_Iq/1');  addL(mdl,'Kaw_q/1','Sum_Iq/2');  addL(mdl,'Sum_Iq/1','Int_q/1');
addL(mdl,'Sum_trq/1','Sum_awq/1'); addL(mdl,'Sum_uq/1','Sum_awq/2'); addL(mdl,'Sum_awq/1','Kaw_q/1');

% decoupling
addL(mdl,'Int_id/1','Decouple/1'); addL(mdl,'Int_iq/1','Decouple/2'); addL(mdl,'we_calc/1','Decouple/3');

% voltage assembly
addL(mdl,'Sum_ud/1','Sum_vd/1');    addL(mdl,'Decouple/1','Sum_vd/2');
addL(mdl,'Sum_uq/1','Sum_vq/1');    addL(mdl,'Decouple/2','Sum_vq/2');
addL(mdl,'Sum_vd/1','VoltLimit/1'); addL(mdl,'Sum_vq/1','VoltLimit/2');

% tracking (anti-windup) feedback from the APPLIED voltage
addL(mdl,'VoltLimit/1','Sum_trd/1'); addL(mdl,'Decouple/1','Sum_trd/2');
addL(mdl,'VoltLimit/2','Sum_trq/1'); addL(mdl,'Decouple/2','Sum_trq/2');

% plant
addL(mdl,'VoltLimit/1','MotorElec/1'); addL(mdl,'VoltLimit/2','MotorElec/2');
addL(mdl,'we_calc/1','MotorElec/3');
addL(mdl,'Int_id/1','MotorElec/4');    addL(mdl,'Int_iq/1','MotorElec/5');
addL(mdl,'MotorElec/1','Int_id/1');    addL(mdl,'MotorElec/2','Int_iq/1');
addL(mdl,'Int_id/1','TorqueCalc/1');   addL(mdl,'Int_iq/1','TorqueCalc/2');

if strcmp(mode,'dynamic')
    addL(mdl,'TorqueCalc/1','MechDyn/1'); addL(mdl,'TLSrc/1','MechDyn/2');
    addL(mdl,'Int_wm/1','MechDyn/3');     addL(mdl,'MechDyn/1','Int_wm/1');
end

% ---- log vector (order must match RUN_* scripts) ----
%  1 t        2 wm       3 we       4 Is_ref   5 id*     6 iq*
%  7 region   8 id       9 iq      10 Is_mag  11 Te     12 vd*
% 13 vq*     14 vd      15 vq      16 Vmag    17 m      18 sat_flag
addL(mdl,'Clock/1','LogMux/1');
switch mode
    case 'prescribed', addL(mdl,'SpeedSrc/1','LogMux/2');
    case 'dynamic',    addL(mdl,'Int_wm/1','LogMux/2');
end
addL(mdl,'we_calc/1','LogMux/3');     addL(mdl,'IsRefSrc/1','LogMux/4');
addL(mdl,'RefGen/1','LogMux/5');      addL(mdl,'RefGen/2','LogMux/6');
addL(mdl,'RefGen/3','LogMux/7');      addL(mdl,'Int_id/1','LogMux/8');
addL(mdl,'Int_iq/1','LogMux/9');      addL(mdl,'TorqueCalc/2','LogMux/10');
addL(mdl,'TorqueCalc/1','LogMux/11'); addL(mdl,'Sum_vd/1','LogMux/12');
addL(mdl,'Sum_vq/1','LogMux/13');     addL(mdl,'VoltLimit/1','LogMux/14');
addL(mdl,'VoltLimit/2','LogMux/15');  addL(mdl,'VoltLimit/3','LogMux/16');
addL(mdl,'VoltLimit/4','LogMux/17');  addL(mdl,'VoltLimit/5','LogMux/18');
addL(mdl,'LogMux/1','SimLog/1');

%% ---------------- finish ----------------
% NOTE: Simulink.BlockDiagram.arrangeSystem was introduced in R2015b and is
% deliberately NOT used - every block position is set explicitly above so the
% layout is identical on R2015a.
save_system(mdl);
fprintf('Built and saved model "%s.slx"  (mode = %s)\n', mdl, mode);
fprintf('  Machine parameters injected from ipmsm_params.m\n');
fprintf('  Reference generator : ipmsm_ref_gen.m (single authoritative source)\n');
end

% =========================================================================
function addGain(mdl, nm, gain, pos)
add_block('simulink/Math Operations/Gain', [mdl '/' nm], 'Gain', gain, 'Position', pos);
end

function addSum(mdl, nm, signs, pos)
add_block('simulink/Math Operations/Sum', [mdl '/' nm], 'Inputs', signs, ...
          'IconShape','round', 'Position', pos);
end

function addInt(mdl, nm, ic, pos)
add_block('simulink/Continuous/Integrator', [mdl '/' nm], ...
          'InitialCondition', ic, 'Position', pos);
end

function addL(mdl, a, b)
add_line(mdl, a, b, 'autorouting','on');
end

% =========================================================================
function mkfcn(mdl, name, code, pos)
%MKFCN  Add a MATLAB Function block and set its body.
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/' name], ...
          'Position', pos);
rt = sfroot;                        % chained sfroot.find() fails on old releases
ch = rt.find('-isa','Stateflow.EMChart');
for k = 1:numel(ch)
    if strcmp(ch(k).Path, [mdl '/' name])
        ch(k).Script = code;
        return
    end
end
error('build_foc_model:chartNotFound','Could not set code for block %s', name);
end
