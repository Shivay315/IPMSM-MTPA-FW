function run_all(stage)
%RUN_ALL  One-command driver for the whole project. MATLAB R2015a compatible.
%
%   run_all              % everything, in order
%   run_all('validate')  % maths validation only  (no Simulink needed)
%   run_all('build')     % generate the Simulink models
%   run_all('sim')       % dynamic Simulink tests A-D
%   run_all('compare')   % four-strategy envelope comparison (no Simulink)
%   run_all('wltp')      % WLTP drive-cycle study (no Simulink)
%
%   Only the 'build' and 'sim' stages require Simulink. Everything else is
%   plain MATLAB and will run without it.

if nargin < 1, stage = 'all'; end

here = setup_paths();
% Simulink models and result files are written to 07_generated, keeping the
% source folders clean. This is also where 'results' will appear.
gen = fullfile(here,'07_generated');
if ~exist(gen,'dir'), mkdir(gen); end
cd(gen);

fprintf('\n########################################################################\n');
fprintf('#  Comparative Study of MTPA and Flux-Weakening Control, EV IPMSM Drive\n');
fprintf('#  Target: MATLAB R2015a + Simulink\n');
fprintf('#  Project folder: %s\n', here);
fprintf('########################################################################\n');

v = version('-release');
fprintf('Detected MATLAB release: %s\n', v);
if str2double(v(1:4)) < 2015
    warning('run_all:old','Project targets R2015a; this release is older.');
end
haveSL = ~isempty(ver('simulink'));
fprintf('Simulink available: %s\n\n', mat2str(haveSL));

doAll = strcmpi(stage,'all');

%% ---- 1. parameters ----
if doAll || any(strcmpi(stage,{'validate','compare','wltp','build','sim'}))
    P = ipmsm_params(); %#ok<NASGU>
    fprintf('[1/6] Parameters loaded.\n');
end

%% ---- 2. mathematical validation ----
if doAll || strcmpi(stage,'validate')
    fprintf('\n[2/6] Validating the reference generator ...\n');
    R = validate_ref_gen(true);
    if ~R.pass
        error('run_all:validationFailed', ...
              ['Reference-generator validation FAILED. Stop here and report ' ...
               'the printed CHECK lines - do not proceed to simulation.']);
    end
    fprintf('[2/6] Validation PASSED.\n');
end

%% ---- 3. build the Simulink models ----
if doAll || strcmpi(stage,'build')
    if haveSL
        fprintf('\n[3/6] Building Simulink models ...\n');
        build_foc_model('prescribed','foc_prescribed_v2');
        build_foc_model('dynamic',   'foc_dynamic_v2');
    else
        fprintf('\n[3/6] SKIPPED - Simulink not available.\n');
    end
end

%% ---- 4. dynamic Simulink tests ----
if doAll || strcmpi(stage,'sim')
    if haveSL
        fprintf('\n[4/6] Running prescribed-speed tests A-D ...\n');
        run_prescribed_speed_sim('ABCD');
    else
        fprintf('\n[4/6] SKIPPED - Simulink not available.\n');
    end
end

%% ---- 5. strategy comparison ----
if doAll || strcmpi(stage,'compare')
    fprintf('\n[5/6] Four-strategy envelope comparison ...\n');
    run_strategy_comparison();
end

%% ---- 6. WLTP ----
if doAll || strcmpi(stage,'wltp')
    fprintf('\n[6/6] WLTP drive-cycle study ...\n');
    run_wltp_study(true);
end

fprintf('\n########################################################################\n');
fprintf('#  DONE. Figures saved in %s\n', fullfile(here,'results'));
fprintf('########################################################################\n\n');
end
