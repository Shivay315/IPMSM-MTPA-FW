function root = setup_paths()
%SETUP_PATHS  Add every project folder to the MATLAB path. R2015a compatible.
%
%   root = SETUP_PATHS()  returns the project root directory.
%
%   Run this once per MATLAB session (RUN_ALL calls it automatically).

root = fileparts(mfilename('fullpath'));
sub  = {'01_params','02_refgen','03_model','04_tests','05_studies', ...
        '06_data','07_generated'};
for k = 1:numel(sub)
    d = fullfile(root, sub{k});
    if ~exist(d,'dir'), mkdir(d); end
    addpath(d);
end
addpath(root);
fprintf('Project paths added. Root: %s\n', root);
end
