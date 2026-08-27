clc; clear;
Rs = 5.3e-3;
Ld = 0.090e-3;
Lq = 0.255e-3;
lambda_m = 0.0385;
p = 6;
Imax = 430;


% MTPA current sweep
I = 0:1:Imax;

DeltaL = Lq - Ld;

id_mtpa = (lambda_m - sqrt(lambda_m^2 + ...
           4*(DeltaL^2)*(I.^2))) / (2*DeltaL);

iq_mtpa = sqrt(I.^2 - id_mtpa.^2);

% Electromagnetic torque
Te_mtpa = 1.5*p*(lambda_m.*iq_mtpa + ...
          (Ld-Lq).*id_mtpa.*iq_mtpa);

% Display selected operating points
fprintf('I = 100 A: id* = %.3f A, iq* = %.3f A, Te = %.3f Nm\n', ...
        id_mtpa(101), iq_mtpa(101), Te_mtpa(101));

fprintf('I = 430 A: id* = %.3f A, iq* = %.3f A, Te = %.3f Nm\n', ...
        id_mtpa(431), iq_mtpa(431), Te_mtpa(431));
    
    
% ---------------------------------------------------------
% MTPA validation at fixed current magnitude
% ---------------------------------------------------------

I_test = 100;                         % A
id_sweep = -I_test:0.1:0;             % A
iq_sweep = sqrt(I_test^2 - id_sweep.^2);

% Torque along constant-current circle
Te_sweep = 1.5*p*(lambda_m.*iq_sweep + ...
           (Ld-Lq).*id_sweep.*iq_sweep);

% MTPA analytical point
id_opt = (lambda_m - sqrt(lambda_m^2 + ...
         4*(DeltaL^2)*I_test^2)) / (2*DeltaL);

iq_opt = sqrt(I_test^2 - id_opt^2);

Te_opt = 1.5*p*(lambda_m*iq_opt + ...
        (Ld-Lq)*id_opt*iq_opt);

% Plot
figure;
plot(id_sweep, Te_sweep, 'LineWidth', 1.5);
hold on;
plot(id_opt, Te_opt, 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
grid on;

xlabel('i_d (A)');
ylabel('Electromagnetic Torque (Nm)');
title('MTPA Validation at I_s = 100 A');
legend('Torque at constant current', 'Analytical MTPA point');

fprintf('\n--- MTPA Validation ---\n');
fprintf('Optimal id* = %.3f A\n', id_opt);
fprintf('Optimal iq* = %.3f A\n', iq_opt);
fprintf('Maximum torque at MTPA = %.3f Nm\n', Te_opt);