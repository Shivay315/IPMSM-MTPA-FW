%% Step 3A: FOC Coordinate Transformation Validation
clear; clc;

%% Test 1: theta_e = 0
theta_e = 0;
ia = 100*sin(theta_e);
ib = 100*sin(theta_e - 2*pi/3);
ic = 100*sin(theta_e + 2*pi/3);

i_alpha = (2/3)*(ia - 0.5*ib - 0.5*ic);
i_beta  = (2/3)*((sqrt(3)/2)*(ib - ic));
id =  i_alpha*cos(theta_e) + i_beta*sin(theta_e);
iq = -i_alpha*sin(theta_e) + i_beta*cos(theta_e);

i_alpha_r = id*cos(theta_e) - iq*sin(theta_e);
i_beta_r  = id*sin(theta_e) + iq*cos(theta_e);
ia_r = i_alpha_r;
ib_r = -0.5*i_alpha_r + (sqrt(3)/2)*i_beta_r;
ic_r = -0.5*i_alpha_r - (sqrt(3)/2)*i_beta_r;

err1 = max(abs([ia,ib,ic] - [ia_r,ib_r,ic_r]));

fprintf('--- Test 1: theta_e = 0 ---\n');
fprintf('ia=%.4f ib=%.4f ic=%.4f\n', ia, ib, ic);
fprintf('id=%.4f iq=%.4f\n', id, iq);
fprintf('Round-trip error = %.2e\n\n', err1);

%% Test 2: theta_e = pi/4
theta_e = pi/4;
ia = 100*sin(theta_e);
ib = 100*sin(theta_e - 2*pi/3);
ic = 100*sin(theta_e + 2*pi/3);

i_alpha = (2/3)*(ia - 0.5*ib - 0.5*ic);
i_beta  = (2/3)*((sqrt(3)/2)*(ib - ic));
id =  i_alpha*cos(theta_e) + i_beta*sin(theta_e);
iq = -i_alpha*sin(theta_e) + i_beta*cos(theta_e);

i_alpha_r = id*cos(theta_e) - iq*sin(theta_e);
i_beta_r  = id*sin(theta_e) + iq*cos(theta_e);
ia_r = i_alpha_r;
ib_r = -0.5*i_alpha_r + (sqrt(3)/2)*i_beta_r;
ic_r = -0.5*i_alpha_r - (sqrt(3)/2)*i_beta_r;

err2 = max(abs([ia,ib,ic] - [ia_r,ib_r,ic_r]));

fprintf('--- Test 2: theta_e = pi/4 ---\n');
fprintf('ia=%.4f ib=%.4f ic=%.4f\n', ia, ib, ic);
fprintf('id=%.4f iq=%.4f\n', id, iq);
fprintf('Round-trip error = %.2e\n', err2);