function results = validate_ref_gen(verbose)
%VALIDATE_REF_GEN  Independent numerical validation of the reference generator.
%
%   Runs four checks and prints PASS/FAIL for each. Nothing in this project
%   should be trusted until this script passes.
%
%   CHECK 1  MTPA closed form vs brute-force maximisation on the current circle
%   CHECK 2  MTPV closed form vs brute-force maximisation on the voltage ellipse
%   CHECK 3  Full generator vs brute-force constrained global optimum,
%            swept over speed and over three current demands
%   CHECK 4  Constraint compliance: |Is| <= Is_cmd and |V| <= Vmax everywhere
%
%   RESULTS = VALIDATE_REF_GEN returns a struct of the numerical margins.

if nargin < 1, verbose = true; end
P = ipmsm_params();
Te = @(id,iq) 1.5*P.p*(P.lam*iq + (P.Ld-P.Lq).*id.*iq);
pass = true;

fprintf('\n================= REFERENCE-GENERATOR VALIDATION =================\n');
fprintf('Parameters: Rs=%.4g Ohm  Ld=%.4g H  Lq=%.4g H  lam=%.4g Wb  p=%d\n', ...
        P.Rs,P.Ld,P.Lq,P.lam,P.p);
fprintf('Limits    : Is_max=%.0f A  Vmax=%.3f V  (Vdc=%.0f V)\n', ...
        P.Is_max, P.Vmax, P.Vdc);
fprintf('Derived   : lam/Ld = %.2f A (characteristic current)\n', P.I_ch);

%% ---------------- CHECK 1 : MTPA ----------------
Is_list = [50 100 200 300 430];
e1 = 0;
for Is = Is_list
    idg = linspace(-Is, 0, 2e6);
    iqg = sqrt(max(Is^2 - idg.^2, 0));
    [Tb, k] = max(Te(idg, iqg));
    id_c = (P.lam - sqrt(P.lam^2 + 8*P.dL^2*Is^2))/(4*P.dL);
    iq_c = sqrt(max(Is^2 - id_c^2, 0));
    e1 = max(e1, abs(Te(id_c,iq_c) - Tb));
    if verbose
        fprintf(['  MTPA Is=%3.0f A : closed (%8.2f,%7.2f) Te=%7.3f | ' ...
                 'brute (%8.2f,%7.2f) Te=%7.3f\n'], ...
                Is, id_c, iq_c, Te(id_c,iq_c), idg(k), iqg(k), Tb);
    end
end
ok1 = e1 < 1e-3;  pass = pass && ok1;
fprintf('CHECK 1  MTPA closed form            : %s  (max dTe = %.2e Nm)\n', pf(ok1), e1);

%% ---------------- CHECK 2 : MTPV ----------------
e2 = 0;
for Psi = [0.09 0.07 0.05 0.03 0.02]
    u  = linspace(-Psi, Psi, 2e6);
    ig = (u - P.lam)/P.Ld;
    qg = sqrt(max(Psi^2 - u.^2, 0))/P.Lq;
    Tb = max(Te(ig, qg));
    u_c = (P.lam*P.Lq - sqrt((P.lam*P.Lq)^2 + 8*P.dL^2*Psi^2))/(4*P.dL);
    u_c = min(max(u_c,-Psi),Psi);
    id_c = (u_c - P.lam)/P.Ld;
    iq_c = sqrt(max(Psi^2 - u_c^2,0))/P.Lq;
    e2 = max(e2, abs(Te(id_c,iq_c) - Tb));
end
ok2 = e2 < 1e-3;  pass = pass && ok2;
fprintf('CHECK 2  MTPV closed form            : %s  (max dTe = %.2e Nm)\n', pf(ok2), e2);

%% ---------------- CHECK 3 & 4 : full generator ----------------
rpm  = [0 500 1000 2000 3000 3660 4000 5000 6000 8000 10000 11400];
demands = [P.Is_max 250 100];
e3 = 0; worstI = 0; worstV = 0; n4 = 0;
for Is_ref = demands
    if verbose
        fprintf('\n  --- Is_ref = %.0f A  (Is_cmd = %.0f A) ---\n', ...
                Is_ref, min(Is_ref,P.Is_max));
        fprintf(['  %7s %5s %10s %9s %8s %9s %9s %6s %6s\n'], ...
                'rpm','reg','id*','iq*','|Is|','Te','Vmag','I_ok','V_ok');
    end
    for r = rpm
        we = P.p*r*2*pi/60;
        [id_s, iq_s, reg] = ipmsm_ref_gen(Is_ref, we, P.Rs, P.Ld, P.Lq, ...
                                          P.lam, P.Vmax, P.Is_max, P.we_min);
        Is_cmd = min(Is_ref, P.Is_max);
        Ism = hypot(id_s, iq_s);
        vd  = P.Rs*id_s - we*P.Lq*iq_s;
        vq  = P.Rs*iq_s + we*(P.Ld*id_s + P.lam);
        Vm  = hypot(vd, vq);

        I_ok = Ism <= Is_cmd + 1e-6;
        V_ok = (Vm <= P.Vmax + 1e-6) || reg == 4;   % region 4 is flagged infeasible
        worstI = max(worstI, Ism - Is_cmd);
        if reg ~= 4, worstV = max(worstV, Vm - P.Vmax); end
        if reg == 4, n4 = n4 + 1; end
        pass = pass && I_ok && V_ok;

        % brute-force constrained global optimum
        Psi = max(P.Vmax - P.Rs*Is_cmd, 0.05*P.Vmax)/max(abs(we),P.we_min);
        idg = linspace(-Is_cmd, 0, 4e5);
        iqc = sqrt(max(Is_cmd^2 - idg.^2, 0));
        rad = Psi^2 - (P.Ld*idg + P.lam).^2;
        iqv = -ones(size(idg));
        m   = rad > 0;
        iqv(m) = sqrt(rad(m))/P.Lq;
        iqb = min(iqc, iqv);
        good = iqb >= 0;
        if any(good)
            Tb = max(Te(idg(good), iqb(good)));
            e3 = max(e3, abs(Te(id_s,iq_s) - Tb));
        end
        if verbose
            fprintf('  %7.0f %5d %10.2f %9.2f %8.1f %9.2f %9.2f %6s %6s\n', ...
                    r, reg, id_s, iq_s, Ism, Te(id_s,iq_s), Vm, pf(I_ok), pf(V_ok));
        end
    end
end
ok3 = e3 < 1e-2;  ok4 = (worstI <= 1e-6) && (worstV <= 1e-6);
pass = pass && ok3 && ok4;
fprintf('\nCHECK 3  Generator vs global optimum : %s  (max dTe = %.2e Nm)\n', pf(ok3), e3);
fprintf('CHECK 4  Constraint compliance       : %s  (worst dI = %.2e A, dV = %.2e V)\n', ...
        pf(ok4), worstI, worstV);
fprintf('         region-4 (infeasible demand) points encountered: %d\n', n4);

fprintf('\n----------------------------------------------------------------\n');
fprintf('OVERALL: %s\n', pf(pass));
fprintf('----------------------------------------------------------------\n\n');

results = struct('pass',pass,'mtpa_err',e1,'mtpv_err',e2,'gen_err',e3, ...
                 'worst_I_excess',worstI,'worst_V_excess',worstV,'n_infeasible',n4);
end

function s = pf(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end
