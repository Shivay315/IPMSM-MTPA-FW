function [id_star, iq_star, region] = ipmsm_ref_gen(Is_ref, we, Rs, Ld, Lq, lam, Vmax, Is_max, we_min)
%IPMSM_REF_GEN  Physically correct MTPA / field-weakening operating point.
%#codegen
%
%   [ID_STAR, IQ_STAR, REGION] = IPMSM_REF_GEN(IS_REF, WE, RS, LD, LQ, LAM, VMAX, IS_MAX, WE_MIN)
%
%   Returns the torque-maximising (id,iq) reference that satisfies BOTH the
%   current-limit circle and the voltage-limit ellipse at electrical speed WE.
%   This is the single authoritative source of id* and iq* in the project.
%
%   REGION codes:
%     1 = MTPA          (below base speed; voltage limit not active)
%     2 = FW Region I   (on current circle AND voltage ellipse)
%     3 = MTPV          (deep FW; voltage ellipse only, inside current circle)
%     4 = Infeasible    (demand cannot meet the voltage limit at this speed)
%
% =========================================================================
% MATHEMATICAL BASIS
% =========================================================================
% Motor convention, steady state, magnetically linear machine:
%
%   vd = Rs*id - we*Lq*iq
%   vq = Rs*iq + we*(Ld*id + lam)
%   Te = (3/2)*p*[ lam*iq + (Ld-Lq)*id*iq ]           ... (T)
%
% Neglecting the resistive drop (Rs*Is << Vmax at the speeds where the
% voltage limit is active) the voltage constraint vd^2+vq^2 <= Vmax^2 becomes
% a constraint on FLUX magnitude:
%
%   (Ld*id + lam)^2 + (Lq*iq)^2  <=  Psi^2 ,   Psi = Vmax/|we|      ... (V)
%
% This is an ellipse in the (id,iq) plane centred at (-lam/Ld, 0) with
% semi-axes Psi/Ld (d) and Psi/Lq (q). It shrinks as speed rises.
% The current limit is a circle centred at the origin:
%
%   id^2 + iq^2  <=  Is^2 ,   Is = min(Is_ref, Is_max)              ... (C)
%
% The feasible set is the intersection of (V) and (C); both are convex, so
% the torque-maximising point is unique and lies either at the interior
% optimum (MTPA), on one boundary (MTPV), or on both (Region I).
%
% ---- resistive-drop allowance ------------------------------------------
% (V) drops the Rs*i term. By the triangle inequality the true voltage
% magnitude obeys |v| <= |v_emf| + Rs*|i|, so enforcing
%
%       Vmax_eff = Vmax - Rs*Is                                     ... (R)
%
% in place of Vmax GUARANTEES |v| <= Vmax rather than merely approximating
% it. For this machine Rs*Is_max = 2.28 V, i.e. a 1.1% reserve. Without (R)
% the commanded voltage overshoots Vmax by exactly that amount and the SVPWM
% limiter clips continuously - which was one of the observed symptoms of the
% original model.
%
% ---- (a) MTPA : maximise (T) subject to (C) with equality ---------------
% With dL = Lq-Ld > 0, Te  prop. to  iq*(lam - dL*id). Lagrange conditions give
%       -dL*iq = 2*mu*id ,   lam - dL*id = 2*mu*iq
% Eliminating mu and substituting iq^2 = Is^2 - id^2:
%
%       2*dL*id^2 - lam*id - dL*Is^2 = 0
%   =>  id = [ lam - sqrt(lam^2 + 8*dL^2*Is^2) ] / (4*dL)            ... (M)
%
%   (Negative root taken: reluctance torque requires id < 0.)
%   NOTE: an earlier version of this project used
%         id = [lam - sqrt(lam^2 + 4*dL^2*Is^2)]/(2*dL), which does NOT
%         satisfy the stationarity condition and under-estimates torque by
%         ~8% at rated current. (M) has been verified against brute-force
%         numerical maximisation of (T) on the circle.
%
% ---- (b) MTPV : maximise (T) subject to (V) with equality --------------
% Substituting u = Ld*id + lam (d-axis flux), so id = (u-lam)/Ld and
% Lq^2*iq^2 = Psi^2 - u^2, the same Lagrange procedure gives
%
%       2*dL*u^2 - lam*Lq*u - dL*Psi^2 = 0
%   =>  u = [ lam*Lq - sqrt((lam*Lq)^2 + 8*dL^2*Psi^2) ] / (4*dL)     ... (P)
%
% ---- (c) Region I : intersection of (C) and (V), both with equality ----
% Substituting iq^2 = Is^2 - id^2 into (V):
%
%       (Ld^2 - Lq^2)*id^2 + 2*lam*Ld*id + (lam^2 + Lq^2*Is^2 - Psi^2) = 0
%
% a quadratic in id. Of its real roots we keep those that are physically
% admissible (id <= 0, |id| <= Is) and select the one of larger torque.
%
% All three closed forms have been verified against brute-force numerical
% optimisation over the constraint sets (see VALIDATE_REF_GEN).
% =========================================================================

dL = Lq - Ld;

% ---- current demand: never exceed the inverter limit -------------------
Is = min(abs(Is_ref), Is_max);

% ---- available flux magnitude at this speed ----------------------------
we_a = max(abs(we), we_min);

% Reserve the worst-case resistive drop so the voltage limit is guaranteed,
% not merely approximated -- see (R) above.
Vmax_eff = max(Vmax - Rs*Is, 0.05*Vmax);
Psi      = Vmax_eff / we_a;

% ---------------- Region 1 : MTPA ---------------------------------------
id_M = (lam - sqrt(lam^2 + 8*dL^2*Is^2)) / (4*dL);
iq_M = sqrt(max(Is^2 - id_M^2, 0));

if (Ld*id_M + lam)^2 + (Lq*iq_M)^2 <= Psi^2
    id_star = id_M;  iq_star = iq_M;  region = 1;
    return
end

% ---------------- Region 3 : MTPV ---------------------------------------
% Reached only when the unconstrained optimum on the voltage ellipse already
% lies inside the current circle. For the present parameter set
% (lam/Ld = 427.8 A, Is_max = 430 A) this requires ~100,000 rpm and is
% therefore INACTIVE below n_max = 11,400 rpm. Retained for correctness.
u_V = (lam*Lq - sqrt((lam*Lq)^2 + 8*dL^2*Psi^2)) / (4*dL);
u_V = min(max(u_V, -Psi), Psi);
id_V = (u_V - lam)/Ld;
iq_V = sqrt(max(Psi^2 - u_V^2, 0))/Lq;

if id_V^2 + iq_V^2 <= Is^2
    id_star = id_V;  iq_star = iq_V;  region = 3;
    return
end

% ---------------- Region 2 : FW Region I --------------------------------
a = Ld^2 - Lq^2;                       % < 0
b = 2*lam*Ld;
c = lam^2 + Lq^2*Is^2 - Psi^2;
D = b*b - 4*a*c;

if D >= 0
    sD = sqrt(D);
    r1 = (-b + sD)/(2*a);
    r2 = (-b - sD)/(2*a);

    id_best = 0;  iq_best = 0;  Te_best = -inf;  found = false;
    for k = 1:2
        if k == 1, idc = r1; else, idc = r2; end
        if idc <= 0 && idc >= -Is
            iqc = sqrt(max(Is^2 - idc^2, 0));
            Tec = lam*iqc + (Ld-Lq)*idc*iqc;      %  prop. to  Te, common factor dropped
            if Tec > Te_best
                Te_best = Tec;  id_best = idc;  iq_best = iqc;  found = true;
            end
        end
    end

    if found
        id_star = id_best;  iq_star = iq_best;  region = 2;
        return
    end
end

% ---------------- Region 4 : demand infeasible --------------------------
% The current circle of radius Is does not reach the voltage ellipse: at this
% speed the demanded current is too small to suppress the back-EMF. Apply the
% maximum available demagnetising current. The voltage limit CANNOT be met
% here - this is a physical statement about the demand, not a controller
% failure, and is flagged so post-processing can exclude such points.
id_star = -Is;
iq_star = 0;
region  = 4;

end
