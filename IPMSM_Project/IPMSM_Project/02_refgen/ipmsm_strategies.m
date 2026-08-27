function [id_star, iq_star, region] = ipmsm_strategies(strategy, Is_ref, we, P)
%IPMSM_STRATEGIES  The four control strategies on one identical plant.
%
%   STRATEGY : 1 = Conventional FOC, id = 0
%              2 = MTPA only (no voltage awareness)
%              3 = Flux weakening only (no MTPA optimisation)
%              4 = Combined MTPA + FW  (calls IPMSM_REF_GEN)
%
%   REGION   : 1 = current-limited / MTPA, 2 = voltage-limited (FW),
%              3 = MTPV, 4 = infeasible at this speed
%
%   All four share the SAME machine parameters, the SAME current limit and
%   the SAME voltage limit, so the comparison is meaningful.

Is  = min(abs(Is_ref), P.Is_max);
% Same resistive-drop reserve as IPMSM_REF_GEN, so every strategy is held to
% an identical voltage budget.
Psi = max(P.Vmax - P.Rs*Is, 0.05*P.Vmax) / max(abs(we), P.we_min);

switch strategy

    % ---------------------------------------------------------------
    case 1  % Conventional FOC, id = 0
    % Torque comes only from the PM term: Te = 1.5*p*lam*iq.
    % No reluctance torque, no flux weakening. The voltage limit caps iq at
    %   (Ld*0 + lam)^2 + (Lq*iq)^2 <= Psi^2  =>  iq <= sqrt(Psi^2-lam^2)/Lq
    % and when Psi < lam the machine cannot be operated at all: the open-
    % circuit back-EMF already exceeds the inverter voltage.
    % ---------------------------------------------------------------
        rad = Psi^2 - P.lam^2;
        if rad <= 0
            id_star = 0; iq_star = 0; region = 4; return
        end
        iq_v    = sqrt(rad)/P.Lq;
        iq_star = min(Is, iq_v);
        id_star = 0;
        region  = 1 + (iq_star < Is - 1e-9);      % 1 current-limited, 2 voltage-limited

    % ---------------------------------------------------------------
    case 2  % MTPA only - no voltage awareness
    % Maximum torque per ampere, but the voltage limit is ignored. Above
    % base speed the demand becomes physically unrealisable; region 4 marks
    % those points so they can be excluded from the comparison.
    % ---------------------------------------------------------------
        dL      = P.dL;
        id_star = (P.lam - sqrt(P.lam^2 + 8*dL^2*Is^2))/(4*dL);
        iq_star = sqrt(max(Is^2 - id_star^2, 0));
        feas    = (P.Ld*id_star + P.lam)^2 + (P.Lq*iq_star)^2 <= Psi^2;
        region  = 1 + 3*(~feas);                   % 1 feasible, 4 infeasible

    % ---------------------------------------------------------------
    case 3  % Flux weakening only - no MTPA optimisation
    % Maximise the torque-producing current iq subject to BOTH limits, and
    % demagnetise only as much as the voltage limit demands. Below base
    % speed this reduces to id = 0. Found by bisection on iq, which is
    % robust; a direct fixed-point iteration converges to a spurious point.
    % ---------------------------------------------------------------
        hi = min(Is, Psi/P.Lq);
        [ok_hi, id_hi] = local_feasible(hi, Is, Psi, P);
        if ok_hi
            id_star = id_hi; iq_star = hi;
            region  = 1 + (hi < Is - 1e-9);
            return
        end
        if ~local_feasible(0, Is, Psi, P)
            id_star = max(-Is, (Psi - P.lam)/P.Ld); iq_star = 0; region = 4; return
        end
        lo = 0;
        for k = 1:200
            mid = 0.5*(lo + hi);
            if local_feasible(mid, Is, Psi, P), lo = mid; else, hi = mid; end
        end
        [~, id_star] = local_feasible(lo, Is, Psi, P);
        iq_star = lo;  region = 2;

    % ---------------------------------------------------------------
    case 4  % Combined MTPA + FW
    % ---------------------------------------------------------------
        [id_star, iq_star, region] = ipmsm_ref_gen(Is_ref, we, P.Rs, P.Ld, ...
                                     P.Lq, P.lam, P.Vmax, P.Is_max, P.we_min);

    otherwise
        error('ipmsm_strategies:badStrategy', 'strategy must be 1..4');
end
end

% -------------------------------------------------------------------------
function [ok, id_use] = local_feasible(iq, Is, Psi, P)
%LOCAL_FEASIBLE  Does an id exist satisfying both limits at this iq?
%   Voltage admits  id in [(-Psi_r-lam)/Ld , (Psi_r-lam)/Ld]
%   Current admits  id in [-sqrt(Is^2-iq^2) , +sqrt(Is^2-iq^2)]
%   We return the least-negative id in the intersection.
ok = false; id_use = 0;
if P.Lq*iq > Psi, return; end
Psi_r  = sqrt(max(Psi^2 - (P.Lq*iq)^2, 0));
id_hi  = (Psi_r - P.lam)/P.Ld;
id_lo  = -sqrt(max(Is^2 - iq^2, 0));
if id_hi < id_lo, return; end
ok = true;  id_use = min(0, max(id_hi, id_lo));
end
