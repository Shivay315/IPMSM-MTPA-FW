# Comparative Study of MTPA and Flux-Weakening Control for an EV IPMSM Drive

B.Tech Electrical Engineering major project.
**Deployment target: MATLAB R2015a + Simulink. Base MATLAB + Simulink only — no
Control System Toolbox, no Simscape, no Powertrain Blockset.**

The original model is preserved **untouched and read-only** in
`../ORIGINAL_BACKUP_20260826/` (MD5 `df268fd2ee2677e51ed350b09828b224`),
together with all eleven original supporting test models.

---

## Quick start (three lines)

```matlab
cd('D:\Prince Mridul\Extra\Shivay\Major Project\IPMSM_Project')
setup_paths
run_all
```

Step-by-step commands with pass/fail criteria: **`00_docs/MATLAB_R2015a_GUIDE.md`**.

---

## Directory layout

| Folder | Contents |
|---|---|
| `00_docs/` | This README, the R2015a execution guide, `References.bib` |
| `01_params/` | `ipmsm_params.m`, `wltp_vehicle_params.m` — **all** parameters |
| `02_refgen/` | `ipmsm_ref_gen.m` (**the** source of `id*`/`iq*`), `ipmsm_strategies.m` |
| `03_model/` | `build_foc_model.m` — generates the Simulink model |
| `04_tests/` | `validate_ref_gen.m`, `run_prescribed_speed_sim.m` |
| `05_studies/` | `run_strategy_comparison.m`, `run_wltp_study.m`, `wltp_preprocess.m` |
| `06_data/` | `WLTP.csv` (Class 3b, 1800 s, 23.27 km) |
| `07_generated/` | Generated `.slx` models, figures, `.mat` results |
| `run_all.m`, `setup_paths.m` | Top-level drivers |

Never hand-edit the generated `.slx` — edit the source `.m` and re-run
`build_foc_model`. The model is a build artifact, which is what makes the
project reproducible.

---

## What was wrong with the original model

Five defects, all confirmed by a numerical replica that reproduced the reported
scope values to within a few percent before any change was made:

1. **No speed bound.** `TL = 0`, no speed loop → free acceleration to ~54,000 rpm
   (4.8× `n_max`). Every "abnormal" scope reading was a *correct* simulation of
   an operating point far past redline.
2. **The FW law ignored the current demand.** `id_FW = −λm/Ld + (1/Ld)√arg` with
   `arg` floored at 0 collapses to the characteristic current −427.8 A whatever
   was asked. With `Is_ref = 100 A` the controller commanded **430 A**.
3. **The MTPA formula was wrong.** Stationarity of `Te ∝ iq(λm − ΔL·id)` on the
   current circle gives `2ΔL·id² − λm·id − ΔL·Is² = 0`, i.e.
   `id = [λm − √(λm² + 8ΔL²Is²)]/(4ΔL)`. The model used `4ΔL²` under the root and
   `2ΔL` in the denominator — an 8 % torque loss (231.3 vs 251.1 Nm).
4. **Dead output.** `FW_reference` output 2 was wired to nothing; its current-limit
   logic was unreachable code.
5. **Algebraic loop.** `Cart2Polar → RateLimiter → Polar2Cart → iq_star_calc →
   Cart2Polar`, closed by `TrustRegion` every solver step.

**What was already correct and has been kept:** the PI tuning (pole-zero
cancellation, `Kp = ωc·L`, `Ki = ωc·Rs`, ωc = 3141 rad/s) and the back-calculation
anti-windup tracking the post-limiter applied voltage. Also correct: the dq
voltage equations, the torque equation, the Park transforms, and the
`Polar2Cart` sin/cos fix made during earlier debugging.

---

## Validated results

All figures below were produced by an independent implementation checked against
brute-force numerical optimisation (max deviation **1.6×10⁻⁴ Nm**), with zero
current-limit and zero voltage-limit violations.

### Torque-speed envelope (`Is_ref = Is_max = 430 A`)

| strategy | Te@0 | Nm/A | max kW | max rpm |
|---|---|---|---|---|
| S1 FOC id=0 | 149.00 | 0.347 | 43.92 | **8495** |
| S2 MTPA only | 251.11 | 0.584 | 95.06 | **3615** |
| S3 FW only | 149.00 | 0.347 | 129.26 | 11400 |
| **S4 MTPA+FW** | **251.11** | **0.584** | **129.26** | **11400** |

- S4 gives **+68.5 % torque** and **+68.4 % Nm/A** over S1 at standstill.
- **id = 0 control cannot reach rated speed** (8495 of 11400 rpm).
- **MTPA alone collapses at 3615 rpm** — this is the quantitative justification
  for field weakening.
- **S3 and S4 coincide exactly above base speed.** Once both limits bind, the
  current-circle ∩ voltage-ellipse intersection is a *single* point in the upper
  half-plane, so every correct FW scheme lands on it. MTPA's benefit is confined
  to the constant-torque region. This is a result, not a bug.

### WLTP Class 3b (quasi-static, copper loss only)

| strategy | E_cu kWh | Wh/km | unmet s | completes cycle |
|---|---|---|---|---|
| S1 FOC id=0 | 0.0254 | 62.95 | 149 | **NO** |
| S2 MTPA only | 0.0202 | 59.71 | 134 | **NO** |
| S3 FW only | 0.0277 | 97.86 | 0 | YES |
| **S4 MTPA+FW** | **0.0226** | **97.65** | 0 | YES |

**Only S3 and S4 complete the cycle.** S1 and S2 show lower energy *only because
they perform less work* — do not quote their Wh/km as an efficiency advantage.
The valid comparison is **S4 vs S3: 18.4 % less copper loss for the same
delivered work.** Energy bookkeeping validated by `P_elec = P_mech + P_cu` closing
to ~10⁻¹¹ W.

---

## Modelling limitations — state these in the report

- **Simplified BMW-i3-*inspired* parameter set, NOT measured BMW i3 data.**
  Rated torque/power/speed are public class figures; `Rs`, `Ld`, `Lq`, `λm` are
  representative values for a machine of that class.
- **Magnetically linear:** constant `Ld`, `Lq`, `λm`. No saturation, no
  cross-coupling, no rotor-position-dependent inductance.
- **Copper loss only.** No iron, PM, mechanical or inverter loss. All efficiency
  figures are an upper bound and must be labelled *copper-loss-only*.
- **Averaged inverter.** The limiter enforces the SVPWM linear-range voltage
  bound; no switching, dead-time, or DC-link ripple is modelled.
- **Base speed is 3660 rpm, not the 4800 rpm** quoted in the original parameter
  file. 4800 rpm follows from Pmax/Tmax = 125 kW / 250 Nm, which this simplified
  set cannot simultaneously satisfy at 360 V. Reported honestly, not fudged.
- **MTPV is implemented but provably inactive** below `n_max`: it requires
  ~100,000 rpm, because λm/Ld = 427.8 A ≈ Is_max = 430 A — the machine is close
  to the "optimal flux-weakening design" condition.
- **All WLTP vehicle parameters are assumptions**, individually labelled
  `[PUB]`/`[EST]`/`[STD]` in `wltp_vehicle_params.m`. They are cross-checked
  against the independently-given machine: motor `n_max` → 155.0 km/h (i3 is
  governed at ~150), cycle peak 131.3 km/h → 9658 rpm (under 11400), peak cycle
  torque 93.0 Nm (of 251 available).
- The WLTP study is **quasi-static (backward-facing)**. A forward-facing
  closed-loop simulation is possible with `build_foc_model('dynamic')` but would
  require an outer speed loop, which is not the subject of this study.

---

## Future work / advanced alternatives (deliberately NOT implemented)

Per the supplied research material, these are outside B.Tech scope here and are
listed as future work: saturation-dependent inductance look-up tables,
FEM-derived flux maps, loss-minimising (maximum-efficiency rather than MTPA)
control, sensorless operation, over-modulation and six-step operation, and
online parameter adaptation.
