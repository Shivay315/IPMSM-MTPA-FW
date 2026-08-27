# Execution Guide — MATLAB R2015a + Simulink

Every step below states the **exact command**, the **expected result**, and the
**pass / fail condition**. You can compare digit-for-digit.

## Validation status — what has actually been executed

| Component | Status |
|---|---|
| `ipmsm_params` | **EXECUTED** in GNU Octave 11.3 — values below are real output |
| `ipmsm_ref_gen` | **EXECUTED** — `validate_ref_gen` returns `OVERALL: PASS` |
| `ipmsm_strategies` | **EXECUTED** — full 4-strategy sweep, 601 speed points |
| `run_strategy_comparison` | **EXECUTED** headless — table below is real output |
| `wltp_preprocess`, `run_wltp_study` | **EXECUTED** — full 1801-sample cycle |
| All 12 `.m` files | **PARSE-CHECKED** by Octave — 0 failures |
| Post-R2015a API scan | **PASS** — 0 occurrences |
| Non-ASCII scan | **PASS** — all files pure ASCII (R2015a reads `.m` in system locale) |
| `build_foc_model`, `run_prescribed_speed_sim` | **Requires final MATLAB R2015a execution on your machine** (Simulink-only; Octave has no Simulink) |

Numbers marked *(verified)* below were produced by real execution, not predicted.
The two Simulink steps (3 and 4) are the only unexecuted parts of the project.

## R2015a compatibility statement

Verified by automated scan of all 12 `.m` files — **zero** post-R2015a functions:

| Avoided (too new) | Used instead |
|---|---|
| `Simulink.BlockDiagram.arrangeSystem` (R2015b) | explicit `Position` on every block |
| `yline` (R2018b) | `plot([x1 x2],[v v],'k--')` + `text` |
| `string`, `contains`, `strlength` (R2016b) | `char`, `strcmp`, `sprintf` |
| `Simulink.SimulationInput` / `parsim` (R2017a) | plain `sim(mdl)` + `assignin('base',…)` |
| `readmatrix` / `readtable` opts (R2019a) | `fopen` + `textscan` |
| PID Controller block + Control System Toolbox | PI built from Gain/Integrator/Sum |

Oldest requirement used: `strjoin` (R2013a). `gobjects` was removed in favour of
a plain cell array so the code also runs in GNU Octave (used for validation). **No toolboxes beyond base MATLAB + Simulink are required.**

---

## STEP 0 — Put the folder on the path

**EXACT COMMAND**
```matlab
cd('D:\Prince Mridul\Extra\Shivay\Major Project\IPMSM_Project')
setup_paths
```

**PASS** — `which ipmsm_params` returns a path inside `IPMSM_Project_params`.
**FAIL** — "not found": you are in the wrong folder.

---

## STEP 1 — Parameters *(no Simulink needed)*

**EXACT COMMAND**
```matlab
P = ipmsm_params()
```

**EXPECTED RESULT** — a struct. Key derived values:

| Field | Expected |
|---|---|
| `P.Vmax` | `207.8461` |
| `P.I_ch` (λm/Ld) | `427.7778` |
| `P.Te_max_model` | `251.1142` *(verified)* |
| `P.n_base` | `3659.7` rpm *(verified)* |
| `P.Kp_d`, `P.Kp_q` | `0.28274`, `0.80111` *(verified — identical to your original model)* |
| `P.Ki_d` | `16.6504` *(verified — identical to your original model)* |

**PASS** — `P.Te_max_model` is 251.11 (matches the 250 Nm nameplate to 0.4 %).
**FAIL** — anything near 231.3 means the old, incorrect MTPA formula is still
being used somewhere.

---

## STEP 2 — Mathematical validation *(no Simulink needed)* ★ most important step

**EXACT COMMAND**
```matlab
R = validate_ref_gen(true)
```

**EXPECTED RESULT**
```
CHECK 1  MTPA closed form            : PASS  (max dTe = 6.31e-12 Nm)
CHECK 2  MTPV closed form            : PASS  (max dTe = 4.63e-11 Nm)
CHECK 3  Generator vs global optimum : PASS  (max dTe = 1.86e-04 Nm)
CHECK 4  Constraint compliance       : PASS  (worst dI = 1.42e-14 A, dV = 0.00e+00 V)
         region-4 (infeasible demand) points encountered: 1
OVERALL: PASS
```

Spot-check rows (`Is_ref = 430 A`):

| rpm | region | id* | iq* | Te | Vmag |
|---|---|---|---|---|---|
| 0 | 1 | −251.27 | 348.95 | 251.11 | 2.3 |
| 3660 | 2 | −256.3 | 345.3 | 251.05 | 207.2 |
| 10000 | 2 | −410.5 | 128.2 | 122.52 | 207.8 |
| 11400 | 2 | −415.0 | 112.5 | 108.28 | 207.8 |

And critically, at `Is_ref = 100 A` every row must show **`|Is| = 100.0`** — never 430.

**PASS** — `OVERALL: PASS` and `R.pass == 1`.
**FAIL** — any CHECK failing. Do **not** proceed to Simulink; send me the printed
CHECK lines. `CHECK 4` failing means a constraint is violated (real bug);
`CHECK 3` failing means the region-selection logic picked a sub-optimal point.

---

## STEP 3 — Build the Simulink models *(Simulink required)*

**EXACT COMMAND**
```matlab
build_foc_model('prescribed','foc_prescribed_v2');
build_foc_model('dynamic','foc_dynamic_v2');
```

**EXPECTED RESULT**
```
Built and saved model "foc_prescribed_v2.slx"  (mode = prescribed)
  Machine parameters injected from ipmsm_params.m
  Reference generator : ipmsm_ref_gen.m (single authoritative source)
```
Two `.slx` files appear. Opening `foc_prescribed_v2` shows ~30 blocks: `RefGen`,
two PI chains, `Decouple`, `VoltLimit`, `MotorElec`, `TorqueCalc`, `LogMux`, `SimLog`.

**PASS** — both models build with no error, and
```matlab
set_param('foc_prescribed_v2','SimulationCommand','update')
```
completes **with no algebraic-loop warning**.

**FAIL** —
- *"Invalid setting … MATLAB Function"* → the Stateflow `.Script` API differs;
  tell me and I will switch to a `matlabFunctionBlock`-free variant.
- *Algebraic loop reported* → tell me which blocks; the graph is loop-free by
  construction so this would indicate a wiring bug.
- *"Undefined function 'ipmsm_ref_gen'"* → Step 0 was skipped.

---

## STEP 4 — Dynamic tests A–D *(Simulink required)*

**EXACT COMMAND**
```matlab
OUT = run_prescribed_speed_sim('ABCD')
```

**EXPECTED RESULT — pass conditions per test**

| Test | Condition | Expected |
|---|---|---|
| A (1000 rpm) | `\|id−id*\|`, `\|iq−iq*\|` | < 2 A |
| A | on MTPA trajectory | `\|i−i_MTPA\|` < 1 A (id≈−251.3, iq≈348.9) |
| A | voltage saturation | ~0 % of samples |
| B (2000→6000) | regions visited | `[1 2]`, exactly 1 region change |
| B | max step in `id*` | < 25 A per solver step (continuous transition) |
| C (10000 rpm) | `id*` | ≈ −410 A (strongly negative) |
| C | region | `2` (FW Region I) |
| D (0→n_max) | `\|Te_sim − Te_analytical\|` | < 5 Nm |
| **all** | `max\|V\|` | **≤ 207.85 V** |
| **all** | `max\|Is\|` | **≤ 430 A** |
| **all** | torque-equation residual | < 1e−6 |

Final line must read `PRESCRIBED-SPEED VALIDATION OVERALL: PASS`.

**FAIL interpretations**
- `max|V| > 207.85` → the limiter is not in the active path.
- `|id−id*|` large **and** `sat` high → windup; the tracking feedback
  (`Sum_trd`/`Sum_trq`) is mis-wired.
- Test B showing many region changes → chattering at the MTPA/FW boundary;
  tell me and I will add hysteresis.
- Test D error > 5 Nm → sweep too fast for quasi-steady comparison; raise `T`
  in the Test D block from 4.0 s to 10 s and re-run.

---

## STEP 5 — Four-strategy comparison *(no Simulink needed)*

**EXACT COMMAND**
```matlab
R = run_strategy_comparison();              % with figures
% R = run_strategy_comparison(430,false,false);   % headless, numbers only
```

**EXPECTED RESULT**

| strategy | Te@0 Nm | Nm/A@0 | max kW | max rpm | Te@n_max |
|---|---|---|---|---|---|
| S1 FOC id=0 | 149.00 | 0.347 | 43.90 | **8493** | 0 |
| S2 MTPA only | 251.11 | 0.584 | 94.93 | **3610** | 0 |
| S3 FW only | 149.00 | 0.347 | 129.26 | 11400 | 108.28 |
| **S4 MTPA+FW** | **251.11** | **0.584** | **129.26** | **11400** | **108.28** |

Ten figures are written to `IPMSM_Project/07_generated/results/`.

**PASS** — table matches to ±0.1, and figure 5 (`|V|` vs speed) shows every
feasible curve at or below the `V_max` line. *(Verified numerically: max `|V|`
over all feasible points = **207.7895 V** ≤ `Vmax` = 207.8461 V; max `|Is|` =
**430.0000 A** ≤ 430 A — the original `Vmag > Vmax` symptom is gone.)*
**FAIL** — S4 not best on torque *and* speed range means the generator regressed.

Headline results for the report:
- S4 gives **+68.5 % torque** and **+68.4 % Nm/A** over S1 at standstill.
- S1 reaches only **8495 of 11400 rpm** — id=0 control cannot reach rated speed.
- S2 collapses at **3615 rpm** — MTPA alone is unusable above base speed.
- **S3 and S4 coincide exactly above base speed** — once both limits bind, the
  circle∩ellipse intersection is a single point, so all correct FW schemes land
  on it. MTPA's benefit is confined to the constant-torque region.

---

## STEP 6 — WLTP drive-cycle study *(no Simulink needed)*

**EXACT COMMAND**
```matlab
W = run_wltp_study(true);
```

**EXPECTED RESULT — preprocessing**
```
distance 23.266 km, max motor 9658 rpm (n_max 11400)
T_motor max 93.1 Nm, min -67.1 Nm (capability 251.1)
samples over torque capability : 0
samples over speed  limit      : 0
```

**EXPECTED RESULT — comparison**

| strategy | E_cu kWh | Wh/km | unmet s | complete |
|---|---|---|---|---|
| S1 FOC id=0 | 0.0254 | 62.95 | **149** | NO |
| S2 MTPA only | 0.0202 | 59.71 | **134** | NO |
| S3 FW only | 0.0277 | 97.86 | 0 | YES |
| **S4 MTPA+FW** | **0.0226** | **97.65** | 0 | YES |

*(All verified by execution. Delivered work `E_mech`: S1 1.4393, S2 1.3689,
S3 2.2491, **S4 2.2493 kWh** — S1 and S2 do 36-39 % less work. Energy balance
`P_elec - (P_mech + P_cu)` closes to 7e-12 .. 1.5e-11 W.)*

**PASS** — energy-balance column `< 1e-6 W` (it should be ~1e-11), and only S3
and S4 complete the cycle.

**Read this carefully for the report:** S1 and S2 show *lower* energy **only
because they fail to complete the cycle** and therefore perform less work. The
valid comparison is **S4 vs S3: S4 uses 18.4 % less copper loss for the same
delivered work.** Do not quote S1/S2 Wh/km as an efficiency advantage.

---

## STEP 7 — Everything at once

**EXACT COMMAND**
```matlab
run_all
```
Runs Steps 1–6 in order and stops with an error if Step 2 fails.

---

## Known manual steps in R2015a

**None.** The model is generated by script; no block is placed or wired by hand.
The only requirement is Step 0 (`setup_paths`), because the `RefGen` MATLAB
Function block calls `ipmsm_ref_gen.m` from the path.

If you prefer the model to be fully self-contained (no path dependency), say so
and I will inline the generator body into the block instead of calling out to it.

---

## If something fails

Send me:
1. the full console text from the failing command,
2. `ver` output,
3. for a Simulink failure, the diagnostic-viewer message.

Do **not** hand-edit the `.slx` — change the source `.m` and re-run
`build_foc_model`. Hand edits are lost on the next build, which is the point:
the model is a build artifact, not a document.
