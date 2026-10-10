# Belt drive — build routine, measured belt-on states, idler design

§22 belt-on routine (B0–B13) · §22.1 belt-on plant states, J01 2026-08-12 · §22.2 idler
specification · §22.3 belt geometry and tension · §22.4 J02 acceptance (SCB pulley) · §22.5 J01
recipe A (superseded) · **§22.6 J01 recipe B, pulley of record** · §22.7 B12 MIT law ·
§22.8 deferred items.

*Hub: [`README.md`](README.md). Master table: [`CONSTANTS.md`](CONSTANTS.md) §8.*

---

## 22. Belt-on routine

Every belt-on number is a **difference against the joint's own belt-off baseline**, never
another joint's (the inter-joint drag spread is ~25% and real).

Plant for B1–B8: belt fitted, idlers per §22.2, output pulley bare, leg links **off**. Gate on
**amps**, not percentages: percentages move with calibration corrections, amps do not.

| # | Step | Procedure | Gate | J01 result |
|---|---|---|---|---|
| **B0** | Fit belt | Idlers per §22.2, belt per §22.3, C = 44.5 mm. Then run the idler acceptance test (§22.2) | rollers coast ≥ 1 s, no axial walk | done |
| **B1** | Alignment | `V` | ≤ 8° elec from stored ZEA | 0.62° (recipe B) |
| **B2** | Electrical tripwire | `1` then `3` (phases 1 and 3 only; phase 4 ratchets belt-on, §22.6.6) | `R_eff` within ~2% of stored | 0.22404 Ω (+0.26%) |
| **B3** | Belt drag | Phase 5, then again with direction order swapped (`-` then `5`). Store each direction **pooled over both orders** | `drag_c` 0.30–0.40 A (J01 band; restate per joint). Belt-on `Ke` is never carried | 0.3169 / 0.3330 A (§22.6.6) |
| **B4** | Breakaway | `B`/`b`, output **free**, 10 positions × 2 directions. Between presses rotate by hand and **let the rotor settle into a detent** (a rotor left mid-creep reads far low in the opposite direction). For A/B comparisons use the **same raw positions** as the reference session and compare pairwise (position sd ≈ 0.095 A). Compare breakaway with breakaway, never with `drag_c` | each reading: no `NO MOTION` (0.80 A), `travel` ≤ 200 counts, ≤ 0.60 A | 0.295 A, n = 20 (§22.6.4) |
| **B5** | INL | Phases 5 + 6. Read INL only; phase 6's `T/T_loop` is invalid belt-on | 1/rev, 2/rev vs belt-off | 1/rev −3.3%, even-part r = 0.9986 (2026-08-12) |
| **B6a** | Backlash | Key `w` (swing ladder), output clamped (2 × M3×10 countersunk into the top plate, perpendicular). Readout and rules: §22.4.4 | friction-corrected G ≤ 20 counts pass · 20–40 marginal · > 40 slack | G = 5.8 counts (§22.6.3) |
| **B6b** | Belt stiffness (ring) | Output clamped. `t` → `g` → `k` (+ step) or `K` (− step) → wait for `CAPTURE done` → `x` → `d`. **4 × `k` + 4 × `K`.** Fit f_d from `cnt` vs `t_us` (cycle count or damped sine), f_n = f_d/√(1−ζ²). Record the boot each burst came from (boot-to-boot scatter ~8× within-boot) and the ring amplitude (f depends on it, §22.3) | no band; fleet use is the J01-vs-J02 comparison below | f_d 65.0 Hz, f_n 69.8 Hz (§22.6.5) |
| **B7** | J_total | Output bare and **unclamped**. `t` → `g` → `k` → 1 s → `x` → `d`. J01 only (fleet quantity) | predicted 20.5–22 × 10⁻⁶ kg·m²; friction is 40–60% of the impulse, so ±20% | — |
| **B8** | Hot vs cold `R_eff` | Phase 3 cold → load hard 60 s → phase 3 | ΔR/R > 15% = peak force sags mid-jump | — |
| **B10** | Tooth-skip threshold | **Static lever at the output pulley + spring gauge.** Output held, known lever arm, pull until the belt skips. The motor cannot do it: 2 A gives 0.48 N·m at the output against a 3.4–5.7 N·m skip threshold. Do this last | the hard force ceiling; raises the 1.6 A_rep envelope (§22.7.15 6% rule) | **next** |
| **B11** | Store the belt-on row | `joint_cal.h`: `belt = "10mm-9:1"`, four drag fields, `breakaway_A`. Electrical fields carried from belt-off. Keep the belt-off row | — | done (§22.6.8) |
| **B12** | MIT impedance law | §22.7 | — | **CLOSED** 2026-10-03 |
| **B13** | Next joint | M1 → AUTOCALIB → M2 → M4 → B0–B6b | — | — |

Then: leg links on, M14 force-per-amp (load cell), Jacobian characterisation.

The contact offset ~12 mm below the foot pin must appear identically in the IK, the sim model
and foot-position logging.

### Fleet QC — which steps every joint runs

| Step | J01 | Every other joint |
|---|---|---|
| B1, B2, B3, B4, B6a, B6b | full | **full** (B6a/B6b are the tension QC) |
| B7 `J_total` | full | skip |

**J01-vs-J02 comparison** (decides whether the no-tensioner design holds; blocked on J02's
board, D15): B6b ring frequency within 15% · B6a both pass · B3 drag within 30% · B4 within
30%. Record both pulley recipes and re-run J01's ring in the same sitting (clamps are
per-assembly, ~±5% in f).

---

## 22.1 Belt-on plant states — J01, 2026-08-12

Belt 232 mm, C = 44.5 mm, output bare, leg off, one variable at a time. These plants no longer
exist (brass-post idlers replaced; ~1.04 mm slack with no idlers).

| Plant state | `drag_c` (direction mean) | Δ | Per leg | % of 9.81 N |
|---|---|---|---|---|
| Motor alone, belt off | **0.0783 A** | — | 0.353 N | 3.58% |
| + belt, no idlers | **0.2752 A** | +0.197 A | 1.242 N | 12.7% |
| + sliding brass posts | **0.6605 A** | +0.385 A | 2.982 N | 30.4% |
| A1 historical (rubbing magnet) | 1.05 A | — | 4.511 N | 46.0% (uncorrected) |

Newtons via `F = 2·G·η·Kt·I`, Ḡ 87.7 /m, η 0.92, corrected for M1 (×1.010768) and M2 (÷0.9621).
J02, same diagnostic: belt-off ~0.143 → no idlers 0.381 A, belt increment 0.238 A (+21% on J01's).

**Rules from these runs:**
- **Alignment is friction-biased.** ZEA +2.88° elec with the posts, +0.15° without. Keep the
  belt-off ZEA; do not re-align belt-on.
- **Phase 6 `T/T_loop` is belt-off only.** 0.958 → 1.042 → 1.291 as drag rises (≈ +0.57 per A).
  `T_DELAY_PER_LOOP = 0.958` is the belt-off value and stays.
- **Belt-on `Ke` is never carried.** 0.017750 → 0.017952 → 0.018089 (+1.9%) tracking drag.
- **Electrical constants are plant-independent:** `R_eff` −0.47%, `L` +2.1%, parity 0/4000.
- Forward/reverse asymmetry with the posts: Coulomb 32.9%, viscous 167.5%; without them 0.3% /
  9.8% (thread-walking of the posts).

---

## 22.2 Idler specification

```
IDLER, per side (x2):
  Roller       2x bearing 3 x 9 x 5 mm ZZ (measured 8.99 OD x 4.96 wide)
               -> 9.92 mm stack under a 10 mm belt
  Shaft        M3 partially-threaded socket screw with >= 10 mm PLAIN shank,
               or a 3 mm ground dowel + nut.  NO THREAD IN THE BEARING BORE
  End washers  conical M3, contacting the INNER race only.
               Never a flat 7 mm DIN 125 washer: it bridges onto the outer
               race and locks the roller
  Clamp path   through the inner races only (tube spacers 3.2 ID / 4.8 OD,
               spacers + bearings = plate gap)
  Mounting     FIXED M3 holes in BOTH plates at (1.72, +-10.00) mm, every joint
  Flanges      none. The idler does not track the belt; the output pulley does
```

**Acceptance (part of B0):** flick each roller — it must coast ≥ 1 s (if not, clamp load is
reaching an outer race). Pen-mark it, run phase 5: the mark rotated and the roller has not
walked axially.

Measured effect: sliding brass posts 0.385 A → rolling bearing idlers **0.048 A** (J02, §22.4.8).

Fixed holes because the idler is in double shear across two plates: two slots cannot be set
identically by hand, and a tilted axis edge-loads the bearings. Withdrawn, do not re-raise:
slotted idler with jackscrew/grub screw, printed spring cartridge, M2/M2.5 grub screws, tape on
the idler OD, the 230 mm (115T) belt (shorter than the 230.96 mm path).

---

## 22.3 Belt geometry and tension

### Belt path

```
L = 2*sqrt(C^2 - (r2-r1)^2) + r1*(pi - 2a) + r2*(pi + 2a),   a = asin((r2-r1)/C)
  r1 = 3.8197 mm (12T)   r2 = 34.3775 mm (108T)   C = 44.5 mm
  no-idler path = 230.96 mm   ->   slack on a 232 mm belt = 1.04 mm
```

CAD span 32.352 mm vs closed form 32.349 mm. Do not use the approximate formula
`2C + pi(r1+r2) + (r2-r1)^2/C`: this drive is at (r₂−r₁)/C = 0.687, far outside its range
(it gives 229.98 mm). The swing test confirmed the exact form (835 counts measured; 980
predicted exact, 1570 approximate).

Tension cannot be set by printing accurately: dT/d(intrusion) = 48–240 N/mm, so a ±0.5 mm belt
tolerance is 22–108 N. It is measured by consequence (B6a, B6b). The pluck method is retired
(idlers subdivide the spans; span ambiguity alone is a 5× tension range).

### Belt length — caliper jig

**Method.** Two printed ⌀10 mm shafts clipped to the caliper jaws, belt looped over both,
**1.475 kg** hung from the lower jaw (vertical, no hand pressure), read outside-to-outside.
ΔL = 2 × ΔR; shaft diameter, belt offset, stretch and angle errors cancel in the ranking.

**Protocol (next batch of 20):** record to 2 d.p.; randomise measurement order (PLA shafts creep
monotonically); re-read one reference belt at the start, every 5, and at the end; never merge
readings across shaft installs without the reference belt. Replace the shafts periodically.

**Results, n = 10 (2026-08-31):** 104.7 ×1, 104.8 ×3, 104.9 ×5, 105.0 ×1. Mean 104.86,
**σ(L) 0.158 mm, range 0.60 mm** of loop. At EA 20–50 kN that is 52–129 N of tension.

**Rule:** calibrate on the **longest** belt. Too long → slack, unseated teeth near zero torque
(hard to diagnose); too short → more drag (visible in `drag_c`). Fleet decision (2026-09-28):
holes fixed at (1.72, ±10.00); belt length is absorbed by pulley compensation bins.

### Idler geometry solver

Four-circle signed-radius path: pinion r₁ 3.8197 at origin, pulley r₂ 34.3775 at (44.5, 0),
idlers −4.881 (cord radius, back side) at (1.72, ±y). Increments are robust; absolute values
need a measured anchor.

| y (mm) | Take-up (mm) | Pinion wrap | Teeth in mesh |
|---|---|---|---|
| 9.0 | 2.275 | 165.1° | 5.50 |
| 9.5 | 1.695 | 149.2° | 4.97 |
| **10.0** | **1.238** | **137.6°** | **4.59** |
| 10.5 | 0.875 | 128.3° | 4.28 |
| 11.0 | 0.590 | 120.6° | 4.02 |
| 12.0 | 0.207 | 108.0° | 3.60 |

Local slope at y = 10: −0.78 mm/mm. Idler and pinion cord circles collide below y = 8.53.
dL/dr₂ = 4.655; dL/dC = 1.454 (no idlers) / 1.281 (y = 10). No idlers: 93.3° = 3.11 teeth.

**Belt-length sensitivity at y = 10** (pitch-seated pulley):

| Caliper reading | Loop (mm) | No-idler slack | Stretch @ y 10.0 |
|---|---|---|---|
| 105.00 (J02) | 232.00 | 1.04 | **0.20** |
| 104.90 | 231.80 | 0.84 | 0.40 |
| 104.86 (mean) | 231.74 | 0.78 | 0.46 |
| 104.80 | 231.60 | 0.64 | 0.60 |
| 104.70 (shortest) | 231.40 | 0.44 | 0.80 |

The batch range of 0.60 mm of loop = 0.77 mm of equivalent y. y = 10.00 is predicted to hold
every measured belt taut (prediction, D4). Tension in newtons is not stated: cord EA is
unmeasured (D8); at EA_eff 4–8.5 kN, 0.20 mm ≈ 3–7 N and 0.80 mm ≈ 14–29 N.

### Ring stiffness is amplitude-dependent

STD plant, 12 captures: **f_d = 86.9 − 0.522·A** (A = ring amplitude in counts, r = −0.982;
sd 4.86 → 0.91 Hz once removed). The drive is a softening spring (tooth engagement).

| Ring amplitude | f_d | k_motor |
|---|---|---|
| 54 counts | 59 Hz | 2.8 N·m/rad |
| 29 counts | 72 Hz | 4.2 N·m/rad |
| → 0 (extrapolated) | ≈ 87 Hz | 6.0 N·m/rad |

- No single stiffness number exists; always record ring amplitude with f.
- The ring measures cord + teeth + hub + fixture in series, so it cannot give cord EA.
- **Fixture:** k_fix ≥ 10 × k_belt (≈ 2,300–4,900 N·m/rad at the output). Torsional stiffness
  goes as the square of the reaction radius: react at the rim, not the 3 mm shaft. A glued rim
  bond passed (3 → 7 bond spots indistinguishable); the screwed fleet clamp has not had the
  add-stiffness test.
- Drivetrain f_n 59–72 Hz → Tier-2 impedance ceiling ~20–29 Hz (f_n/3), held pending D1.

---

## 22.4 J02 — SCB pulley acceptance (2026-09-05 → 2026-09-23)

Printed 108T grooves held the belt cord above pitch radius. Fixed with slicer X-Y contour
compensation −0.10 mm (pulley **SCB**) and accepted at the existing y = 10.00 holes with the
batch's longest belt. ⚠ SCB is a 0.6-profile (recipe A) print; any J02 reprint is recipe B and
needs re-acceptance (§22.6.6).

| Session | Pulley | Idlers | What ran |
|---|---|---|---|
| P0 | STD | — | swing 835, rings |
| P1 | A2 | — | swing 373, ring |
| P5 | SCB | none | ramped swing, ring |
| P6 | SCB | none | ladders |
| P7 | SCB | none | phase 5 |
| P8 | SCB | y = 10 | full acceptance |

### 22.4.1 J02 configuration

| Item | Value |
|---|---|
| Output pulley | 108T GT2, SCB: Arachne walls, slicer contour −0.10 mm (stored in the 3MF and filename), CAD boss +0.20 mm. Recipe A |
| Pulley mount | Screwed |
| Bearing seat | Printed 14.95 mm (A's 15.10). No shim. Owner reports a tight fit, no play (hand assessment) |
| Idlers | 2 × 3×9×5 ZZ per side, conical M3, fixed holes (1.72, ±10.00) |
| Belt | 116T GT2 10 mm, caliper 105.0 (longest of the batch), loop ≈ 232.00 mm |
| C | 44.5 mm |
| Pinion wrap at y = 10 | 137.6° = 4.59 teeth |
| Belt stretch at y = 10 | 0.20–0.37 mm |

J02 runs the longest belt, so its preload is the fleet minimum at this hole position.

### 22.4.2 Acceptance (P8)

| Criterion | Prediction | Measured | Verdict |
|---|---|---|---|
| B6a friction-corrected G | ≈ 0 | −6 to +13 counts (I_f 0.30–0.40 A, bracketed, not measured) | ✅ |
| Stepped ladder to 1.6 A (11.4 N) | clean | repeats −1 / −1 | ✅ |
| drag_c | ≤ 0.50 A (J02 projection) | **0.340 A** (0.306–0.400) | ✅ |
| Coulomb asymmetry | < 20% | 8.0% pooled | ✅ |
| Reverse drag fit rms | < 10 mA | 10.2 / 13.4 mA | ❌ marginal (D10) |
| Idler roller coast / pen mark | ≥ 1 s, rotates, no walk | not recorded | — |
| Ring f_n | — | 72.5 / 70.1 Hz | not a criterion (§22.4.7) |
| Belt teeth after the no-idler ratchet events | — | intact | ✅ |

### 22.4.3 Pulley seating — measurement

A seated belt pins its cord at r_pitch = 108 × 2.000 / 2π = **34.3775 mm**. Riding high costs
belt path at dL/dr₂ = 4.655 mm/mm (0.1 mm high uses 45% of the take-up). Printed tooth tips
come out 0.4–0.7 mm under the 68.25 mm nominal, so **tip OD is a geometry fingerprint, not the
path datum.**

| ID | Quantity | Procedure |
|---|---|---|
| **T** | Tip OD | Tooth tip to tooth tip across a diameter, n ≥ 3, middle value (±0.08 mm) |
| **M-B** | Over-belt OD | Belt fitted and hand-tensioned; caliper closed to **light contact only**; n = 3, middle value (±0.08 mm) |
| ~~M-A~~ | — | Caliper pressed into the belt back. Retired (0.34 mm between reading sets). Never compare M-A with M-B |

- Cord radius: `r_cord = OB/2 − 0.386` mm (belt land 0.640 − GT2 pitch-line offset 0.254).
- Path difference: `ΔS = (OB₁ − OB₂)/2 × 4.655`; pairwise uncertainty ±0.26 mm = ±180 counts.
- Idler cord radius = 4.495 + 0.386 = 4.881 mm.
- **Tip-OD fingerprint:** a −0.10 mm/side slicer offset must drop tip OD by ~0.20 mm (SC1/SCB:
  −0.26 / −0.23 ✅; Fusion FV1/FV2: +0.04 / −0.01, tips unmoved). Check it on every compensated
  print before trusting a belt reading. ⚠ All values in this section are 0.6-profile numbers;
  re-baseline on recipe B (D5).

**Variant register** (all 0.6 profile except J01-P12B):

| ID | Build | Tip OD | Over-belt (M-B) | r_cord | Status |
|---|---|---|---|---|---|
| A | Arachne, no compensation | 67.76 | 70.10 | 34.664 | reference |
| A2 | sibling of A, screwed | 67.67 | — | — | swing 373, ring 94–97 Hz; retired |
| SC1 | slicer −0.10, boss uncompensated | 67.50 | — | — | boss loose; superseded |
| **SCB** | slicer −0.10 + CAD boss +0.20 | 67.53 | 69.60 | 34.414 | **J02** |
| FV1 / FV2 | Fusion offset routes | 67.80 / 67.75 | 70.08 / 69.95 | — | rejected (tips unmoved) |
| STD | legacy standard walls, glued | 67.2–67.5 | — | — | retired |
| J01-P10 | slicer −0.10, standard walls | 67.55–67.58 | — | — | drag_c 0.414 A; replaced |
| J01-P12 | slicer −0.12 + precise wall, recipe A | — | — | — | G 31–48; superseded |
| **J01-P12B** | slicer −0.12 + precise wall, **recipe B**, boss CAD 15.26 → 14.95 | within 0.05 of P12 | — | — | **J01, pulley of record** |

**Results:**
- A2 → SCB no-idler swing 373 → 984 ± 3.5 counts (+0.895 mm path); caliper A → SCB +1.16 mm.
  Agree within ±180 counts.
- SCB cord sits at 34.39–34.41 mm (within ~0.04 mm of pitch), conditional on the 105.0 belt
  being 232.00 mm (soft by ±0.1–0.2 mm; 0.1 mm of belt = 0.021 mm of cord radius).
- Print scale: printer not undersize (A flange +0.07%, boss 0.00). Effective offset at nominal
  −0.10: −0.175 mm/side on convex cylinders, −0.115 on tips.
- Metal 12T pinion: over-belt 8.26 mm within the seated 8.12–8.41 range — seating defect is
  specific to the printed pulley.

### 22.4.4 Swing ladder — the B6a readout

```
swing(I) = G + s*(I - I_f)
  G    = slack + mesh lost motion            <- the deliverable
  s    = elastic compliance, counts per amp
  I_f  = static breakaway (B4 mean)
raw intercept = G - s*I_f   ->   always friction-correct: G = intercept + slope*I_f
```

**Procedure** — key `w` (acSwingLadder), entered via `e → f → w`:

1. **Setup.**
   - Output clamped: 2 × M3×10 countersunk into the top plate, perpendicular (fleet standard).
   - Run every ladder of a set **at one clamp setting**, output not moved. Record with each
     ladder whether the output was re-clamped since the last (a re-clamp moves slope/k ~10%,
     not the intercept; §22.5.4).
   - Record pulley ID, idler state, belt ID and bearing-seat state by hand (the banner's belt
     field does not carry the pulley variant).
2. **Points:** ±0.6, 1.0, 1.4, 1.6 A.
3. **Four legs per point:** `+cond` (discarded) → `−` → `+` → `−rpt`, 5 s dwell each (creep
   converges in ~4 s). `swing_a = + − m1`, `swing_b = + − m2`, `repeat = m2 − m1`.
4. **Skip detector:** |repeat| ≥ 0.5 tooth (1365.3 counts per tooth) discards the point and
   stops the ladder.
5. **Output:** `SW,<amps>,<minus1>,<plus>,<minus2>,<swing_a>,<swing_b>,<swing>,<repeat>`. Refit
   offline. ~80 s. Ignore `lps` on the first telemetry line after the ladder.

**Taking G:**
- G = intercept + slope × I_f per ladder; report mean and spread when > 1 ladder.
- **Single-ladder noise ±7 counts**, so a G within ~7 of 20 is not a clear pass or fail.
- I_f = this plant's B4 mean (`B`/`b`). State the value used. Compare joints only at matched I_f.
- Discard ladders run while the intercept was still climbing (new pulley bedding in).
- Residuals are convex: the drive softens with load; linear fits are conveniences.

**Scale:** 1 count = 1.4648 µm of belt at the pinion pitch line; 7.107 N belt force per
reported amp on J02 (`calKt` 0.027146 N·m/A_rep).

### 22.4.5 Swing results (SCB, belt 105.0)

**No idlers (P5, P6):**

| Current | Pooled swing | sd |
|---|---|---|
| 0.6 A | 765.2 | 3.3 |
| 1.0 A | 878.5 | 3.5 |
| 1.4 A | 1024 (v1 only; v2 crept) | 4.2 |
| 1.6 A | ratchet ~0.96 tooth | — |

Ramped ±2.0 A: 984 ± 3.5 counts. 2-point fit 0.6/1.0 A: slope 283 cnt/A, raw intercept 0.872 mm,
**G 0.996 mm** at I_f 0.2983 A (J02 belt-off breakaway, a proxy; +0.1 A of I_f = +28 counts).
Below the 1.042 mm fully-seated bound. k_beltline 30–34 kN/m.

**Idlers at y = 10 (P8):**

| Current | Run 1 | Run 2 | Pooled |
|---|---|---|---|
| 0.6 A | 52 | 59 | 55.5 |
| 1.0 A | 113 | 126 | 119.5 |
| 1.4 A | 189 | 202 | 195.5 |
| 1.6 A | 245 | 249 | 247 |

Pooled intercept −63.1, slope 189.1 cnt/A, k_beltline 51.3 kN/m. **G = −6 / +1 / +13 counts at
I_f 0.30 / 0.34 / 0.40 A** — slack and mesh lost motion ≈ 0. Chord slopes 160 → 190 → 258 cnt/A.

### 22.4.6 Idler position

The solver and sensitivity tables are in §22.3. J02's y = 10.00 was accepted by measurement.
Moving to 10.2 would cut this belt's stretch to 0.04 mm. Thin bedding-in headroom (0.20 mm) is
remedied with a mid-batch belt, not a hole move (D2).

### 22.4.7 Ring results by state

| State | Pulley / mount | Idlers | Frequency (Hz) | ζ | Amplitude | n | k_motor (N·m/rad) | k_beltline (kN/m) |
|---|---|---|---|---|---|---|---|---|
| P0 | STD, glued | none | f_d 59.6–72.0 | — | 29–54 counts | 12 | 2.8–4.2 | 190–290 |
| P1 | A2, screwed | none | f_d 93.6–97.1 | — | 5.6–8.5 | 3 | 7.46–7.76 | 510–530 |
| P5 | SCB, screwed | none | f_n 82.3 ± 0.4 | 0.39 | 11–12 overshoot | 4 | 5.40 | 370 |
| P8 | SCB, screwed | y = 10 | f_n 72.5 (+) / 70.1 (−) | 0.41 / 0.70 | 15 / 4 | 2 | 4.19 / 3.92 | 287 / 269 |

k_motor = (2πf)²·J_rotor; k_beltline = k_motor / r₁². The ring (small amplitude) reads 5–12×
stiffer than the ladder slope (large excursion). Tensioning lowered f_n 12–15% (cause untested).
Not carried into CONTROL §10 (D1).

### 22.4.8 Drag — phase 5 (SCB, belt 105.0)

| State | Run order | Fwd: Coulomb A + viscous mA/(rad/s) | Rev | Fit rms fwd / rev (mA) | Ke |
|---|---|---|---|---|---|
| No idlers (P7) | fwd first | 0.2773 + 1.948 | 0.2721 + 1.743 | 3.04 / 3.88 | 0.017867 |
| No idlers (P7) | rev first | 0.3192 + 1.599 | 0.2977 + 1.466 | 4.19 / 2.40 | 0.017870 |
| y = 10 (P8) | fwd first | 0.3200 + 3.082 | 0.4003 + 1.561 | 5.99 / 10.20 | 0.017883 |
| y = 10 (P8) | rev first | 0.3320 + 2.922 | 0.3060 + 2.535 | 3.97 / 13.37 | 0.017881 |

drag_c (mean of 4): no idlers **0.2916 A**, y = 10 **0.3396 A** (+0.048 A = the rolling idlers).
Pool both run orders (single-direction intercepts shift 0.04–0.08 A with order). Belt-on Ke is
not carried.

### 22.4.9 Skip and ratchet

| Plant | Procedure | Held clean up to | Event |
|---|---|---|---|
| STD, no idlers | ramped ±2.0 A | 13.9 N | — |
| SCB, no idlers | ramped ±2.0 A ×3 | 14.2 N | — |
| SCB, no idlers | stepped 1/2/3 A | 1.0 A (7.1 N) | 2.0 A: continuous ratchet |
| SCB, no idlers | stepped 4-leg | 1.0 A | 1.4 A: 60-count creep; 1.6 A (11.4 N): ratchet ~0.96 tooth |
| **SCB, y = 10** | stepped 4-leg ×2 | **1.6 A (11.4 N)** | — (2.0 A not attempted) |

With idlers (4.59 teeth in mesh) the skip threshold is unknown; design point is 213 N of belt
force at 30 A Iq. B10 is the instrument (D3). Read counts 5 s after a current limit (~10 counts
of creep converge first).

### 22.4.10 Pulley CAD route

Slicer contour compensation is global (it also shrinks the boss, flange and any register boss;
holes unaffected). Neither Fusion offset route (FV1 Offset Face, FV2 sketch offset) moved the
tips. **Used:** slicer route plus CAD boss parameter `slicer_contour_comp`.

**Fleet CAD route, if the slicer coupling is ever removed (D6):** roll the timeline back before
the tip fillet; Offset Face the groove faces **and** the OD cylinder by −`gt2_groove_offset`;
set tip fillet radius to `tip_r − offset`; confirm the pattern carries the offset. Accept only
if a print matches the recipe-B fingerprint (tip OD, M-B, hand skip). Do not increase the offset
beyond 0.12 without a failing seating result (each 0.1 mm/side costs a full extrusion of tooth).

### 22.4.11 Standing warnings

- **Slicer contour compensation is not in the model file.** Reslicing without it silently removes
  ~1.1 mm of slack. It is stored in the 3MF and the filename; the recipe is the G-code md5 (§22.6.1).
- A global contour offset loosens every outer-contour mating feature (register bosses → later
  output backlash).
- A loose bearing seat corrupts belt measurements: 0.10 mm of centre float ≈ 99 counts.

### 22.4.12 J02 session data

Bench supply ~12.5 V, `vbus_scale` 0.008516, `VBUS_LIVE` false. Banner vs UT89X −0.24 to −0.32%
every session (instrument resolution). P6–P8 were built from an uncommitted working tree; the
first commit containing the ladder is `b2a562e`.

- **Encoder counters** (`mt6816.h`): `perr` counts parity failures only (a lower bound on
  corrupted frames); `nmg` is a live flag written only on a successful read; `jrej` (angle-jump
  rejects) was inactive (`SPI_JUMP_GUARD` false). P8: +8 parity events, `nmg`/`ovs` 0 → clean.
- 2.7 M parity errors accumulated during handling between P7 and P8; the fault tracks handling,
  not rotation. Suspected SPI harness/connector or HVPP (pin 2, internal pulldown). After P8 the
  J02 ESC board failed (hot at idle, no serial). Unresolved; gates J02 re-acceptance.
- dq convention confirmed: |I|/Iq = 1.2250 vs √(3/2) = 1.2247.

---

## 22.5 J01 recipe A, −0.10 → −0.12 (2026-09-27 → 28) — superseded by §22.6

Recipe A was sliced on the 0.6 mm nozzle profile. Results kept for the findings that still apply.

| | −0.10 (P10) | −0.12 (P12) |
|---|---|---|
| drag_c, first run | 0.414 A | **0.291 A** (−30%) |
| B4 breakaway | — | 0.288 A, range 0.085–0.49 |
| B6b | — | f_d 64.3 ± 0.4 Hz, f_n ≈ 69.5 |
| B6a G | — | 31–48 counts (gate exception; closed by recipe B) |

- −0.02 mm/side of compensation cut drag ~30% on the 0.6 profile (unmeasured on recipe B).
- Ladder intercepts rose over the first three ladders (bedding-in), then held (L4–L6 mean −6.3,
  sd 7.4). Wear rejected.
- Belt-on, mean breakaway ≈ dynamic drag_c (no extra static stick from rest). Belt-off only,
  breakaway > drag_c.

### 22.5.4 The clamp moves stiffness, not backlash

| | L1–L3 | L4–L6 (after re-clamp) | Change |
|---|---|---|---|
| Slope (cnt/A) | 167.1 | 186.9 | +12% |
| k_beltline (kN/m) | 57.6 | 51.5 | −11% |
| Intercept | −4.0 | −6.3 | within ±7 noise |

k is recorded with its clamp. Clamps are per-assembly, so expect ~±10% in k (≈ ±5% in ring
frequency) from the clamp alone in cross-joint comparisons.

### 22.5.9 Breakaway structure (B4)

| Component | Formula, per rotor position | Value |
|---|---|---|
| Symmetric (friction) | (I₊ + I₋)/2 | 0.29 ± 0.07 A |
| Direction-biased | (I₊ − I₋)/2 | up to ±0.18 A |

The direction-biased part is cogging or position-dependent preload (moderate confidence; D16).
Breakaway correlates with release jump distance (r = 0.85, stick-slip). **Sim range (proposed):**
Coulomb randomised 0.2–0.4 A plus a position-dependent ripple of ±0.18 A.

---

## 22.6 J01 recipe B — pulley of record (2026-09-30)

Same −0.12 + precise-wall pulley, re-sliced on the correct 0.4 mm nozzle profile. Single-variable
A/B: same plate, belt (104.85) and idlers. Raw logs: `docs/cal/pulley acceptance/`.

### 22.6.1 Recipe of record — B

| Item | Value |
|---|---|
| Slicer | CrealityPrint V7.3.0.6149 |
| Printer profile | `Creality K2 Pro 0.4 nozzle` |
| Process | `0.20mm Standard @Creality K2 Pro 0.4 nozzle FOR PULLEY` |
| Filament | `BambuLab PLA Tough+ @Creality K2 Pro 0.4 nozzle` |
| Geometry | `nozzle_diameter = 0.4`, `xy_contour_compensation = −0.12`, `xy_hole_compensation = 0`, `precise_outer_wall = 1`, `precise_z_height = 1`, `wall_generator = arachne`, `wall_loops = 4`, `layer_height = 0.2` |
| Line widths | outer 0.40 / inner 0.48 / infill 0.42 / top 0.48 |
| Arachne thresholds | `min_bead_width = 85%` (0.34 mm), `min_feature_size = 25%` (0.10 mm) |
| Print rule | solo per plate |
| Boss | CAD 15.26, printed 14.95; snug on the bearing (−0.155 mm/side) |
| **Identity** | **md5 of the complete sliced `.gcode`**: `certutil -hashfile <file>.gcode MD5` (not yet recorded) |

**A recipe is its G-code md5, not its settings.** Arachne thresholds are percentages of
`nozzle_diameter`, so the same settings on a different machine profile give a different toolpath
(recipe A made inner beads up to 0.76 mm from a 0.4 mm nozzle). Re-slicing on another profile is
a new recipe and needs a new acceptance.

### 22.6.2 Plant and session

Idlers at (1.72, ±10.00), top plate on, no top screw, leg off. Ladder and ring clamped (fleet
standard); breakaway with output free. Kick TORQUE(V) 0.2 → 0.8 V. Banner vs UT89X +0.02 V both boots.

### 22.6.3 B6a swing ladder — PASS

| Ladder | Boot | Intercept (cnt) | Slope (cnt/A) | k_beltline (kN/m, true) | Max \|repeat\| | **G at I_f 0.295** |
|---|---|---|---|---|---|---|
| L1 | 1 | −39.7 | 152.4 | 65.6 | 4 | 5.3 |
| L2 | 2 | −40.1 | 148.3 | 67.5 | 2 | 3.7 |
| L3 | 2 | −34.5 | 145.8 | 68.6 | 2 | 8.5 |
| **Mean** | | −38.1 | 148.8 | **67.2** | | **5.8 (sd 2.5) = 0.014° output** |

Worst ladder across the I_f 95% CI (0.24–0.35 A): 16.5 counts. No bedding-in climb across a
reboot. vs recipe A at matched I_f: 48.8 → 5.8 counts. k values are true belt-line k (firmware
prints before 2026-10-01 were × `i_scale`, 0.9621 lower).

### 22.6.4 B4 breakaway — PASS

| | Mean | SEM | Range |
|---|---|---|---|
| All (n = 20) | **0.295 A** | 0.027 | 0.080–0.475 A |
| + direction | 0.314 A | 0.039 | |
| − direction | 0.277 A | 0.038 | |

Position dominates (pair means 0.12–0.42 A, sd 0.095). One reading had travel −240 counts
(> 200, biased high); without it the mean is 0.286. Output torque at 0.295 A_rep = **0.0743 N·m**
(`irepToTorqueOut`, η excluded). vs recipe A: 0.295 vs 0.288, no difference resolvable at n = 20.

### 22.6.5 B6b ring — PASS

| | Value |
|---|---|
| f_d, all 8 | **65.0 Hz** (SEM 1.0) |
| f_d, + kicks / − kicks | 67.6 Hz (A₀ ≈ 18 counts) / 62.4 Hz (A₀ ≈ 28) |
| f_n | 69.8 Hz |
| ζ | 0.36 (+0.31 / −0.41) |
| Amplitude slope | −0.47 ± 0.08 Hz/count (matches §22.3's −0.522) |
| vs recipe A 64.3 Hz | Δ +0.7 Hz (gate ≤ 1 Hz) |

### 22.6.6 B3 drag on recipe B

| Run | Order | Fwd: Coulomb A + viscous mA/(rad/s), rms | Rev | Second − first |
|---|---|---|---|---|
| 1 | fwd first | 0.2265 + 4.009, 7.7 mA | 0.3576 + 2.597, 9.0 mA | +0.131 A |
| 2 | rev first | 0.4072 + 2.466, 13.9 mA | 0.3084 + 2.730, 9.7 mA | +0.099 A |
| **Pooled** | | **0.3169 + 3.24** | **0.3330 + 2.66** | asymmetry 5.0% |

- **Run-order effect:** the direction run second reads 0.10–0.13 A higher; not monotonic warm-up
  (D18). Pooling cancels it, so `joint_cal.h` stores each direction pooled over both orders.
- No drag change attributable to the recipe is resolvable (inside the ±20% plant spread).
- Belt-on breakaway (0.295) is not larger than drag_c (0.292 first run / 0.325 pooled).
- Ke 0.018064 / 0.018070 (+0.7% on belt-off): not carried. Session tripwires: `V` 0.62°, phase 1
  4000/4000, phase 3 R 0.22404 Ω.

**Phase 4 is belt-off only.** Belt-on, its 0.8 ↔ 3.2 A step train ratchets the self-lock
(drift −169 counts, WARN). Phase 3 is immune (drift 0). L stays carried from belt-off.

**Every pulley before J01-P12B was sliced on the 0.6 profile**, including J02's SCB. On the 0.6
profile, −0.12 cost backlash (31–48 counts) where −0.10 did not; on the 0.4 profile −0.12 gives
~6. J02 must be reprinted on recipe B and re-accepted before D15.

### 22.6.7 Consequences

- B6a gate stays ≤ 20 counts; recipe B meets it with ~14 to spare. D17 demoted.
- **Clamp standard:** 2 × M3×10 countersunk into the top plate, perpendicular. No shared jig (the
  holes belong to each top-plate assembly).
- Fleet pulley compensation bins are not settled: needs the 20-belt histogram (§22.3 protocol)
  and one short-belt joint on recipe B (D4).

### 22.6.8 B11 record — J01 belt-on row (recipe B)

In `joint_cal.h`, 2026-09-30. The belt-off baseline row stays at index 13.

| Field | Value |
|---|---|
| date / belt | `"2026-09-30"` / `"10mm-9:1"` |
| zea, dir, R_eff, U0, Ke, L, vbus_scale, i_scale | carried from belt-off |
| drag_c fwd, rev | **0.3169, 0.3330 A** |
| drag_v fwd, rev | **3.24e-3, 2.66e-3 A/(rad/s)** |
| breakaway_A | **0.295 A** |

Not in the row: G 5.8 counts; k 67.2 kN/m (at its clamp); f_d 65.0 / f_n 69.8 Hz.

---

## 22.7 B12 — MIT law (J01) — CLOSED 2026-10-03

Plant: J01, recipe B, belt on. B12a: output bare and free. b1: output clamped. Banner
`Kt_cmd=0.027972`, `1 A_rep = 0.2517 N.m` output. Envelope: τ_max 0.39 N·m (outer), 1.6 A_rep
(inner, compile-time).

### 22.7.1 Results

| Step | Proves | Result |
|---|---|---|
| a0 | Speed filter | `Tf_mit` = 1.0 ms |
| a1 | Clamp chain + validator, disarmed | PASS 22/22 |
| a2 | Signs, clamps, Uq headroom, loop rate | PASS; loop rate −4.7% vs TORQUE(I) (gate ≤ 5%); auto-stop 20 s works |
| a3 | kd is a damper; v_des drives | PASS (§22.7.9, §22.7.11); D19 promoted; O2 logged |
| a4 | Step timing vs kp | PASS on aim (units end to end); kp 6.6 gate missed, explained by O4 (§22.7.13) |
| a5 | Saturation on a moving motor | PASS, both clamps (§22.7.15) |
| b1 | Clamped output, gain envelope | PASS (§22.7.16) |

**"+" direction = counter-clockwise viewed facing the output-pulley shaft end** (the face pointing
outward on the robot).

### 22.7.2 a0 — speed filter

Six TORQUE(V) captures at ~15–17 rad/s motor. Velocity noise (motor rad/s) vs filter:

| Tf (ms) | Measured noise | Lag at 25 Hz |
|---|---|---|
| 0.5 | 0.226 | 4.5° |
| 0.7 | 0.167 | 6.3° |
| **1.0** | **0.120** | **8.9°** |
| 1.5 | 0.082 | 13.3° |
| 2.0 | 0.062 | 17.4° |

At kd 0.365: noise torque 0.120/9 × 0.365 = 4.9 mN·m = 70% of the 7 mN·m budget. **1.0 ms.**

### 22.7.3 a1 — clamp chain

0.10 N·m → 0.3972 A_rep; outer ±1.0 → ±1.5492; outer 5.0 → inner ±1.6000; NaN / ±inf / τ_max ≤ 0
/ env 0 → 0, REJECT. Validator rejects negative kp, negative/NaN kd, kp 1e6, p_des 10, v_des inf,
τ_ff 2.

### 22.7.4 a2 — directions and safety

Four τ_ff pulses ±0.15 N·m, 60 ms, auto-revert; kp = kd = 0.

| Cap | Pulse | Peak v output / motor (rad/s) | Peak excursion | Roll-back after stop | Peak \|Uq\| | Clamp |
|---|---|---|---|---|---|---|
| 1 | +0.15 | 1.84 / 16.6 | +0.079 rad | 6.1 mrad | 0.44 V | 0 |
| 2 | −0.15 | 3.15 / 28.3 | −0.198 | 32.2 mrad | 0.64 V | 0 |
| 3 | +0.15 | 2.76 / 24.8 | +0.146 | 8.9 mrad | 0.58 V | 0 |
| 4 | −0.15 | 2.82 / 25.4 | −0.161 | 22.9 mrad | 0.60 V | 0 |

Signs correct on all repeats; Iq reaches 90% in ~1.5 ms. Friction backed out of the pulses:
mean 0.31 A_rep (vs drag_c 0.317 / 0.333). **O1:** after each pulse the joint rolls back opposite
to its motion at zero torque (6–32 mrad); narrowed by O4, not closed.

### 22.7.6 MIT command reference

| Typed | Meaning |
|---|---|
| `m …;` | MIT command line; ends on Enter or `;` |
| `m;` | Enter MIT mode (stopped), or print MIT status |
| **A** | The **live** command, used every loop while armed. `m kd 0.1;` etc. edit it immediately |
| **B** | **Staged changes** applied to A by `m go` at a known capture sample (B = A + changes, resolved when `m go` fires) |
| `m b vd 2;` | Stage changes. Each `m b` line replaces the previous one; fields not named come from A at `m go`. Survives `g` |
| `m b;` | Clear staged changes (B = A, `m go` only records) |
| `!! vd does nothing while kd = 0` | Printed when a target is multiplied by a zero gain (τ = kp(pd − p) + kd(vd − v) + ff); same for pd with kp = 0 |
| fields | `pd` angle (rad, output, relative to the arm point) · `vd` speed (rad/s, output) · `kp` N·m/rad · `kd` N·m·s/rad · `ff` N·m |
| `m go R D;` | Start a capture; after 50 samples B replaces A. **R** = auto-revert to zero torque R ms after the switch (0 = never). **D** = record every D-th loop: at ~11.9 kHz, D 1 ≈ 84 ms, 2 ≈ 179 ms, 4 ≈ 350 ms, 8 ≈ 0.68 s, 32 ≈ 2.74 s per 1000 samples |
| `m tmax X;` | Raise τ_max (≤ 1.0); reset to 0.39 on every arm. Above the 1.6 A_rep envelope prints `!! … INNER clamp binds first` |
| `g` / `x` / `d` | Arm (p = 0 here; A's pd/vd/ff cleared, kp/kd and staged changes kept; prints A and what `m go` will apply) / stop / dump. Dump **after** `x` (blocks ~1 s). pd/vd/ff typed while stopped are cleared by `g` |

The console is silent while a capture records. When B holds a vd, the motor keeps moving after
`CAPTURE done` — press `x` promptly.

### 22.7.7 Loop rate

| | Before (prefetch 0) | After (prefetch 1 + MIT trim) |
|---|---|---|
| TORQUE(I) armed | ~12,440 lps (80.4 µs) | 12,471–12,491 (80.1 µs) |
| TORQUE(I) disarmed | 66,300 | 65,528–65,607 |
| MIT armed | 11,420–11,600 (86.9 µs) | 11,867–11,956 (84.1 µs) |
| MIT disarmed | 54,790 | 54,830–54,993 |
| **MIT vs TORQUE(I), armed** | −8.2% | **−4.7%** (gate ≤ 5%) |
| MIT mean cost, armed / disarmed | 6.5 / 3.2 µs | 3.95 / 2.9 µs |

- Flash prefetch had no effect; the gain came from the MIT trim (`micros()` → DWT cycle counter,
  per-loop measured τ removed). `FLASH_PREFETCH` stays `true` (harmless).
- `svc_us` is a max including interrupt preemption (~2× the mean); the mean cost is the lps
  difference.
- Disarmed `mitService()` costs ~500 cycles for ~150 by count. Leading hypothesis: branch-target
  misses at 8 flash wait states (prefetch only hides sequential misses). Test: `FLASH_LATENCY_4`
  (in spec at 170 MHz) — predict TORQUE(I) armed lps ≥ +5%. Deferred (§22.8).

### 22.7.9 a3 part 1 — damper by hand: PASS

kp 0, kd 0.1, two captures at decim 32. Velocity differentiated offline from `p`.

| Capture | Moving samples | v range (rad/s output) | Slope through 0 | R² |
|---|---|---|---|---|
| cap 2 | 416 | −1.17 … 0 | −0.1000 | 0.984 |
| cap 3 | 533 | −1.43 … +1.52 | −0.0998 | 0.999 |
| **Pooled** | 949 | | **−0.0998** | **0.998** |

Clamp 0, |Uq| ≤ 0.10 V. Measured τ tracks command to 2.8 mN·m RMS. This checks law, estimator,
sign and current loop, not absolute torque (same Kt both sides).

### 22.7.10 `m b` semantics

A first a3 part-2 attempt commanded zero torque: B was then a complete second command, zeroed by a
reboot, so `m b vd 2` gave vd 2 with kd 0. Fixed in `open_test.cpp`: B is now staged changes
resolved against A at `m go`, re-validated there, with the `!!` zero-gain warning (§22.7.6).

### 22.7.11 a3 part 2 — driven speed: PASS; D19 promoted

kp 0, kd 0.1, B = A + vd ±2, decim 8, three runs per direction.

| Run | v_des | Mean v (output rad/s) | sd | Mean τ (N·m) | τ − τ_cmd RMS | Uq max |
|---|---|---|---|---|---|---|
| 1 | +2 | +1.290 | 0.127 | +0.071 | 3.3 mN·m | 0.31 V |
| 2 | +2 | +1.242 | 0.147 | +0.076 | 3.2 | 0.31 |
| 3 | +2 | +0.838 | 0.187 | +0.116 | 3.1 | 0.29 |
| 4 | −2 | −1.315 | 0.135 | −0.068 | 3.2 | 0.32 |
| 5 | −2 | −1.005 | 0.198 | −0.100 | 3.0 | 0.30 |
| 6 | −2 | −0.966 | 0.177 | −0.103 | 3.1 | 0.30 |

All inside 0.8–1.8 rad/s; clamp 0. Mean of three +1.123 / −1.095 vs drag-map prediction
+1.120 / −1.096 (v = 2 − τ_drag(v)/kd).

**D19 — 14/turn ripple torque**, fitted against absolute rotor angle, three runs per direction:

| Order (per motor turn) | + ripple torque (mN·m output, phase) | − ripple torque |
|---|---|---|
| **14** | **44.4 @ −7°** | **48.2 @ −11°** |
| 28 | 9.4 @ +77° | 9.5 @ +69° |
| 12 (pinion teeth) | 7.1 @ −109° | 9.7 @ −109° |
| 7 (electrical) | 5.2 @ +62° | 6.3 @ +95° |
| 1 | 4.7 | 9.7 |

- Locked to rotor angle (six runs within ±5°); same phase both directions (conservative, not
  friction modulation); amplitude independent of speed (not encoder error).
- 4.9–5.4 mN·m at the motor. The J that makes order 14 direction-independent is 20.2e-6, the
  stored value.
- ~39% of it shows in measured τ (the kd term reacting to speed ripple).

**O2 — friction depends on output angle** (suggested, not established): steady friction falls in
two groups, 0.068–0.076 and 0.100–0.116 N·m, split by output angle (slow runs inside 1.7–4.35 rad).
About ±0.022 N·m once per output turn around 0.089; explains the drag map's ±20% scatter.
Decisive test: one constant-speed run over ≥ 1.5 output turns with a marked output index.

### 22.7.13 a4 — step timing vs kp: PASS on aim

Steps from rest, re-armed each time, judged on the mean stop time (switch → first stop).

| Row | kp / kd | Step | Mean stop time (pred) | Δ | Gate | Rest error (bound) | Uq max | Clamp |
|---|---|---|---|---|---|---|---|---|
| 1 | 41 / 0.157 | ±5.7 mrad | **19.5 ms** (19.2) | +1.6% | ±10% ✅ | ≤ 1.79 mrad (±4.0) ✅ | 0.29 V | 0 |
| 2 | 16.8 / 0.100 | ±14 mrad | **28.0 ms** (31.0) | −9.8% | ±10% ✅ | ≤ 5.01 (±9.8) ✅ | 0.28 V | 0 |
| 3 | 6.6 / 0.063 | ±35 mrad | **61.3 ms** (50.6) | +21% | ±15% ❌ (n = 4) | ≤ 4.42 (±25) ✅ | 0.31 V | 0 |

Units are excluded as the cause: a unit error scales every row by the same factor; the misses
(+2%, −10%, +21%) follow step size in washboard periods. Re-run from the measured start counts
with no fitted parameters, the model gives 17.7 / 28.7 / 57.7 ms (within 10% of every row).

**O4 (established) — the joint rests in one 14/turn detent phase.** Hand-turned starts land at
268–292 counts mod 1170.3 (seven of seven in row 1); releases after up steps 282–296, after down
steps 128–176. Detent ≈ 218 counts, stick band ±70. Consequences:
- Static friction ≈ 0.03–0.05 N·m and very-low-speed sliding ~0.02 N·m — the same size as the
  washboard (≤ ~0.055 N·m), far below 0.068–0.116 N·m at 0.8–1.3 rad/s.
- A joint commanded within ~±70 counts (3 mrad output) of a detent edge with kp·error below
  ~0.05 N·m stalls on the washboard; at zero torque it rolls into the detent. This is D19's job.
- Not every release lands in the band (a5: 694, 271).

**O3 — roll-back after the peak while still pushing forward** (0.25–2.51 mrad): belt wind-up
≤ 0.5 mrad (b1 stiffness) explains row 1; rows 2–3 need the washboard. Closed as explained.

### 22.7.15 a5 — saturation on a moving motor: PASS; B12a CLOSED

kp 41, kd 0.157, step ±28.5 mrad (3 × the saturation error).

| cap | step | clamp code / ms | max τ_cmd | peak | v_peak (output) | rest error | Uq max | `cl=` |
|---|---|---|---|---|---|---|---|---|
| 1 | up | 1 only / 12.5 | 0.3900 | 115.4% | 2.07 | +0.38 mrad | 0.65 | 140/0/0 |
| 2 | up | 1 only / 11.1 | 0.3900 | 115.0% | 2.44 | +1.40 | 0.69 | 124/0/0 |
| 3 | down | 1 only / 11.5 | 0.3900 | 118.7% | 2.22 | −1.21 | 0.66 | 132/0/0 |
| 4 | down | 1 only / 10.1 | 0.3900 | 119.5% | 2.53 | +1.34 | 0.72 | 115/0/0 |
| 5 | up, `tmax 0.6` | 2 only / 2 ms pulse | 0.4028 | — | — | — | — | 0/22/0 |

Inner clamp (cap 5) fired cleanly on a 2 ms pulse (typo `m go 2`); not rerun (same code path,
gates nothing further). No wind-up, no limit cycle.

- Plateau τ 0.366–0.373 vs 0.390 (−5%): Iq sags as back-EMF ramps; first check is back-EMF
  feed-forward in the current loop if a torque shortfall appears in fast moves.
- **Measured Iq overshoots the clamped command by up to 6%** (1.69 vs 1.600 A_rep). **Rule: when
  the envelope is raised after B10, set it ≥ 6% below the demonstrated-safe current.**

### 22.7.16 b1 — clamped output, gain envelope: PASS; B12 CLOSED

Output clamped as in B6b, current mode, held `ff` step, decim 1.

| cap | kp | kd | ff | deflection (cnt / mrad) | belt secant K (N·m/rad) | overshoot | ζ (overshoot) | ring f |
|---|---|---|---|---|---|---|---|---|
| 1–2 | 0 | 0 | 0.200 | 33.0 / 1.41, 29.1 / 1.24 | 142, 161 | 21%, 27% | 0.44, 0.38 | 66, 72 Hz |
| 3–4 | 41 | 0 | 0.226 | 25.8 / 1.10, 25.7 / 1.09 | 163, 165 | 35%, 36% | 0.31 | 68 Hz |
| 5–6 | 41 | 0.157 | 0.226 | 24.9 / 1.06, 24.4 / 1.04 | 172, 175 | 24% | 0.41 | (~80) |
| 7–8 | 41 | 0.250 | 0.226 | 24.0 / 1.02 | 180 | 17% | 0.49–0.50 | (~80) |
| 9–10 | 41 | 0.365 | 0.226 | 23.7 / 1.01 | 182 | 9–10% | 0.60 | — |

Every ring decays, no buzz, clamp 0. ζ rises at every kd rung. Belt secant K = belt torque at rest
÷ deflection. ζ from the decrement at kp 0: 0.20–0.23.

- **The belt is a softening series spring:** secant 142–182 N·m/rad at 25–33 counts vs small-signal
  280–335. Output stiffness = kp·K/(kp+K): at kp 41, **80–89% of commanded** (60–77% at kp 100).
  Static stiffness ff/p: 206 at kp 41 vs 152 at kp 0.
- kd buys ~0.75 ζ per N·m·s/rad on this mode.
- **kd injects encoder-quantisation torque at rest: kd × 0.039 N·m** (6 / 10 / 14 mN·m measured).
  At kd 2.0 (`MIT_RANGES` ceiling) that is 78 mN·m. Plant-independent.
- Measured τ ripples ±6% around command during the ring; peak current +4.5% over command.

**Sim / RL ranges (J01, fleet-level until the leg says otherwise):** belt mode **55–72 Hz**
(output clamped, current mode); **K_belt 140–335 N·m/rad output, softening**; natural **ζ
0.20–0.45**; demonstrated gain box **kp ≤ 41 N·m/rad, kd ≤ 0.365 N·m·s/rad** (ζ ≈ 0.6 at the top).
`MIT_RANGES` (kp 200, kd 2.0) is a validator ceiling, not demonstrated.

**Next:** B10 skip threshold → 7l homing / boot pose → J03 → leg.

---

## 22.8 Deferred items

| # | Item | Promote when |
|---|---|---|
| **D19** | **Active.** 14/turn ripple (44–48 mN·m output): characterise per joint (amplitude and phase vs electrical angle) including an **at-rest quasi-static torque–angle sweep** (p_des ramp at kp ~41 over ≥ 1170 counts, both directions → washboard shape and static friction); evaluate a Tier-0 feed-forward. Contract-freeze question: does Tier 0 cancel it, and does τ include it? | Per joint at bring-up |
| D1 | Tier-2 impedance ceiling from the ring | Before any CONTROL §10 ceiling update; needs a second joint and > 1 boot |
| D2 | B6a ladder repeat in a later session (recipe B G = 5.8 holds?) | Next J01 session. If G > 40: mid-batch belt or compensation change (holes are fixed) |
| D3 | Skip threshold on the tensioned plant (B10) | Before any command above 1.6 A (11 N belt force) and before leg attachment |
| D4 | Fleet compensation bins: confirm the 20-belt batch maximum ≤ 105.0; one joint on a short belt (≤ 104.7) on recipe B, drag_c in band and G ≤ 20 | 20-belt batch measurement / joint 3 |
| D5 | Fleet pulley repeatability: two prints of the same md5 (tip OD, M-B); re-baseline the fingerprint on recipe B | Before printing any fleet pulley |
| D6 | Fleet CAD route (§22.4.10) | Only if the slicer-setting coupling is to be removed |
| D7 | Belt-difference swing, 105.0 vs 104.7, predicted 410 counts | Before selective assembly relies on the caliper jig |
| D8 | Cord EA two-weight test (1.475 / 5 kg with a steel-wire splay reference) | Only if tension in newtons becomes load-bearing |
| D9 | J02 B4 breakaway on its tensioned plant | With J02's replacement board |
| D10 | Reverse drag fit rms 10–14 mA | Inspect the drag map for per-idler or per-output-rev terms |
| D11 | "1/rev disturbance gone" (hand feel only) | Confirm h1 in the next drag map |
| D12 | J02 reprint on recipe B (boss from recipe-B behaviour) and re-acceptance | When J02's board is replaced; before D15 |
| D13 | Chalk witness test for tooth contact | Only if a future pulley fails seating |
| D14 | Stepped vs ramped loading, no-idler ratchet | Only if the no-idler swing is used as a QC step again |
| D15 | B6b J01-vs-J02 15% comparison (4 + 4 in `t` mode, plus B4) | When J02 has a working board and recipe-B pulley |
| D16 | Cogging vs position-dependent preload in B4's ±0.18 A | If the sim friction model needs the ripple's period/phase (dense angle sweep) |
| D17 | Control-side backlash tolerance test (kp steps in position hold, watch for limit cycle, ~10 min) | If any fleet joint fails G ≤ 20 |
| D18 | Drag run-order effect (+0.10–0.13 A second direction) | If a drag feed-forward needs better than ±20%. Test: one direction twice with a rest vs back-to-back |
| O1/O2 | Roll-back at zero torque; friction vs output angle | When a friction feed-forward is proposed, or rest errors exceed bounds |
| — | Flash wait states 8 → 4 (§22.7.7) | Tier-0 loop with CAN measured below its need, or a loop-rate gap fails a gate |
| — | kp/kd to a stability limit | Gait/RL needs kp > 41 or kd > 0.365; run it **on the leg** |
| — | b2/b3 inertia bar | Leg rings or misses the √(1 + J_L/J_m) bandwidth prediction |
| — | Back-EMF feed-forward in the current loop (−5% plateau sag) | b1 or the leg shows a torque shortfall in fast moves |
| — | Shim the top-plate clearance (wear debris, not drag) | Next teardown |
| — | Does the M4 screw axially preload the pinion bearing? Define a shim stack or screw depth | Next teardown; decides whether pinion drag is fleet or per-unit |
| — | Recipe B G-code md5 | Next pulley print |
