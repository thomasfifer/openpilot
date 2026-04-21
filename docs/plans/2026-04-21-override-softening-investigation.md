# Volvo CMA — driver-override softening investigation

**Date:** 2026-04-21
**Platform:** Volvo XC40 Recharge (CMA). Findings likely apply to SPA / Polestar 2 with minor signal differences.
**Outcome:** First-pass fix: cap openpilot's commanded steering angle to `±6°` of actual wheel angle. See "First-pass fix" below.

*Route IDs and dongle IDs referenced in this document are kept in `docs/plans/.private/` (gitignored). Placeholders below (`<stock-pothole-route>`, etc.) map to entries there.*

---

## Problem statement

On the Volvo CMA port, steering-wheel override while openpilot is engaged requires much more driver force than override while stock Pilot Assist (PA / LCA) is engaged. Subjectively reported by the lead porter and reproducible across multiple drives. Goal of this investigation: understand the mechanism, then pick a fix that doesn't regress lane-tracking quality.

---

## Method

Three logged drives were compared:

| Tag | Mode | Scenario |
|---|---|---|
| `<stock-pothole-route>` | stock Pilot Assist engaged, openpilot latActive=False | driver swerves left to avoid a pothole (transient, ~1-2 s) |
| `<stock-sustained-route>` | stock Pilot Assist engaged, openpilot latActive=False | driver holds wheel biased-left, staying in-lane, ~20+ s |
| `<op-override-route>`    | openpilot lateral active | driver pushes wheel left hard for ~2 s |

Analysis pipeline (scripts in `/tmp/volvo_analysis/`, not committed; easily reconstructable from this doc):

1. `tools.lib.logreader.LogReader` to stream CAN and openpilot service messages.
2. `opendbc.can.CANParser` with `volvo_mid_1` DBC per physical bus.
3. Extracted signals: `LCA.LCA_STEER`, `LCA.LCA_RATE_OF_CHANGE`, `LCA.LCA_STEER_LOOSELY(_INV)`, `LCA_5.LCA_5_STEER`, `LCA_2.PILOT_ASSIST_ENGAGED`, `LCA_4.LCA_ENABLE`, `PSCM.PSCM_ANGLE_SENSOR`, `PSCM.DRIVER_INPUT_DEVIATION`, `PSCM.HANDS_ON_STEERING_WHEEL_A/B`, `DRIVER_INPUT.STEERING_DRIVER_INPUT`, `DRIVER_INPUT.STEERING_DRIVER_RATE_OF_CHANGE`.
4. Dense 100 Hz aligned time-series tables around the override windows.
5. Blind byte-level scan across all `(bus, addr, byte)` present in the stock route, to surface signals not captured in the RE'd DBC.

---

## Bus topology (important for interpreting results)

- **Bus 0 (main):** LCA camera / vision module side. Panda forwards openpilot's CAN commands here. PSCM sees this (or bus 2 — see below).
- **Bus 1 (powertrain):** engine, transmission, camera telematics. **PSCM does not read bus 1.** Any signals we find on bus 1 are diagnostic outputs; they cannot influence EPS behavior directly.
- **Bus 2 (party):** PSCM / EPS side. `DRIVER_INPUT` and `PSCM` messages live here.
- `src ≥ 128` in the CAN log = panda's own transmission echo (`src = 128 + bus_tx`). Not a separate bus.

Two addresses (`0x092`, `0x460`) appear on **both** bus 0 and bus 1 carrying different payloads — same CAN ID, different messages. Fingerprinting hazard.

---

## Findings

### 1. Stock Pilot Assist has *two distinct* override behaviors

**Transient (pothole swerve, ~200 ms driver jerk):**
- Commanded angle (`LCA_5_STEER`) collapses onto actual wheel angle within ~300 ms.
- Peak `|cmd − actual|` = 14.7° briefly at the swerve peak, then decays to 0.3° within 1.6 s *while the driver continues to apply peak torque*.
- Stock effectively **yields**: its commanded angle tracks wherever the driver has put the wheel.

Observed (100 ms table, stock, pothole at t≈358 s):

| t | actual° | cmd° | err° | drv_tq |
|---|---|---|---|---|
| 357.5 | +3.8 | −0.7 | −4.5 | −11 |
| 358.0 | +4.5 | −10.2 | **−14.7** | −5 |
| 358.3 | −0.6 | −13.2 | −12.6 | 0 |
| 359.0 | −4.4 | −8.9 | −4.5 | −3 |
| 359.5 | −2.9 | −3.7 | −0.9 | −9 |
| 359.6 | −2.7 | −3.0 | **−0.3** | −12 |

**Sustained (biased hold on straight road, 20+ s):**
- Commanded angle does **not** collapse. Stock fights back with a sustained offset.
- Peak `|cmd − actual|` = 12° transient; decays to steady-state 5–8° held for 5+ s.
- Driver torque needed to maintain a modest deviation: ~5–7 Nm sustained.

Observed (100 ms table, stock HOLD-2 at t≈240 s):

| t | actual° | cmd° | err° | drv_tq |
|---|---|---|---|---|
| 239.8 | +2.4 | +0.8 | −1.6 | −6 |
| 240.2 | +7.8 | −2.2 | −10.1 | −7.5 |
| 240.6 | +3.5 | −8.4 | **−11.9** | −4.5 |
| 241.0 | −0.2 | −8.9 | −8.7 | −4 |
| 243.0 | +1.5 | −5.0 | −6.5 | −7 |
| 245.0 | +1.3 | −3.9 | −5.3 | −7 |
| 246.0 | +1.6 | −4.9 | −6.5 | −7 |

### 2. openpilot has neither behavior

Under the same override-attempt shape, openpilot's lateral planner keeps demanding the "correct" angle regardless of driver resistance. The commanded angle **grows** in the direction the driver is pushing against, amplifying the EPS fight:

| t | actual° | cmd° | err° | drv_tq |
|---|---|---|---|---|
| 76.6 | −0.9 | −0.9 | 0.0 | +10 |
| 77.0 | −3.7 | +1.1 | +4.8 | +13 |
| 77.5 | −4.9 | +8.9 | +13.8 | +12 |
| 77.8 | +0.3 | +12.5 | +12.2 | +10 |
| 78.0 | +2.7 | +12.7 | +10.0 | +10 |

Commanded angle grew from ~0° to +13° over 1 second while the driver pushed the wheel to −5°. `|cmd − actual|` ~14° sustained for ~1 s until driver released. This is planner-side wind-up.

### 3. Quantitative driver-torque vs. cmd-actual gap

At steady state during sustained override, driver torque needed to hold the wheel scales approximately linearly with `|cmd − actual|`:

| | steady gap | driver torque |
|---|---|---|
| stock HOLD-2 (t=241–246) | ~6° | ~7 Nm |
| openpilot (t=77.5–78.0) | ~12° | ~13 Nm |

**Ratio ≈ 1 Nm per 1° of gap.** Doubling the gap doubles the driver effort. This points directly to: *bound the gap → bound the effort*.

### 4. `LCA_RATE_OF_CHANGE` is not a softening knob (initial hypothesis was wrong)

Observed values on bus 0:

| | `LCA_RATE_OF_CHANGE` |
|---|---|
| stock inactive (baseline) | 34 |
| stock sustained override | 69–132 (median ~112) |
| openpilot active | pinned at 80 regardless |

Initially hypothesised that higher values → softer EPS response. **Incorrect.** Per the porter's confirmation, this signal dictates how fast the EPS aims to reach the commanded angle — higher = *faster* convergence = *more* assist torque = *harder* to override. Stock runs it higher during override, which would make stock harder, not easier.

So the difference in override feel is **not** explained by this signal. It's explained by the gap in commanded angle (see finding 3).

### 5. Bus-1 diagnostic signals (interesting but not actionable)

Blind byte-level scan surfaced several bus-1 messages whose bytes are dormant during calm driving and activate during driver override or lane departure. Not currently in `volvo_mid_1.dbc`.

| Signal | Activation | Behavior |
|---|---|---|
| `0x092 byte 1 bits 3,4` | transient override onset | flips `CB → D3`, does not activate during sustained hold |
| `0x092 byte 5` | override onset | counts up from 0 as override deviation accumulates |
| `0x053 byte 2` | override onset | redundant copy of `0x092 byte 5` integrator |
| `0x13C byte 4 bit 0` | lane departure | flips `C0 → C1` and stays set for ~4 s of departure |
| `0x13C byte 5` | lane departure | ramps 0 → 0xE6 departure-deviation integrator |
| `0x037 byte 0 bit 0 + byte 1` | LKA haptic | sporadic pulses at ~800 ms cadence for ~2.2 s — **this is the vibration trigger**, not a raw waveform |
| `0x460 bytes 1/3/5` (bus 0) | lane departure | slow 1 Hz status bytes that change during departure |

**Key property:** these signals fire when stock LCA is the one driving. When openpilot replaces stock LCA's command stream, the detectors stay idle — the stock camera module no longer "feels" it's steering, so its override detector doesn't trigger. Useful as offline analysis markers. Not useful as real-time override detection inputs while openpilot is active.

Also: **PSCM does not read bus 1** (confirmed by vehicle wiring), so these signals *cannot* be the mechanism by which stock softens. Stock's yielding must happen on the command side (bus 0 → PSCM).

### 6. `0x037` is the LKA haptic — a flag, not a waveform

The steering-wheel vibration during lane-departure warning is commanded by `0x037`'s sporadic pulses (upper-nibble changes like `0x80 00 → 0x81 2A / 0xFF B8 / 0x80 40`). The PSCM expands the flag into physical torque ripple. To reproduce this under openpilot would require either keeping the stock LKA substate alive or synthesising these pulses from openpilot.

---

## What does NOT work as a fix

### Using `CS.out.steeringPressed` as the override gate
`steeringPressed` is set by `|STEERING_DRIVER_INPUT| > 2`, which trips on a light hand rest. Baseline driver torque on this platform with the driver's hand resting is 0–6 Nm (mean ~4). Any fix gated on this flag would soften during normal hands-on driving. `steeringPressed` is a hands-on proxy for DM timers, not an override indicator.

### Modulating `LCA_RATE_OF_CHANGE` downward during override
Refuted by finding 4. Wrong direction of effect.

### Blending commanded angle toward actual on driver wheel-rate spikes (Mechanism 2)
Would mirror stock's transient-yield behavior. Correct idea in principle. **Deferred** because a correct implementation requires: (a) persistent state across frames for the low-pass, (b) hysteresis on the rate trigger (`STEERING_DRIVER_RATE_OF_CHANGE` is noisy, std 10–20 at calm driving, up to 21 at baseline — a single threshold would chatter), (c) a graceful release ramp when the trigger deactivates. Non-trivial. Saved for a follow-up patch if the simpler cap (below) proves insufficient on the road.

---

## First-pass fix: cap commanded angle to ±6° of actual wheel

### Change
In `opendbc_repo/opendbc/car/volvo/carcontroller.py`, immediately after `apply_angle = actuators.steeringAngleDeg`, clip:

```python
apply_angle = float(np.clip(
    apply_angle,
    CS.out.steeringAngleDeg - CarControllerParams.MAX_ERR_DEG,
    CS.out.steeringAngleDeg + CarControllerParams.MAX_ERR_DEG,
))
```

And add to `CarControllerParams` in `values.py`:

```python
MAX_ERR_DEG = 6.0  # max |commanded − actual| steering angle in degrees
```

### Why it works

- During **normal driving** (including sharp curves like highway exits): the EPS tracks the planner's output. Observed steady-state `|cmd − actual|` is typically < 3°. The 6° cap has headroom and does not activate. No interference with lane tracking.
- During **sustained override**: the driver holds the wheel; actual can't advance; the gap tries to grow. The cap stops it at 6°. Per finding 3, driver torque needed is proportional to the gap, so capping the gap caps the driver effort at ~6 Nm — matching stock's steady-state feel.
- During **transient override** (pothole): the cap also helps (peak gap can't exceed 6°), but the behavior still differs from stock's full yield. That's what Mechanism 2 would add later.

### Placement rationale (carcontroller.py)

The cap goes *after* `apply_angle = actuators.steeringAngleDeg` and *before* the `if not CC.latActive: apply_angle = CS.out.steeringAngleDeg` line. This way:
- When `latActive=False`, the `CS.out.steeringAngleDeg` assignment overrides the cap (cap irrelevant — as it should be, since inactive already sends current angle).
- `live_testing` overrides still work unconstrained for standstill testing.

### Tuning knobs

| `MAX_ERR_DEG` | feel |
|---|---|
| 6 | matches stock's sustained-hold steady state |
| 9 | slightly stiffer than stock, still much softer than current openpilot |
| 12 | matches stock's transient peak, stiffer than stock steady-state, much softer than current |

Starting value: **6.0°** (match stock). Tune on-road if needed.

---

## Future work

1. **Mechanism 2 (transient yield):** low-pass blend of commanded angle toward actual when `STEERING_DRIVER_RATE_OF_CHANGE` exceeds a hysteretic threshold, with graceful release ramp. Needs persistent state, Schmitt-trigger on the rate signal, and a fade-out path when the trigger clears.
2. **Expose `STEERING_DRIVER_RATE_OF_CHANGE` on CarState** so Mechanism 2 (and future override logic) can use it without rooting around in `CS.msg_*` dicts.
3. **LKA haptic replication under openpilot:** decode the bit-level encoding of `0x037` (the pulse message) and either keep the stock module's LKA substate alive or synthesise the pulses from openpilot when lane-departure without-blinker is detected.
4. **DBC additions:** add the bus-1 diagnostic messages (`0x092`, `0x053`, `0x13C`, `0x037`, `0x170`) to `volvo_mid_1.dbc` once bit semantics are finalised. Beware that some addresses (`0x092`, `0x460`) carry different payloads on bus 0 vs. bus 1.
5. **Stock-vs-openpilot telemetry dashboard:** the analysis scripts compute `|cmd − actual|` and driver torque over time; a live on-device plot would make tuning much faster than offline log analysis.

---

## Reproducing the analysis

Minimal steps:

```python
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser

# Route IDs: see docs/plans/.private/
lr = LogReader("<dongle>|<route>/<segment>")

# Extract steering signals
parsers = {bus: CANParser("volvo_mid_1", [...], bus) for bus in (0, 1, 2)}
for evt in lr:
    if evt.which() == "can":
        frames = [(m.address, bytes(m.dat), m.src) for m in evt.can]
        for bus, parser in parsers.items():
            parser.update([evt.logMonoTime, frames])
            # read parser.vl[msg_name][sig_name]
```

For the blind byte-level diff, iterate all `(bus, addr, byte)` pairs, compute per-byte bit-toggle count and unique-value count in a calm window vs an override window, and rank by "static-in-calm, active-in-override". This surfaces unmapped flags like the ones in finding 5.
