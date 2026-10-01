# Arm Design Considerations

Ongoing design discussions and tradeoffs for the scoop arm control system. Decisions made here feed into implementation on the `feature/arm-control` branch.

---

## Position Tracking Across Power Cycles

**Question:** How does the arm know where it is after a reboot?

### Option A: Persist position to flash

Write the current step count to STM32 internal flash on every scoop (or periodically). On boot, read it back and resume from that position.

**Pros:**
- No movement on boot — arm stays where it was
- Instant ready state

**Cons:**
- If the arm is physically moved while powered off (bumped, manual adjustment), the saved position is **wrong** and the firmware doesn't know
- STM32L4 flash has limited write endurance (~10K cycles per page) — frequent writes degrade it
- Adds flash driver complexity (erase-before-write, page management)
- Silent failure mode: stale data looks identical to good data

### Option B: Home on boot (retract until HOME switch triggers)

On startup, slowly retract the arm until the HOME limit switch triggers, then zero the step counter. Always starts from a known-good position.

**Pros:**
- Always correct regardless of what happened while powered off
- No flash wear
- Simpler code — reuses existing limit switch + stepper infrastructure
- Fail-safe: if switch doesn't trigger, hits ARM_CAL_MAX_STEPS and stops
- Retracting is the safe direction (arm pulls inward)

**Cons:**
- Arm moves on every boot (takes a few seconds)
- If the arm is already home, it still briefly presses against the switch (minimal concern — the switch is NC so it just reads triggered immediately and stops)
- Requires limit switch to be functional — but it's already wired and NC (fail-safe)

### Option C: Hybrid — persist + verify with home switch

Save position to flash, but on boot, verify by checking if the HOME switch state matches the saved position. If mismatch, re-home.

**Pros:**
- Fast boot when position is correct
- Self-corrects when position is wrong

**Cons:**
- Most complex implementation
- Still has flash wear concerns
- Only catches errors at the home position — if the arm is mid-travel and saved position is wrong, the switch check won't help

### Decision

**TBD** — discussing.

---

## Calibration Safety

**Decided: 2026-04-01**

During calibration (`C\n` command):
- B1 button = latched emergency stop (requires MCU reset to clear)
- Wrong-direction limit switch = immediate stop + error message
- Expected limit switch = normal stop, records step count
- Max step limit (10,000) = safety cutoff if no switch triggers

Step count from calibration is hardcoded into `scoop_sequence.h`. Limit switches are passive safety backup during normal scoop operation — not load-bearing.

**Rationale:** User doesn't want to depend on switches working every scoop. Calibrate once (supervised), hardcode the result, switches are belt-and-suspenders.

---

## Servo-as-Function-of-Stepper

**Decided: 2026-04-01**

During retract (Phase 4), the servo angle interpolates linearly from CLOSED to DUMP position based on stepper progress. Updated every 30 steps to avoid flooding the servo UART.

```
servo_pos = CLOSED + ((DUMP - CLOSED) * steps_done) / total_steps
```

**Open questions:**
- [ ] Is linear interpolation the right profile? Might want to hold closed longer at the start of retraction, then dump faster at the end. Would need to test on hardware.
- [ ] 30-step update interval — is this too frequent or too sparse? Depends on total step count and servo response time.
- [ ] Should the servo use time-based moves (HX65_MovePosition with time_ms > 0) or instant moves (time_ms = 0) during interpolation?

---

## Limit Switch Wiring: NC vs NO

**Decided: 2026-04-01**

NC (normally closed) to GND with internal pull-up.

**Rationale:** Fail-safe. If a wire breaks or comes loose, the pin floats HIGH (pulled up), which reads as "triggered." This stops motion rather than allowing a runaway arm. NO wiring would fail silently — a broken wire would never trigger, defeating the safety purpose.

---

## Deploy Sequence (Startup Safety Gate)

**Question:** How does the robot transition from powered-on to operational?

### Problem

If the arm homes on boot automatically, something could be in the way. The robot shouldn't move anything until a human deliberately activates it.

### Proposed Boot Flow

```
Power on → Jetson boots → ROS2 starts → UI comes up
    ↓
STANDBY mode (everything inert, no motors, no arm movement)
    ↓
Operator presses DEPLOY (web UI button and/or B1 on Nucleo)
    ↓
Arm homes (retract to HOME switch, zero position)
    ↓
ACTIVE mode (teleop/autonomy enabled, scoop ready)
```

### Open Questions

**What does STANDBY block?**
- ~~Option 1: Only arm/scoop — drive motors still work in standby~~
- **DECIDED: Option 2 — Everything blocked.** No motor commands accepted until deployed. Robot is completely inert. Reason: arm could be dragging on the ground if extended.

**What happens if homing fails during deploy?**
- HOME switch doesn't trigger (broken wire, obstruction, max steps reached)
- **DECIDED: Option A** — Go back to STANDBY with error displayed in UI. Operator can retry or investigate. No power cycle needed.

**Can the operator re-enter STANDBY?**
- **DECIDED: Yes.** Useful for maintenance, clearing a jam, swapping something. Re-deploy requires another press-and-hold + re-home.

**Where does deploy state live?**
- STM32 needs to know (reject motor commands in STANDBY)
- Jetson/UI needs to know (show correct buttons, block commands)
- New UART command needed — format TBD
- STM32 reports state back so Jetson can query on connect

**Deploy trigger:**
- **DECIDED: Press-and-hold 5 seconds in web UI** with loading bar that fills during hold. Release cancels. Prevents accidental activation.
- No physical button — UI only.

**Deploy sequence is scalable:**
- Designed as a checklist of deploy steps, not a single action
- Arm homing is just step 1. Future steps: sensor checks, self-test, camera verification, etc.
- Each step reports success/failure to the UI

### Decisions Summary

| Topic | Decision | Date |
|-------|----------|------|
| Standby scope | Blocks everything (motors + arm + scoop) | 2026-04-01 |
| E-stop | Works in all states, never gated | 2026-04-01 |
| STM32 state on Jetson reboot | Survives — Jetson queries on connect | 2026-04-01 |
| Position tracking | Home on boot via deploy (Option B) | 2026-04-01 |
| Deploy trigger | Press-and-hold 5 sec in web UI with loading bar | 2026-04-01 |
| Re-deploy | Allowed — return to standby from UI | 2026-04-01 |
| Deploy architecture | Scalable checklist of steps | 2026-04-01 |
| Homing failure | Return to standby with error, retry allowed | 2026-04-01 |

### Still Open

- ~~What happens to the arm when re-entering STANDBY mid-travel?~~ **DECIDED: Stop arm, begin homing retract.**
- Deploy command format over UART — leaning toward multi-word commands (see UART Command Safety below)
- Should the deploy checklist be configurable or hardcoded?
- Dev mode toggle: UART command (`DEV ON\n`/`DEV OFF\n`) vs UI-only (Jetson gates everything)?

---

## Dev Mode

**Question:** How to bypass the deploy gate during development?

### Decision (2026-04-01)

- Toggle in the web UI — dev mode ON skips the deploy requirement
- Dev mode OFF (default) = full deploy gate, everything blocked until press-and-hold deploy
- Dev mode ON = all commands pass through immediately, no deploy needed
- UI should make it visually obvious when dev mode is on (colored banner, warning text)

**Open:** Does the STM32 need to know about dev mode, or does the Jetson handle all gating?
- If Jetson gates: simpler STM32, but if Jetson crashes mid-operation the STM32 has no safety gate
- If STM32 gates: more robust, but need a UART command to toggle dev mode
- Leaning toward: **STM32 handles the gate** (safer — doesn't depend on Jetson being healthy)

---

## UART Command Safety

**Question:** Single-letter commands are vulnerable to noise-triggered activation.

### Current Commands and Risk

| Command | Format | Risk Level |
|---------|--------|------------|
| `M` | `M <fl> <fr> <rl> <rr>\n` | Low — requires 4 params |
| `X` | `X\n` | None — e-stop is always safe to trigger accidentally |
| `S` | `S\n` | **High** — single byte triggers blocking scoop |
| `C` | `C\n` | **High** — single byte starts calibration |
| `T` | `T\n` | Low — just prints debug output |

### Proposed Changes

| Old | New | Rationale |
|-----|-----|-----------|
| `S\n` | `S SCOOP\n` | Prevent noise-triggered scoop |
| `C\n` | `C CALIBRATE\n` | Prevent noise-triggered calibration |
| `D\n` | `D DEPLOY\n` | Prevent noise-triggered deploy |
| `X\n` | `X\n` (keep) | E-stop should be easy to trigger |
| `M ...` | `M ...` (keep) | Already requires params |
| `T\n` | `T\n` (keep) | Harmless debug output |

**Status:** Discussing.

---

## Re-entering STANDBY

**Decided: 2026-04-01**

If the operator returns to STANDBY while the arm is mid-scoop or mid-travel:
1. Arm stops immediately (current step aborted)
2. Arm begins homing retract (slow retract to HOME switch)
3. Once home, system is in STANDBY

If homing fails during the return-to-standby retract, system stays in STANDBY with error displayed. Arm is stopped wherever it is — operator must investigate.

### Things to Consider

- ~~If we block everything in standby, how do we test drive motors without deploying the arm? Could be annoying during development. Maybe standby only blocks the arm, not driving.~~ **DECIDED: Block everything.** If the arm is extended/dragging, driving would grind it into the ground. Safety wins over dev convenience — can always add a dev-mode bypass later if needed.
- If we allow re-deploy from the UI, we need to handle the case where the arm is mid-travel when STANDBY is re-entered. Does it stop in place? Retract first? Just freeze?
- B1 dual-purpose (deploy vs e-stop based on state) is clean but could confuse the operator — "I pressed B1 and nothing happened" because they thought they were in a different state. A dedicated deploy source (UI only?) might be safer.
- The deploy command needs to be something the Jetson sends intentionally, not something that could be triggered by UART noise. Single character `D\n` is fine but maybe require a confirmation like `D CONFIRM\n` to avoid accidental deploy.
- E-stop (`X\n`) should work in ALL states — standby, active, calibration. E-stop is never gated.
- If homing fails and we return to standby, the UI should show WHY it failed (B1 abort, wrong switch, max steps) so the operator knows what to fix.
- Should there be a timeout on standby? Robot sitting powered on for hours not deployed — does it matter? Probably not, but worth noting.
- What about the scoop sequence — if the arm is mid-scoop and the robot loses Jetson connection, the scoop finishes (it's blocking on STM32) and then the motor watchdog kicks in. Is that the right behavior? Or should loss-of-connection during a scoop abort the scoop?
- The deploy state should survive a Jetson reboot but NOT an STM32 reboot. If only the Jetson restarts, the STM32 is still active and the arm is already homed. The Jetson should query STM32 state on connect rather than assuming standby.

---

## Future Considerations

_Add new topics here as they come up during discussion._
