# UI Deploy Overlay Redesign

**Date:** 2026-04-02
**Branch:** feature/arm-control
**Author:** benji

---

## Summary

Replace the full-screen standby overlay with a pane-level overlay scoped to the teleop control panel. Add a LiDAR status dot placeholder in the status bar. Reduce deploy hold time from 5s to 3s. Create CHANGELOG.md and update all CLAUDE.md files to reference it.

---

## Behavior Matrix

| Robot State | Dev Mode | Teleop Pane |
|-------------|----------|-------------|
| STANDBY     | OFF      | Overlay visible, keys blocked |
| STANDBY     | ON       | Overlay hidden, keys accessible |
| DEPLOYING   | OFF      | Overlay visible (shows deploying state), keys blocked |
| DEPLOYING   | ON       | Overlay hidden, keys accessible |
| ACTIVE      | OFF      | Overlay hidden, keys accessible |
| ACTIVE      | ON       | Overlay hidden, keys accessible |

SPACE bar (e-stop) is always accessible regardless of overlay state — it is handled by the keyboard listener, not blocked by the overlay.

---

## Changes

### index.html

1. **Remove** `#standby-overlay` (the `position: fixed; inset: 0` full-screen div and all its CSS).

2. **Add** `position: relative` to `#teleop-panel`.

3. **Add** `#teleop-deploy-overlay` — an absolutely positioned div inside `#teleop-panel`:
   - `position: absolute; inset: 0; z-index: 10`
   - Background: `rgba(10, 10, 10, 0.92)` — opaque enough to clearly block the controls behind it
   - Centered column layout (flexbox)
   - Contains: title ("ROBOT IN STANDBY"), subtitle ("Hold DEPLOY for 3 seconds"), deploy button (same press-and-hold logic), deploy status text
   - `display: none` when hidden

4. **Change** `DEPLOY_HOLD_MS` from `5000` to `3000`.

5. **Update** `deployStart()` status text: "Hold for 5 seconds..." → "Hold for 3 seconds..."

6. **Update** `#standby-subtitle` text: "Press and hold DEPLOY for 5 seconds to activate" → "Hold DEPLOY for 3 seconds to activate"

7. **Update** `applyStateUI()`:
   - Reference `#teleop-deploy-overlay` instead of `#standby-overlay`
   - Hide overlay when `robotState === 'ACTIVE' || devMode`
   - Show overlay when `robotState === 'DEPLOYING'` (deploying state within pane)
   - Show overlay when `robotState === 'STANDBY' && !devMode`

8. **Add LiDAR status dot** to `#statusbar`:
   - New `<div class="status-item">` with `dot-lidar` and label "LiDAR"
   - Positioned after Camera dot
   - Managed by `updateDots()` — grey (`null`) until `/scan` topic is wired up
   - Add `lastScanTime = 0` variable for future use (same pattern as `lastOdomTime`)

### CHANGELOG.md (new file, repo root)

Create `CHANGELOG.md` with an `[Unreleased]` section capturing this session's changes and a note on the convention.

### CLAUDE.md (root)

Add to the **Documentation Structure** section:
- Reference `CHANGELOG.md` as the session log
- Instruction: add a CHANGELOG entry when merging to main; use `[Unreleased]` for in-progress work

### ros2/CLAUDE.md

Update the "Current State" table to reflect the ui_server changes (pane overlay, dev mode gating, 3s deploy).

---

## What Does NOT Change

- `ui_server.py` — no Python changes in this session. LiDAR dot is a UI placeholder only; `/scan` subscription comes when hardware is connected.
- STM32 firmware — not touched.
- WebSocket message protocol — no new message types.
- E-stop behavior — SPACE always works.
- Dev mode toggle in topbar — no change.

---

## File Scope

| File | Change |
|------|--------|
| `ros2/litterbot_bridge/litterbot_bridge/ui/index.html` | Main UI changes |
| `CHANGELOG.md` | New file |
| `CLAUDE.md` | Doc convention update |
| `ros2/CLAUDE.md` | Status table update |
