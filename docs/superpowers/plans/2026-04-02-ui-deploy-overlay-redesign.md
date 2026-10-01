# UI Deploy Overlay Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the full-screen standby overlay with a pane-level overlay scoped to the teleop panel, reduce deploy hold time to 3s, add a LiDAR status dot, and create CHANGELOG.md with updated doc conventions.

**Architecture:** All UI logic lives in a single self-contained `index.html`. The pane-level overlay sits inside `#teleop-panel` using absolute positioning, so it covers only the control buttons. `applyStateUI()` shows/hides it based on `robotState` and `devMode`. No backend changes.

**Tech Stack:** Vanilla HTML/CSS/JS. No build step. No test framework for the UI — verification is manual browser inspection after launching the ROS2 stack.

---

## File Map

| File | Action | What changes |
|------|--------|-------------|
| `ros2/litterbot_bridge/litterbot_bridge/ui/index.html` | Modify | Deploy overlay, LiDAR dot, hold time, JS logic |
| `CHANGELOG.md` | Create | Session log, branching convention |
| `CLAUDE.md` | Modify | Add CHANGELOG reference to doc structure section |
| `ros2/CLAUDE.md` | Modify | Update ui_server status table entry |

---

## How to test manually

All verification steps assume you are in WSL2 with the ROS2 stack running:

```bash
cd ~/litterbot_ws && source install/setup.bash
ros2 launch litterbot_bridge litterbot.launch.py
# Open browser: http://localhost:8765
```

To kill the stack between test runs:
```bash
pkill -f "ros2 launch"; pkill -f "ui_server"; pkill -f "stm32_bridge"
```

---

## Task 1: Reduce deploy hold time and update subtitle text

**Files:**
- Modify: `ros2/litterbot_bridge/litterbot_bridge/ui/index.html`

This is the simplest change — do it first so the JS constant is correct before we restructure the overlay.

- [ ] **Step 1: Change DEPLOY_HOLD_MS**

Find line 611:
```js
const DEPLOY_HOLD_MS = 5000;
```
Change to:
```js
const DEPLOY_HOLD_MS = 3000;
```

- [ ] **Step 2: Update deployStart() status text**

Find in `deployStart()` (around line 907):
```js
status.textContent = 'Hold for 5 seconds...';
```
Change to:
```js
status.textContent = 'Hold for 3 seconds...';
```

- [ ] **Step 3: Commit**

```bash
git add ros2/litterbot_bridge/litterbot_bridge/ui/index.html
git commit -m "ui: reduce deploy hold time from 5s to 3s"
```

---

## Task 2: Replace full-screen overlay with pane-level overlay

**Files:**
- Modify: `ros2/litterbot_bridge/litterbot_bridge/ui/index.html`

This is the main structural change. We remove the fixed full-screen `#standby-overlay` and replace it with `#teleop-deploy-overlay` inside `#teleop-panel`.

- [ ] **Step 1: Remove full-screen overlay CSS**

Find and delete these CSS blocks (lines ~379–439):
```css
/* ---- Standby overlay ---- */
#standby-overlay {
  position: fixed;
  inset: 0;
  background: rgba(0,0,0,0.85);
  z-index: 1000;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 24px;
}
#standby-overlay.hidden { display: none; }

#standby-title {
  font-size: 22px;
  font-weight: 700;
  color: #ef5350;
  letter-spacing: 0.08em;
  text-transform: uppercase;
}
#standby-subtitle {
  font-size: 13px;
  color: #888;
  margin-top: -12px;
}

#deploy-btn {
  position: relative;
  width: 220px;
  height: 52px;
  border: 2px solid #66bb6a;
  border-radius: 8px;
  background: #1a1a1a;
  color: #66bb6a;
  font-size: 15px;
  font-weight: 700;
  letter-spacing: 0.06em;
  cursor: pointer;
  overflow: hidden;
  user-select: none;
  transition: border-color 0.2s;
}
#deploy-btn:hover { border-color: #81c784; }
#deploy-btn.deploying { border-color: #ffa726; color: #ffa726; cursor: wait; }

#deploy-progress {
  position: absolute;
  left: 0; bottom: 0;
  height: 4px;
  width: 0%;
  background: #66bb6a;
  transition: width 0.05s linear;
  border-radius: 0 0 6px 6px;
}
#deploy-btn.deploying #deploy-progress { background: #ffa726; }

#deploy-status {
  font-size: 12px;
  color: #666;
  min-height: 18px;
}
```

- [ ] **Step 2: Add pane-level overlay CSS**

In the `</style>` area, after the `#teleop-panel` block, add:

```css
/* ---- Pane-level deploy overlay ---- */
#teleop-deploy-overlay {
  position: absolute;
  inset: 0;
  z-index: 10;
  background: rgba(10, 10, 10, 0.92);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 16px;
}
#teleop-deploy-overlay.hidden { display: none; }

#deploy-title {
  font-size: 18px;
  font-weight: 700;
  color: #ef5350;
  letter-spacing: 0.08em;
  text-transform: uppercase;
}
#deploy-subtitle {
  font-size: 12px;
  color: #888;
  margin-top: -8px;
}

#deploy-btn {
  position: relative;
  width: 200px;
  height: 48px;
  border: 2px solid #66bb6a;
  border-radius: 8px;
  background: #1a1a1a;
  color: #66bb6a;
  font-size: 14px;
  font-weight: 700;
  letter-spacing: 0.06em;
  cursor: pointer;
  overflow: hidden;
  user-select: none;
  transition: border-color 0.2s;
}
#deploy-btn:hover { border-color: #81c784; }
#deploy-btn.deploying { border-color: #ffa726; color: #ffa726; cursor: wait; }

#deploy-progress {
  position: absolute;
  left: 0; bottom: 0;
  height: 4px;
  width: 0%;
  background: #66bb6a;
  transition: width 0.05s linear;
  border-radius: 0 0 6px 6px;
}
#deploy-btn.deploying #deploy-progress { background: #ffa726; }

#deploy-status {
  font-size: 11px;
  color: #666;
  min-height: 16px;
}
```

- [ ] **Step 3: Add `position: relative` to `#teleop-panel` CSS**

Find:
```css
#teleop-panel {
  flex: 1;
  padding: 12px 16px;
  background: #0d0d0d;
  overflow-y: auto;
}
```
Change to:
```css
#teleop-panel {
  flex: 1;
  padding: 12px 16px;
  background: #0d0d0d;
  overflow-y: auto;
  position: relative;
}
```

- [ ] **Step 4: Remove the full-screen overlay HTML**

Find and delete this block (lines ~494–505):
```html
<!-- Standby overlay — blocks everything until deploy completes -->
<div id="standby-overlay">
  <div id="standby-title">ROBOT IN STANDBY</div>
  <div id="standby-subtitle">Press and hold DEPLOY for 5 seconds to activate</div>
  <button id="deploy-btn"
    onmousedown="deployStart()" onmouseup="deployCancel()" onmouseleave="deployCancel()"
    ontouchstart="deployStart()" ontouchend="deployCancel()" ontouchcancel="deployCancel()">
    DEPLOY
    <div id="deploy-progress"></div>
  </button>
  <div id="deploy-status"></div>
</div>
```

- [ ] **Step 5: Add pane-level overlay HTML inside `#teleop-panel`**

Find the opening tag of `#teleop-panel`:
```html
<div id="teleop-panel">
  <h3>Keyboard Teleop</h3>
```
Change to:
```html
<div id="teleop-panel">
  <div id="teleop-deploy-overlay">
    <div id="deploy-title">ROBOT IN STANDBY</div>
    <div id="deploy-subtitle">Hold DEPLOY for 3 seconds to activate</div>
    <button id="deploy-btn"
      onmousedown="deployStart()" onmouseup="deployCancel()" onmouseleave="deployCancel()"
      ontouchstart="deployStart()" ontouchend="deployCancel()" ontouchcancel="deployCancel()">
      DEPLOY
      <div id="deploy-progress"></div>
    </button>
    <div id="deploy-status"></div>
  </div>
  <h3>Keyboard Teleop</h3>
```

- [ ] **Step 6: Update applyStateUI() to reference the new overlay**

Find the existing `applyStateUI()` function (around line 954) and replace it entirely with:

```js
function applyStateUI() {
  const overlay = document.getElementById('teleop-deploy-overlay');
  const btn = document.getElementById('deploy-btn');
  const status = document.getElementById('deploy-status');
  const banner = document.getElementById('dev-banner');
  const devBtn = document.getElementById('dev-toggle');
  const stateLbl = document.getElementById('lbl-state');

  // Dev mode banner
  if (devMode) {
    banner.classList.add('active');
    devBtn.classList.add('active');
  } else {
    banner.classList.remove('active');
    devBtn.classList.remove('active');
  }

  // State label
  if (stateLbl) stateLbl.textContent = robotState + (devMode ? ' DEV' : '');

  // State dot
  if (robotState === 'ACTIVE') {
    setDot('dot-state', 'green');
  } else if (robotState === 'DEPLOYING') {
    setDot('dot-state', 'orange');
  } else {
    setDot('dot-state', 'red');
  }

  // Pane overlay visibility
  if (robotState === 'ACTIVE' || devMode) {
    overlay.classList.add('hidden');
  } else if (robotState === 'DEPLOYING') {
    overlay.classList.remove('hidden');
    btn.classList.add('deploying');
    btn.textContent = 'DEPLOYING...';
    const prog = document.createElement('div');
    prog.id = 'deploy-progress';
    prog.style.width = '100%';
    btn.appendChild(prog);
    status.textContent = 'Homing arm — please wait...';
  } else {
    // STANDBY
    overlay.classList.remove('hidden');
    btn.classList.remove('deploying');
    btn.innerHTML = 'DEPLOY<div id="deploy-progress"></div>';
    status.textContent = '';
  }
}
```

- [ ] **Step 7: Verify manually**

Launch the stack and open `http://localhost:8765`.

Check each scenario:

| Scenario | How to trigger | Expected |
|----------|----------------|----------|
| Initial load (STANDBY, no dev mode) | Fresh page load | Teleop pane covered by overlay; logs/camera visible; WASD keys do nothing |
| Dev mode ON | Click DEV in topbar | Overlay disappears; WASD keys work |
| Dev mode OFF | Click DEV again | Overlay reappears; WASD keys blocked |
| Deploy hold (3s) | Hold DEPLOY button | Progress bar fills in 3 seconds (not 5); overlay disappears on ACTIVE state received |
| SPACE bar | Press SPACE at any time | E-stop toggles regardless of overlay state |

- [ ] **Step 8: Commit**

```bash
git add ros2/litterbot_bridge/litterbot_bridge/ui/index.html
git commit -m "ui: replace full-screen standby overlay with pane-level teleop overlay"
```

---

## Task 3: Add LiDAR status dot to status bar

**Files:**
- Modify: `ros2/litterbot_bridge/litterbot_bridge/ui/index.html`

- [ ] **Step 1: Add lastScanTime variable**

Find (around line 602):
```js
let lastOdomTime    = 0;
let lastEncoderTime = 0;
```
Change to:
```js
let lastOdomTime    = 0;
let lastEncoderTime = 0;
let lastScanTime    = 0;  // populated when /scan topic is wired up
```

- [ ] **Step 2: Add LiDAR dot to updateDots()**

Find in `updateDots()`:
```js
setDot('dot-cam', null);  // not connected yet
```
Change to:
```js
setDot('dot-cam',   null);  // not connected yet
setDot('dot-lidar', null);  // not connected yet — wired up when rplidar_ros is added
```

- [ ] **Step 3: Add LiDAR dot HTML to status bar**

Find in `#statusbar`:
```html
<div class="status-item"><div class="dot" id="dot-cam"></div><span>Camera</span></div>
```
Change to:
```html
<div class="status-item"><div class="dot" id="dot-cam"></div><span>Camera</span></div>
<div class="status-item"><div class="dot" id="dot-lidar"></div><span>LiDAR</span></div>
```

- [ ] **Step 4: Verify manually**

Reload `http://localhost:8765`. Status bar should now show: STATE | WebSocket | E-Stop | STM32 | Bridge | Odom | Camera | LiDAR — all grey except WS (green if connected).

- [ ] **Step 5: Commit**

```bash
git add ros2/litterbot_bridge/litterbot_bridge/ui/index.html
git commit -m "ui: add LiDAR status dot placeholder to status bar"
```

---

## Task 4: Create CHANGELOG.md

**Files:**
- Create: `CHANGELOG.md` (repo root)

- [ ] **Step 1: Create the file**

Create `/mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/CHANGELOG.md` with this content:

```markdown
# Changelog

Entries are added when merging to `main`. Use `[Unreleased]` for in-progress work on feature branches.

Format: one entry per session — date, author, bullet list of what changed.

---

## [Unreleased] — feature/arm-control

### 2026-04-02 (benji)
- Replaced full-screen deploy overlay with pane-level overlay scoped to teleop panel
- Deploy hold time reduced from 5s to 3s
- Added LiDAR status dot placeholder to status bar
- Created CHANGELOG.md and added doc conventions to CLAUDE.md
- Updated ros2/CLAUDE.md to reflect ui_server changes
```

- [ ] **Step 2: Commit**

```bash
git add CHANGELOG.md
git commit -m "docs: add CHANGELOG.md with branching convention"
```

---

## Task 5: Update CLAUDE.md files

**Files:**
- Modify: `CLAUDE.md` (repo root)
- Modify: `ros2/CLAUDE.md`

- [ ] **Step 1: Add CHANGELOG reference to root CLAUDE.md**

Find the **Documentation Structure** table in `CLAUDE.md`:
```markdown
| Layer | File | Purpose |
|-------|------|---------|
| **1 — Agent briefs** | `CLAUDE.md` files | ...
| **2 — Dev logs** | `Litterbot_Obsidian/Technical Documentation/*.md` | ...
| **3 — Reference** | `docs/`, `LB_FIRM/`, `Report/` | ...
```
Change to:
```markdown
| Layer | File | Purpose |
|-------|------|---------|
| **1 — Agent briefs** | `CLAUDE.md` files | Machine-readable context for AI agents. Auto-loaded by Claude Code. Kept at repo root and each subsystem root. Always current. |
| **2 — Changelog** | `CHANGELOG.md` | Session log — one entry per merge to main. Use `[Unreleased]` on feature branches. Add an entry every time you merge. |
| **3 — Dev logs** | `Litterbot_Obsidian/Technical Documentation/*.md` | Human+agent readable. Detailed technical docs: protocols, test procedures, node specs, known bugs, kinematics math. Updated as we build and test. |
| **4 — Reference** | `docs/`, `LB_FIRM/`, `Report/` | Static reference material: datasheets, logbooks, original firmware, course deliverables. Not updated regularly. |
```

Then find the **Rules** section:
```markdown
**Rules:**
- After every working session, update the **Current Project Status** table above
- After building or testing any node/module, update the relevant Obsidian doc
- Keep `CLAUDE.md` files factual and current — they are the source of truth for any agent starting a new session
```
Change to:
```markdown
**Rules:**
- After every working session, update the **Current Project Status** table above
- After building or testing any node/module, update the relevant Obsidian doc
- Keep `CLAUDE.md` files factual and current — they are the source of truth for any agent starting a new session
- Add a `CHANGELOG.md` entry when merging to `main`. Use `[Unreleased]` for in-progress work on feature branches.
```

- [ ] **Step 2: Update ui_server row in ros2/CLAUDE.md**

Find in `ros2/CLAUDE.md`:
```markdown
| `ui_server` | `litterbot_bridge/ui_server.py` | **Done** — deploy overlay, dev mode toggle, state broadcasting, LiDAR pane |
```
Change to:
```markdown
| `ui_server` | `litterbot_bridge/ui_server.py` | **Done** — pane-level deploy overlay (teleop panel only), dev mode gating (overlay hidden in dev mode), 3s deploy hold, LiDAR status dot placeholder |
```

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md ros2/CLAUDE.md
git commit -m "docs: add CHANGELOG convention to CLAUDE.md, update ros2/CLAUDE.md ui_server status"
```

---

## Final check

After all tasks are done, run through this checklist:

- [ ] Deploy overlay covers only the teleop pane (not the logs or camera)
- [ ] WASD/F keys are blocked when STANDBY + dev mode OFF
- [ ] WASD/F keys work when dev mode ON (any state)
- [ ] SPACE bar e-stop always works regardless of overlay
- [ ] Deploy hold fills in ~3 seconds (not 5)
- [ ] Status bar shows 8 dots: STATE | WS | E-Stop | STM32 | Bridge | Odom | Camera | LiDAR
- [ ] CHANGELOG.md exists at repo root
- [ ] `git log --oneline` shows 5 clean commits for this work
