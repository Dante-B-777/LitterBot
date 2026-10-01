# UI 3-Column Layout Redesign + Tabbed Navigation

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restructure the Command Center UI from a 2-column layout to a 3-column tabbed layout with proper aspect ratios for camera and LiDAR, fix the right-column twitching bug, and organize functionality into tabs.

**Architecture:** Single-file change to `index.html`. Three tabs at the top: **Command** (teleop + logs + sensors), **Calibration** (arm cal + servo setup), **Recordings** (record/playback controls). Each tab shares the same 3-column structure where applicable. Camera + LiDAR live in the center column with CSS `aspect-ratio`. ResizeObserver keeps the LiDAR canvas crisp.

**Tech Stack:** HTML, CSS, vanilla JS (no build step, self-contained file)

**File:** `ros2/litterbot_bridge/litterbot_bridge/ui/index.html`

---

## Current Layout (2-column, no tabs)

```
+------ #log-col (flex 1.4) ------+------ #right-col (flex 1) ------+
| [TX log]                        | [Camera  — fixed 170px height]  |
| [RX log]                        | [LiDAR   — canvas 120px height] |
| [Node log]                      | [Odom grid]                     |
|                                 | [Teleop + calibration + rec]    |
+---------------------------------+---------------------------------+
```

**Problems:**
1. Camera panel has a fixed pixel height (`flex: 0 0 170px`), becomes a weird squished strip
2. LiDAR canvas has CSS `height: 120px` but HTML attributes `width=480 height=240` -- aspect ratio fight
3. `#right-col` uses `overflow-y: auto` -- scrollbar appears/disappears as content updates, causing twitching
4. Log panels get 58% of width while visual feeds are squeezed into 42%
5. Right column is overloaded: teleop + odom + record + playback + calibration all stacked

## Target Layout (3-column, tabbed)

```
[COMMAND] [CALIBRATION] [RECORDINGS]           <- tab bar
+--- #log-col (280px) ---+--- #center-col (flex 1) ---+--- #right-col (300px) ---+
```

**Tab: Command (default)**
```
| [TX log]               | [Camera — 16:9]            | [Teleop controls]       |
| [RX log]               | [LiDAR  — 2:1]             | [Odom grid]             |
| [Node log]             | [LiDAR info]               | [Speed slider]          |
|                        |                            | [Key legend]            |
```

**Tab: Calibration**
```
| [TX log]               | [Camera — 16:9]            | [Arm calibration]       |
| [RX log]               | [LiDAR  — 2:1]             | [Servo setup]           |
| [Node log]             | [LiDAR info]               |                         |
```

**Tab: Recordings**
```
| [TX log]               | [Camera — 16:9]            | [Record controls]       |
| [RX log]               | [LiDAR  — 2:1]             | [Playback controls]     |
| [Node log]             | [LiDAR info]               | [Bag list]              |
```

The left column (logs) and center column (sensor feeds) stay visible across all tabs. Only the right column content swaps.

---

### Task 1: Add tab bar and tab content containers to HTML

**Files:**
- Modify: `ros2/litterbot_bridge/litterbot_bridge/ui/index.html` (HTML body)

- [ ] **Step 1: Add tab bar HTML after `#statusbar`**

Insert a tab bar between `</div><!-- statusbar -->` and `<div id="main">`:

```html
<div id="tab-bar">
  <button class="tab active" data-tab="command" onclick="switchTab('command')">COMMAND</button>
  <button class="tab" data-tab="calibration" onclick="switchTab('calibration')">CALIBRATION</button>
  <button class="tab" data-tab="recordings" onclick="switchTab('recordings')">RECORDINGS</button>
</div>
```

- [ ] **Step 2: Restructure `#main` into 3 columns with tabbed right-column content**

Replace the entire `<div id="main">` section. The left and center columns stay constant. The right column gets three `tab-content` divs:

```html
<div id="main">

  <div id="log-col">
    <div class="log-panel">
      <div class="log-header stm32-tx">Jetson to STM32</div>
      <div class="log-body" id="log-stm32-tx"></div>
    </div>
    <div class="log-panel">
      <div class="log-header stm32-rx">STM32 to Jetson</div>
      <div class="log-body" id="log-stm32-rx"></div>
    </div>
    <div class="log-panel">
      <div class="log-header node">Node to Node</div>
      <div class="log-body" id="log-node"></div>
    </div>
  </div>

  <div id="center-col">
    <div id="camera-panel">
      <img id="camera-feed" alt="LitterBot camera feed">
      <div id="camera-placeholder">[ Camera — not yet connected ]</div>
    </div>
    <div id="lidar-panel">
      <div id="lidar-title">LiDAR</div>
      <canvas id="lidar-canvas"></canvas>
      <div id="lidar-info">[ LiDAR — not yet connected ]</div>
    </div>
  </div>

  <div id="right-col">
    <!-- Tab: Command -->
    <div class="tab-content active" id="tab-command">
      <div id="teleop-panel">
        <!-- deploy overlay, key grid, space/scoop keys, speed slider, legend, status overlays -->
        <!-- (all existing teleop content moves here, minus calibration and recording sections) -->
      </div>
      <div id="odom-panel">
        <!-- existing odom grid -->
      </div>
    </div>

    <!-- Tab: Calibration -->
    <div class="tab-content" id="tab-calibration">
      <div id="cal-panel">
        <h3>Arm Calibration</h3>
        <!-- CAL ARM button with hold-2s -->
        <h3>Servo Setup</h3>
        <!-- START SERVO SETUP, save buttons, done button -->
      </div>
    </div>

    <!-- Tab: Recordings -->
    <div class="tab-content" id="tab-recordings">
      <div id="rec-panel">
        <h3>Record</h3>
        <!-- REC button, status -->
        <h3>Playback</h3>
        <!-- playback select, refresh, play button, status -->
      </div>
    </div>
  </div>

</div>
```

Remove `width="480" height="240"` from the `<canvas>` tag.

---

### Task 2: Add tab bar and tab content CSS

**Files:**
- Modify: `ros2/litterbot_bridge/litterbot_bridge/ui/index.html` (CSS `<style>` block)

- [ ] **Step 1: Add tab bar CSS**

```css
  /* ---- Tab bar ---- */
  #tab-bar {
    display: flex;
    gap: 0;
    background: #111;
    border-bottom: 1px solid #2a2a2a;
    flex-shrink: 0;
    padding: 0 16px;
  }

  .tab {
    padding: 8px 20px;
    border: none;
    border-bottom: 2px solid transparent;
    background: transparent;
    color: #666;
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.08em;
    cursor: pointer;
    transition: color 0.15s, border-color 0.15s;
  }
  .tab:hover { color: #aaa; }
  .tab.active {
    color: #4fc3f7;
    border-bottom-color: #4fc3f7;
  }

  .tab-content {
    display: none;
    flex-direction: column;
    flex: 1;
    min-height: 0;
    overflow-y: auto;
  }
  .tab-content.active {
    display: flex;
  }
```

---

### Task 3: Update CSS for 3-column layout and fix twitching

**Files:**
- Modify: `ros2/litterbot_bridge/litterbot_bridge/ui/index.html` (CSS)

- [ ] **Step 1: Replace main layout CSS**

```css
  /* ---- Main layout ---- */
  #main {
    display: flex;
    flex: 1;
    min-height: 0;
    overflow: hidden;
  }

  #log-col {
    display: flex;
    flex-direction: column;
    flex: 0 0 280px;
    border-right: 1px solid #2a2a2a;
    overflow: hidden;
  }

  /* (keep existing .log-panel, .log-header, .log-body, .log-entry CSS unchanged) */

  /* ---- Center column (camera + LiDAR) ---- */
  #center-col {
    display: flex;
    flex-direction: column;
    flex: 1;
    min-height: 0;
    min-width: 0;
    overflow-y: auto;
    border-right: 1px solid #2a2a2a;
  }

  /* ---- Right column (tabbed controls) ---- */
  #right-col {
    display: flex;
    flex-direction: column;
    flex: 0 0 300px;
    min-height: 0;
  }
```

Key changes:
- `#log-col`: `flex: 1.4` -> `flex: 0 0 280px`
- New `#center-col`: `flex: 1`
- `#right-col`: `flex: 1` -> `flex: 0 0 300px`, removed `overflow-y: auto` (scroll now on `.tab-content`)

---

### Task 4: Update camera and LiDAR panel CSS for proper aspect ratios

- [ ] **Step 1: Replace camera panel CSS**

```css
  #camera-panel {
    width: 100%;
    aspect-ratio: 16 / 9;
    display: flex;
    align-items: center;
    justify-content: center;
    background: #111;
    border-bottom: 1px solid #2a2a2a;
    color: #333;
    font-size: 13px;
    letter-spacing: 0.05em;
    position: relative;
    overflow: hidden;
    flex-shrink: 0;
  }
```

- [ ] **Step 2: Change camera-feed object-fit to contain**

```css
  #camera-feed {
    width: 100%;
    height: 100%;
    object-fit: contain;
    display: none;
  }
```

- [ ] **Step 3: Replace LiDAR panel CSS**

```css
  #lidar-panel {
    display: flex;
    flex-direction: column;
    background: #111;
    border-bottom: 1px solid #2a2a2a;
    color: #8f8f8f;
    font-size: 12px;
    letter-spacing: 0.03em;
    padding: 10px;
    gap: 6px;
    flex-shrink: 0;
  }
```

- [ ] **Step 4: Replace LiDAR canvas CSS**

```css
  #lidar-canvas {
    width: 100%;
    aspect-ratio: 2 / 1;
    border: 1px solid #233423;
    border-radius: 6px;
    background:
      radial-gradient(circle at center, rgba(102,187,106,0.08) 0, rgba(102,187,106,0.04) 35%, rgba(0,0,0,0) 36%),
      linear-gradient(180deg, #091109, #050805);
    display: block;
  }
```

- [ ] **Step 5: Delete all height-based media queries**

Delete entirely:
```css
  @media (max-height: 900px) { ... }
  @media (max-height: 760px) { ... }
  @media (max-width: 1100px) { ... }
```

---

### Task 5: Update odom grid for narrower right column

- [ ] **Step 1: Change odom grid to 2 columns**

```css
  #odom-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 6px;
  }
```

---

### Task 6: Add tab switching JS and ResizeObserver

- [ ] **Step 1: Add `switchTab()` function**

```javascript
// ---- Tab switching ----
function switchTab(tabName) {
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
  document.querySelector(`.tab[data-tab="${tabName}"]`).classList.add('active');
  document.getElementById(`tab-${tabName}`).classList.add('active');
}
```

- [ ] **Step 2: Add ResizeObserver for LiDAR canvas**

```javascript
// ---- LiDAR canvas auto-resize ----
(function() {
  const canvas = document.getElementById('lidar-canvas');
  if (!canvas) return;
  const ro = new ResizeObserver(entries => {
    for (const entry of entries) {
      const rect = entry.contentRect;
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.round(rect.width * dpr);
      canvas.height = Math.round(rect.height * dpr);
      renderLidarPlot();
    }
  });
  ro.observe(canvas);
})();
```

---

### Task 7: Style calibration and recording panels

- [ ] **Step 1: Add CSS for cal-panel and rec-panel**

```css
  #cal-panel, #rec-panel {
    padding: 12px 16px;
    background: #0d0d0d;
  }

  #cal-panel h3, #rec-panel h3 {
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: #888;
    margin-bottom: 10px;
    margin-top: 16px;
  }
  #cal-panel h3:first-child, #rec-panel h3:first-child {
    margin-top: 0;
  }
```

---

### Task 8: Remove old recordings page link

- [ ] **Step 1: Remove `#recordings-link` from topbar**

The recordings tab replaces the separate `/recordings` page link. Remove:
```html
<a id="recordings-link" href="/recordings">RECORDINGS</a>
```

---

### Task 9: Visual verification and commit

- [ ] **Step 1: Verify Command tab**
- 3 columns: logs (left), camera+LiDAR (center), teleop+odom (right)
- Camera 16:9, LiDAR 2:1, no twitching
- WASD keys, speed slider, deploy overlay all work
- Keyboard teleop still functions (keys not captured by tab buttons)

- [ ] **Step 2: Verify Calibration tab**
- Logs and sensor feeds still visible
- Right column shows arm cal and servo setup controls
- Hold-to-calibrate and servo save buttons work

- [ ] **Step 3: Verify Recordings tab**
- Right column shows record/playback controls
- REC button, playback dropdown, refresh all functional

- [ ] **Step 4: Verify tab switching**
- Clicking tabs swaps right column content
- Active tab is highlighted
- No flicker or layout shift on switch
- Keyboard events still work after switching tabs

- [ ] **Step 5: Commit**

```bash
git add ros2/litterbot_bridge/litterbot_bridge/ui/index.html
git commit -m "ui: 3-column tabbed layout with proper aspect ratios

- 3-column layout: logs (280px) | camera+LiDAR (flex) | controls (300px)
- 3 tabs: Command (teleop+odom), Calibration, Recordings
- Camera uses aspect-ratio: 16/9, LiDAR uses aspect-ratio: 2/1
- Fix right-col twitching via tab-content scroll
- Add ResizeObserver for crisp LiDAR canvas at any size
- Remove standalone recordings page link"
```
