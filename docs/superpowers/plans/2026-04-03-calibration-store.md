# Calibration Store Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Store scoop calibration values (arm step count + 4 servo positions) in STM32 flash so they persist across reboots and take effect immediately without a firmware rebuild.

**Architecture:** New `calibration_store` firmware module owns all flash read/write. `scoop_sequence.c` reads from it at runtime instead of using `#define` constants. New UART commands let the operator measure and save servo positions via the UI. Values are saved to a dedicated flash page and loaded into RAM on boot.

**Tech Stack:** STM32 HAL FLASH API (HAL_FLASH_Program, HAL_FLASHEx_Erase), FastAPI WebSocket (existing), Vanilla JS UI (existing).

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `firmware/stm32/LitterBot/Core/Inc/calibration_store.h` | Create | Public API for cal store |
| `firmware/stm32/LitterBot/Core/Src/calibration_store.c` | Create | Flash read/write + RAM cache |
| `firmware/stm32/LitterBot/Core/Src/scoop_sequence.c` | Modify | Use CalStore getters instead of #defines |
| `firmware/stm32/LitterBot/Core/Src/arm_control.c` | Modify | Save extend_steps after calibration; fix HX65_Init |
| `firmware/stm32/LitterBot/Core/Src/uart_cmd.c` | Modify | Add CAL SERVO/SAVE commands + servo_cal polling |
| `firmware/stm32/LitterBot/Core/Src/main.c` | Modify | Call CalStore_Init() on boot |
| `ros2/litterbot_bridge/litterbot_bridge/stm32_bridge.py` | Modify | New WebSocket → UART command mappings |
| `ros2/litterbot_bridge/litterbot_bridge/ui/index.html` | Modify | Calibration button + servo setup panel |

---

## Task 1: Fix firmware crash (LD3 solid)

**Files:**
- Modify: `firmware/stm32/LitterBot/Core/Src/arm_control.c`

The firmware is hanging somewhere in init. This must be fixed before anything else.

- [ ] **Step 1: Identify crash location with CubeIDE debugger**

In STM32CubeIDE: press F11 (Debug), set a breakpoint at the first line of `main()` (line 99: `HAL_Init()`). Step through with F6 (Step Over). Watch for the execution to stop progressing — that line is the crash.

Common suspects in order:
1. `SystemClock_Config()` — LSE timeout (line 201: `HAL_RCC_OscConfig`)
2. `MX_USART3_UART_Init()` — HAL_HalfDuplex_Init fails
3. `UartCmd_Init()` → `Arm_Init()` → `CL57T_Init()`

- [ ] **Step 2a: If crash is in SystemClock_Config — bypass LSE**

If `HAL_RCC_OscConfig` is the hang, the LSE crystal is not starting. In `SystemClock_Config()` in `main.c`, change:

```c
RCC_OscInitStruct.LSEState = RCC_LSE_ON;
```
to:
```c
RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
```

And remove MSI auto-calibration at the end of `SystemClock_Config`:
```c
// Comment out this line:
// HAL_RCCEx_EnableMSIPLLMode();
```

- [ ] **Step 2b: If crash is in MX_USART3_UART_Init — verify CubeMX pin**

Open CubeMX (`.ioc` file). Verify USART3 TX is assigned to PC10 (not PB10 which is damaged). If it shows PB10, reassign to PC10, regenerate code, rebuild.

- [ ] **Step 3: Rebuild in CubeIDE (Ctrl+B) and flash**

```bash
scp /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/firmware/stm32/LitterBot/Debug/LitterBot.elf ben@192.168.1.100:/tmp/LitterBot.elf && \
ssh ben@192.168.1.100 "arm-none-eabi-objcopy -O binary /tmp/LitterBot.elf /tmp/LitterBot.bin && sudo st-flash --reset write /tmp/LitterBot.bin 0x08000000"
```

- [ ] **Step 4: Verify LD3 blinks at ~1 Hz**

LD3 (red LED, PB14) should toggle visibly. If it blinks, the crash is fixed. Proceed to Task 2.

- [ ] **Step 5: Commit**

```bash
git add firmware/stm32/LitterBot/Core/Src/main.c firmware/stm32/LitterBot/LitterBot.ioc
git commit -m "fix: resolve STM32 firmware crash in init (LD3 solid)"
```

---

## Task 2: Create `calibration_store.h`

**Files:**
- Create: `firmware/stm32/LitterBot/Core/Inc/calibration_store.h`

- [ ] **Step 1: Create the header**

```c
/**
 * @file    calibration_store.h
 * @brief   Persistent calibration storage in STM32 flash
 *
 * Stores scoop mechanism calibration values in the last flash page
 * (Bank 2, page 255, address 0x080FF800). Values are loaded into RAM
 * on boot and used by scoop_sequence.c at runtime.
 *
 * Flash layout (16 bytes = 2 doublewords):
 *   Word 0: [magic 0xCAFEBABE (32b)] [extend_steps (32b)]
 *   Word 1: [pos_home (16b)] [pos_open (16b)] [pos_closed (16b)] [pos_dump (16b)]
 *
 * If magic is absent (first boot / erased), all getters return the
 * #define defaults from scoop_sequence.h.
 */

#ifndef CALIBRATION_STORE_H
#define CALIBRATION_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>

/** @brief Flash page reserved for calibration data */
#define CAL_FLASH_BANK    FLASH_BANK_2
#define CAL_FLASH_PAGE    255
#define CAL_FLASH_ADDR    0x080FF800UL
#define CAL_MAGIC         0xCAFEBABEUL

/** @brief Servo position identifiers */
typedef enum {
    CAL_POS_HOME   = 0,
    CAL_POS_OPEN   = 1,
    CAL_POS_CLOSED = 2,
    CAL_POS_DUMP   = 3
} CalServoPos_t;

/**
 * @brief Load calibration values from flash into RAM.
 * Call once at boot, before UartCmd_Init.
 */
void CalStore_Init(void);

/**
 * @brief Get the calibrated arm extend step count.
 * Returns stored value if valid, otherwise SCOOP_EXTEND_STEPS default.
 */
uint32_t CalStore_GetExtendSteps(void);

/**
 * @brief Get a calibrated servo position.
 * Returns stored value if valid, otherwise the corresponding SCOOP_POS_* default.
 * @param which  Which position to retrieve (CAL_POS_HOME/OPEN/CLOSED/DUMP)
 */
uint16_t CalStore_GetServoPos(CalServoPos_t which);

/**
 * @brief Save extend step count to flash and update RAM immediately.
 * @param steps  Measured step count from arm calibration
 */
void CalStore_SaveExtendSteps(uint32_t steps);

/**
 * @brief Save a servo position to flash and update RAM immediately.
 * @param which  Which position to save
 * @param pos    Servo position value (0-4095)
 */
void CalStore_SaveServoPos(CalServoPos_t which, uint16_t pos);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_STORE_H */
```

- [ ] **Step 2: Commit**

```bash
git add firmware/stm32/LitterBot/Core/Inc/calibration_store.h
git commit -m "firmware: add calibration_store.h — flash-backed cal value API"
```

---

## Task 3: Create `calibration_store.c`

**Files:**
- Create: `firmware/stm32/LitterBot/Core/Src/calibration_store.c`

- [ ] **Step 1: Create the implementation**

```c
/**
 * @file    calibration_store.c
 * @brief   Persistent calibration storage implementation
 *
 * See calibration_store.h for full documentation.
 */

#include "calibration_store.h"
#include "scoop_sequence.h"  /* For default #define values */

/* Private variables — RAM cache of calibration values -----------------------*/

static uint32_t g_extend_steps;
static uint16_t g_pos[4];  /* Indexed by CalServoPos_t */
static uint8_t  g_valid = 0;

/* Private helpers -----------------------------------------------------------*/

static void load_defaults(void)
{
    g_extend_steps = SCOOP_EXTEND_STEPS;
    g_pos[CAL_POS_HOME]   = SCOOP_POS_HOME;
    g_pos[CAL_POS_OPEN]   = SCOOP_POS_OPEN;
    g_pos[CAL_POS_CLOSED] = SCOOP_POS_CLOSED;
    g_pos[CAL_POS_DUMP]   = SCOOP_POS_DUMP;
}

static void write_to_flash(void)
{
    uint64_t word0 = ((uint64_t)g_extend_steps << 32) | CAL_MAGIC;
    uint64_t word1 = ((uint64_t)g_pos[CAL_POS_DUMP]   << 48)
                   | ((uint64_t)g_pos[CAL_POS_CLOSED]  << 32)
                   | ((uint64_t)g_pos[CAL_POS_OPEN]    << 16)
                   |  (uint64_t)g_pos[CAL_POS_HOME];

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = CAL_FLASH_BANK,
        .Page      = CAL_FLASH_PAGE,
        .NbPages   = 1
    };
    uint32_t page_error;
    HAL_FLASHEx_Erase(&erase, &page_error);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR,     word0);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR + 8, word1);

    HAL_FLASH_Lock();
}

/* Public functions ----------------------------------------------------------*/

void CalStore_Init(void)
{
    load_defaults();

    uint64_t word0 = *(volatile uint64_t*)CAL_FLASH_ADDR;
    uint32_t magic = (uint32_t)(word0 & 0xFFFFFFFF);

    if (magic != CAL_MAGIC) {
        /* No valid calibration data — use defaults */
        g_valid = 0;
        return;
    }

    uint64_t word1 = *(volatile uint64_t*)(CAL_FLASH_ADDR + 8);

    g_extend_steps       = (uint32_t)(word0 >> 32);
    g_pos[CAL_POS_HOME]   = (uint16_t)(word1 & 0xFFFF);
    g_pos[CAL_POS_OPEN]   = (uint16_t)((word1 >> 16) & 0xFFFF);
    g_pos[CAL_POS_CLOSED] = (uint16_t)((word1 >> 32) & 0xFFFF);
    g_pos[CAL_POS_DUMP]   = (uint16_t)((word1 >> 48) & 0xFFFF);
    g_valid = 1;
}

uint32_t CalStore_GetExtendSteps(void)
{
    return g_extend_steps;
}

uint16_t CalStore_GetServoPos(CalServoPos_t which)
{
    if (which > CAL_POS_DUMP) return g_pos[CAL_POS_HOME];
    return g_pos[which];
}

void CalStore_SaveExtendSteps(uint32_t steps)
{
    g_extend_steps = steps;
    g_valid = 1;
    write_to_flash();
}

void CalStore_SaveServoPos(CalServoPos_t which, uint16_t pos)
{
    if (which > CAL_POS_DUMP) return;
    g_pos[which] = pos;
    g_valid = 1;
    write_to_flash();
}
```

- [ ] **Step 2: Add `calibration_store.c` to the CubeIDE project**

In STM32CubeIDE, right-click `Core/Src` → Refresh. The file should appear. If not, right-click project → Properties → C/C++ Build → Settings → verify the source path includes `Core/Src`.

- [ ] **Step 3: Build in CubeIDE (Ctrl+B) — verify no errors**

Expected: build succeeds. If linker complains about `CAL_FLASH_ADDR` overlapping code, the firmware is larger than expected — change `CAL_FLASH_PAGE` to a lower page number (e.g., 250) and update `CAL_FLASH_ADDR` accordingly (`0x080FF800 - (5 * 2048)` = `0x080FEC00` for page 250).

- [ ] **Step 4: Commit**

```bash
git add firmware/stm32/LitterBot/Core/Src/calibration_store.c
git commit -m "firmware: implement calibration_store — flash-backed scoop calibration values"
```

---

## Task 4: Wire CalStore into `scoop_sequence.c` and `main.c`

**Files:**
- Modify: `firmware/stm32/LitterBot/Core/Src/scoop_sequence.c`
- Modify: `firmware/stm32/LitterBot/Core/Inc/scoop_sequence.h` (add note)
- Modify: `firmware/stm32/LitterBot/Core/Src/main.c`

- [ ] **Step 1: Add CalStore include to `scoop_sequence.c`**

At the top of `scoop_sequence.c`, after existing includes:
```c
#include "calibration_store.h"
```

- [ ] **Step 2: Replace `#define` constants in `Scoop_Execute()` with CalStore calls**

In `scoop_sequence.c`, replace the body of `Scoop_Execute()`:

```c
void Scoop_Execute(void)
{
    ArmResult_t result;

    /* Load calibrated values — falls back to #define defaults if not calibrated */
    uint32_t extend_steps    = CalStore_GetExtendSteps();
    uint16_t pos_open        = CalStore_GetServoPos(CAL_POS_OPEN);
    uint16_t pos_closed      = CalStore_GetServoPos(CAL_POS_CLOSED);
    uint16_t pos_dump        = CalStore_GetServoPos(CAL_POS_DUMP);
    uint16_t pos_home        = CalStore_GetServoPos(CAL_POS_HOME);

    uint8_t retract_dir = (SCOOP_EXTEND_DIR == CL57T_DIR_CW)
                        ? CL57T_DIR_CCW : CL57T_DIR_CW;

    /* Phase 1: Pre-position bucket */
    servo_move_wait(pos_open);

    /* Phase 2: Extend arm */
    result = Arm_Extend(extend_steps, SCOOP_EXTEND_DIR, SCOOP_EXTEND_SPEED_US);
    if (result != ARM_OK) {
        servo_move_wait(pos_home);
        return;
    }

    HAL_Delay(100);

    /* Phase 3: Close bucket */
    servo_move_wait(pos_closed);

    /* Phase 4: Retract with servo interpolation */
    result = Arm_RetractWithServo(
        extend_steps,
        retract_dir,
        SCOOP_RETRACT_SPEED_US,
        SCOOP_SERVO_ID,
        pos_closed,
        pos_dump
    );
    if (result != ARM_OK) {
        servo_move_wait(pos_home);
        return;
    }

    /* Phase 5: Dump */
    HAL_Delay(300);

    /* Phase 6: Home */
    servo_move_wait(pos_home);
}
```

- [ ] **Step 3: Add note to `scoop_sequence.h`**

Find the PLACEHOLDER CONSTANTS block header comment and change it to:

```c
/* =========================================================================
 * DEFAULT FALLBACK CONSTANTS
 * These values are used only if no calibration data has been saved to flash.
 * To set real values: use C CALIBRATE command (extend_steps) and
 * CAL SERVO / CAL SAVE commands (servo positions) via the UI.
 * See calibration_store.h for the runtime API.
 * ========================================================================= */
```

- [ ] **Step 4: Call `CalStore_Init()` in `main.c`**

In `main.c`, find `/* USER CODE BEGIN 2 */` and add before `UartCmd_Init`:

```c
CalStore_Init();
```

Also add the include at `/* USER CODE BEGIN Includes */`:
```c
#include "calibration_store.h"
```

- [ ] **Step 5: Build and flash**

```bash
# Build in CubeIDE (Ctrl+B), then:
scp /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/firmware/stm32/LitterBot/Debug/LitterBot.elf ben@192.168.1.100:/tmp/LitterBot.elf && \
ssh ben@192.168.1.100 "arm-none-eabi-objcopy -O binary /tmp/LitterBot.elf /tmp/LitterBot.bin && sudo st-flash --reset write /tmp/LitterBot.bin 0x08000000"
```

- [ ] **Step 6: Verify teleop still works**

Open `http://192.168.1.100:8765`, deploy, drive with WASD. Motors should respond normally. The scoop sequence will still use default values (no calibration saved yet).

- [ ] **Step 7: Commit**

```bash
git add firmware/stm32/LitterBot/Core/Src/scoop_sequence.c \
        firmware/stm32/LitterBot/Core/Inc/scoop_sequence.h \
        firmware/stm32/LitterBot/Core/Src/main.c
git commit -m "firmware: wire calibration_store into scoop_sequence and main"
```

---

## Task 5: Fix `Arm_Init()` and save extend_steps after calibration

**Files:**
- Modify: `firmware/stm32/LitterBot/Core/Src/arm_control.c`

- [ ] **Step 1: Add HX65_Init to Arm_Init()**

In `arm_control.c`, find `Arm_Init()` and add:

```c
void Arm_Init(void)
{
    extern UART_HandleTypeDef huart3;  /* CubeMX-generated handle */
    LimitSwitch_Init();
    CL57T_Init();
    HX65_Init(&huart3);  /* Initialize servo driver — required for all servo ops */
}
```

Also add the include at the top of `arm_control.c`:
```c
#include "hx65_servo.h"
```

- [ ] **Step 2: Save extend_steps at end of Arm_Calibrate()**

In `arm_control.c`, find the end of `Arm_Calibrate()` where it prints the final result. After the final `snprintf`/`print_uart` block and before `return ARM_OK`, add:

```c
    /* Save to flash — takes effect immediately, no reboot needed */
    uint32_t best = (extend_count > retract_count) ? extend_count : retract_count;
    CalStore_SaveExtendSteps(best);
    snprintf(buf, sizeof(buf), "CAL SAVED: extend_steps=%lu\r\n", (unsigned long)best);
    print_uart(huart, buf);

    return ARM_OK;
```

Also add at the top of `arm_control.c`:
```c
#include "calibration_store.h"
```

- [ ] **Step 3: Build (Ctrl+B) — verify no errors**

- [ ] **Step 4: Commit**

```bash
git add firmware/stm32/LitterBot/Core/Src/arm_control.c
git commit -m "firmware: fix HX65_Init in Arm_Init, auto-save extend_steps after calibration"
```

---

## Task 6: Add CAL SERVO commands to `uart_cmd.c`

**Files:**
- Modify: `firmware/stm32/LitterBot/Core/Src/uart_cmd.c`

- [ ] **Step 1: Add servo cal state variables**

In `uart_cmd.c`, in the private variables section (after `g_last_cmd_ms`):

```c
/** @brief True when servo cal mode is active (prints position every 500ms) */
static bool servo_cal_active = false;

/** @brief Timestamp of last position print during servo cal mode */
static uint32_t servo_cal_last_ms = 0;
```

Add include at top:
```c
#include "calibration_store.h"
#include "hx65_servo.h"
#include <stdbool.h>
```

- [ ] **Step 2: Add servo cal polling to `UartCmd_Task()`**

In `UartCmd_Task()`, after the `if (g_ready)` block (after line 132), add:

```c
    /* Servo calibration mode — print position every 500ms */
    if (servo_cal_active && (now_ms - servo_cal_last_ms >= 500)) {
        uint16_t pos;
        if (HX65_ReadPosition(SCOOP_SERVO_ID, &pos) == HX65_OK) {
            char buf[24];
            snprintf(buf, sizeof(buf), "POS: %u\r\n", pos);
            reply(buf);
        }
        servo_cal_last_ms = now_ms;
    }
```

- [ ] **Step 3: Add CAL command handlers to `handle_line()`**

In `handle_line()`, after the `C CALIBRATE` handler (after line 273), before the closing `}`, add:

```c
    /* CAL SERVO START — disable torque, begin printing position every 500ms */
    if (strncmp(s, "CAL SERVO START", 15) == 0) {
        HX65_DisableTorque(SCOOP_SERVO_ID);
        servo_cal_active = true;
        servo_cal_last_ms = now_ms;
        reply("CAL SERVO: torque off, printing position...\r\n");
        return;
    }

    /* CAL SERVO STOP — re-enable torque, stop printing */
    if (strncmp(s, "CAL SERVO STOP", 14) == 0) {
        servo_cal_active = false;
        HX65_EnableTorque(SCOOP_SERVO_ID);
        reply("CAL SERVO: torque on, stopped.\r\n");
        return;
    }

    /* CAL SAVE HOME/OPEN/CLOSED/DUMP — read current position and save */
    if (strncmp(s, "CAL SAVE ", 9) == 0) {
        uint16_t pos;
        if (HX65_ReadPosition(SCOOP_SERVO_ID, &pos) != HX65_OK) {
            reply("CAL SAVE ERR: servo read failed\r\n");
            return;
        }
        char buf[48];
        const char *which_str = s + 9;
        if (strncmp(which_str, "HOME", 4) == 0) {
            CalStore_SaveServoPos(CAL_POS_HOME, pos);
            snprintf(buf, sizeof(buf), "CAL SAVED: pos_home=%u\r\n", pos);
        } else if (strncmp(which_str, "OPEN", 4) == 0) {
            CalStore_SaveServoPos(CAL_POS_OPEN, pos);
            snprintf(buf, sizeof(buf), "CAL SAVED: pos_open=%u\r\n", pos);
        } else if (strncmp(which_str, "CLOSED", 6) == 0) {
            CalStore_SaveServoPos(CAL_POS_CLOSED, pos);
            snprintf(buf, sizeof(buf), "CAL SAVED: pos_closed=%u\r\n", pos);
        } else if (strncmp(which_str, "DUMP", 4) == 0) {
            CalStore_SaveServoPos(CAL_POS_DUMP, pos);
            snprintf(buf, sizeof(buf), "CAL SAVED: pos_dump=%u\r\n", pos);
        } else {
            reply("CAL SAVE ERR: unknown position\r\n");
            return;
        }
        reply(buf);
        return;
    }
```

- [ ] **Step 4: Build (Ctrl+B) — verify no errors**

- [ ] **Step 5: Flash and verify CAL commands work**

From the UI log panel or a serial terminal, with DEV ON:
- Send `CAL SERVO START\n` → should see `POS: XXXX` printing every 500ms (or `CAL SERVO: torque off...`)
- Send `CAL SERVO STOP\n` → printing stops

- [ ] **Step 6: Commit**

```bash
git add firmware/stm32/LitterBot/Core/Src/uart_cmd.c
git commit -m "firmware: add CAL SERVO START/STOP and CAL SAVE commands"
```

---

## Task 7: Update `stm32_bridge.py` with new command types

**Files:**
- Modify: `ros2/litterbot_bridge/litterbot_bridge/stm32_bridge.py`

- [ ] **Step 1: Add new command mappings**

In `stm32_bridge.py`, find the section where WebSocket messages are dispatched to UART (look for the `type` field dispatch — e.g., where `scoop` maps to `S SCOOP\n`). Add the new CAL command mappings in the same pattern:

```python
elif msg_type == 'cal_arm':
    self._send_serial('C CALIBRATE\n')
    self.get_logger().info('Sent: C CALIBRATE')
elif msg_type == 'cal_servo_start':
    self._send_serial('CAL SERVO START\n')
    self.get_logger().info('Sent: CAL SERVO START')
elif msg_type == 'cal_servo_stop':
    self._send_serial('CAL SERVO STOP\n')
    self.get_logger().info('Sent: CAL SERVO STOP')
elif msg_type == 'cal_save_home':
    self._send_serial('CAL SAVE HOME\n')
    self.get_logger().info('Sent: CAL SAVE HOME')
elif msg_type == 'cal_save_open':
    self._send_serial('CAL SAVE OPEN\n')
    self.get_logger().info('Sent: CAL SAVE OPEN')
elif msg_type == 'cal_save_closed':
    self._send_serial('CAL SAVE CLOSED\n')
    self.get_logger().info('Sent: CAL SAVE CLOSED')
elif msg_type == 'cal_save_dump':
    self._send_serial('CAL SAVE DUMP\n')
    self.get_logger().info('Sent: CAL SAVE DUMP')
```

- [ ] **Step 2: Find the exact dispatch location**

The dispatch is in the `_handle_ws_message` method (or similar). Read the current code to find the exact spot before adding.

- [ ] **Step 3: Rebuild ROS2 package**

```bash
cd ~/litterbot_ws && colcon build --packages-select litterbot_bridge && source install/setup.bash
```

Expected: build succeeds with no errors.

- [ ] **Step 4: Commit**

```bash
git add ros2/litterbot_bridge/litterbot_bridge/stm32_bridge.py
git commit -m "ros2: add CAL command types to stm32_bridge WebSocket dispatch"
```

---

## Task 8: Add calibration UI to `index.html`

**Files:**
- Modify: `ros2/litterbot_bridge/litterbot_bridge/ui/index.html`

- [ ] **Step 1: Add CSS for calibration section**

In the `<style>` block, after the `#teleop-deploy-overlay` CSS, add:

```css
/* ---- Calibration section ---- */
#cal-section {
  margin-top: 16px;
  padding-top: 12px;
  border-top: 1px solid #222;
}
#cal-section h4 {
  font-size: 11px;
  font-weight: 700;
  color: #555;
  letter-spacing: 0.1em;
  text-transform: uppercase;
  margin: 0 0 10px 0;
}
.cal-btn {
  position: relative;
  width: 100%;
  height: 40px;
  border: 1px solid #444;
  border-radius: 6px;
  background: #111;
  color: #ccc;
  font-size: 12px;
  font-weight: 600;
  letter-spacing: 0.05em;
  cursor: pointer;
  overflow: hidden;
  user-select: none;
  margin-bottom: 6px;
  transition: border-color 0.2s, color 0.2s;
}
.cal-btn:disabled { opacity: 0.35; cursor: not-allowed; }
.cal-btn:not(:disabled):hover { border-color: #ffa726; color: #ffa726; }
.cal-btn.holding { border-color: #ffa726; color: #ffa726; }
#cal-arm-progress {
  position: absolute;
  left: 0; bottom: 0;
  height: 3px;
  width: 0%;
  background: #ffa726;
  transition: width 0.05s linear;
}
.servo-save-row {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 6px;
  margin-bottom: 6px;
}
#servo-setup-controls { display: none; }
#servo-setup-controls.visible { display: block; }
```

- [ ] **Step 2: Add calibration HTML inside `#teleop-panel`**

After the closing tag of the keyboard teleop controls (find `</div>` that closes the WASD key display area), add:

```html
<!-- Calibration section -->
<div id="cal-section">
  <h4>Calibration</h4>

  <!-- Arm calibration — 2s hold -->
  <button class="cal-btn" id="cal-arm-btn"
    onmousedown="calArmStart()" onmouseup="calArmCancel()" onmouseleave="calArmCancel()"
    ontouchstart="calArmStart()" ontouchend="calArmCancel()" ontouchcancel="calArmCancel()">
    CAL ARM (hold 2s)
    <div id="cal-arm-progress"></div>
  </button>

  <!-- Servo setup -->
  <button class="cal-btn" id="servo-setup-start-btn" onclick="servoSetupStart()">
    START SERVO SETUP
  </button>

  <div id="servo-setup-controls">
    <div class="servo-save-row">
      <button class="cal-btn" onclick="calSave('home')">SAVE HOME</button>
      <button class="cal-btn" onclick="calSave('open')">SAVE OPEN</button>
    </div>
    <div class="servo-save-row">
      <button class="cal-btn" onclick="calSave('closed')">SAVE CLOSED</button>
      <button class="cal-btn" onclick="calSave('dump')">SAVE DUMP</button>
    </div>
    <button class="cal-btn" onclick="servoSetupStop()"
      style="border-color:#ef5350; color:#ef5350;">
      DONE (re-enable torque)
    </button>
  </div>
</div>
```

- [ ] **Step 3: Add calibration JavaScript**

In the `<script>` section, after the existing `deployStart`/`deployCancel` functions, add:

```js
/* ---- Arm calibration (2s hold) ---- */
const CAL_ARM_HOLD_MS = 2000;
let calArmTimer = null;
let calArmStart_t = 0;
let calArmInterval = null;

function calArmStart() {
  const btn = document.getElementById('cal-arm-btn');
  const prog = document.getElementById('cal-arm-progress');
  if (btn.disabled) return;
  calArmStart_t = Date.now();
  btn.classList.add('holding');
  calArmInterval = setInterval(() => {
    const pct = Math.min(100, ((Date.now() - calArmStart_t) / CAL_ARM_HOLD_MS) * 100);
    prog.style.width = pct + '%';
    if (pct >= 100) {
      clearInterval(calArmInterval);
      calArmInterval = null;
      prog.style.width = '0%';
      btn.classList.remove('holding');
      sendWS({ type: 'cal_arm' });
    }
  }, 50);
}

function calArmCancel() {
  if (calArmInterval) {
    clearInterval(calArmInterval);
    calArmInterval = null;
  }
  const btn = document.getElementById('cal-arm-btn');
  const prog = document.getElementById('cal-arm-progress');
  btn.classList.remove('holding');
  prog.style.width = '0%';
}

/* ---- Servo setup ---- */
function servoSetupStart() {
  sendWS({ type: 'cal_servo_start' });
  document.getElementById('servo-setup-controls').classList.add('visible');
  document.getElementById('servo-setup-start-btn').style.display = 'none';
}

function servoSetupStop() {
  sendWS({ type: 'cal_servo_stop' });
  document.getElementById('servo-setup-controls').classList.remove('visible');
  document.getElementById('servo-setup-start-btn').style.display = '';
}

function calSave(pos) {
  const typeMap = {
    home:   'cal_save_home',
    open:   'cal_save_open',
    closed: 'cal_save_closed',
    dump:   'cal_save_dump'
  };
  if (typeMap[pos]) sendWS({ type: typeMap[pos] });
}
```

- [ ] **Step 4: Gate calibration buttons based on robot state**

In the `applyStateUI()` function, add at the end:

```js
  // Gate calibration buttons — require ACTIVE or dev mode
  const calAllowed = robotState === 'ACTIVE' || devMode;
  document.querySelectorAll('.cal-btn').forEach(b => {
    b.disabled = !calAllowed;
  });
```

- [ ] **Step 5: Verify `sendWS` function exists**

Search for the existing `sendWS` helper (or equivalent) in the script. It should be the function that sends a JSON object over the WebSocket. If it's named differently (e.g., `send`, `ws.send`), use the correct name in the JS added in Step 3.

- [ ] **Step 6: Test manually**

Stop and restart the ROS2 stack on the Jetson:
```bash
ssh ben@192.168.1.100 "sudo systemctl stop litterbot && source /opt/ros/humble/setup.bash && source ~/litterbot_ws/install/setup.bash && nohup ros2 launch litterbot_bridge litterbot.launch.py mock_serial:=false serial_port:=/dev/ttyACM0 > /tmp/litterbot.log 2>&1 &"
```

Open `http://192.168.1.100:8765`. Check:
- [ ] Calibration section visible in teleop panel
- [ ] CAL ARM and servo buttons are grey (disabled) in STANDBY
- [ ] After DEV ON: buttons become active
- [ ] Hold CAL ARM for 2s → progress bar fills → `C CALIBRATE\n` sent (visible in blue Jetson→STM32 log)
- [ ] Start Servo Setup → 4 save buttons appear
- [ ] Save buttons send correct commands (visible in log)
- [ ] Done button hides save buttons

- [ ] **Step 7: Commit**

```bash
git add ros2/litterbot_bridge/litterbot_bridge/ui/index.html
git commit -m "ui: add calibration section — arm cal hold button and servo setup panel"
```

---

## Task 9: End-to-end test and docs update

- [ ] **Step 1: Full calibration workflow test (requires hardware)**

With STM32 connected and real hardware running:

1. Open UI, deploy (or DEV ON)
2. Send CAL ARM hold → arm extends to EXTEND switch, retracts → `CAL SAVED: extend_steps=XXXX` appears in log
3. Start Servo Setup → bucket becomes free
4. Move bucket to HOME position → Save Home → `CAL SAVED: pos_home=XXXX` in log
5. Move bucket to OPEN position → Save Open
6. Move bucket to CLOSED position → Save Closed
7. Move bucket to DUMP position → Save Dump
8. Done
9. Power cycle STM32 (unplug/replug USB)
10. Send `S SCOOP\n` in dev mode — arm should use the calibrated values

- [ ] **Step 2: Update CLAUDE.md status table**

In root `CLAUDE.md`, update the firmware status rows:

```markdown
| STM32 firmware — Scoop sequence | `scoop_sequence.c/h` written — **uses CalStore values at runtime, falls back to placeholders if uncalibrated** |
| STM32 firmware — Calibration store | **Done** — flash-backed cal values, UI workflow for arm + servo positions |
```

- [ ] **Step 3: Update CHANGELOG.md**

Add to the `[Unreleased]` section:
```markdown
- Added calibration_store module — extend_steps and 4 servo positions stored in STM32 flash
- Arm calibration now auto-saves extend_steps to flash on completion
- Added CAL SERVO START/STOP and CAL SAVE commands to UART protocol
- Added HX65_Init fix to Arm_Init (servo driver was never initialized)
- UI: calibration section in teleop panel — 2s hold arm cal + servo setup workflow
```

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md CHANGELOG.md
git commit -m "docs: update status and changelog for calibration store feature"
```
