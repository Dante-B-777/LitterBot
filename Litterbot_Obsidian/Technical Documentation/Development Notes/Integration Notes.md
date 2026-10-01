# LitterBot — Integration Notes

> Hardware bring-up log. Updated as we connect subsystems and debug real hardware.
>
> **Last updated: 2026-03-29 (session 4)**

---

## Session 3 — 2026-03-29: Jetson Deployment + STM32 Hardware Bring-Up

### What We Did

#### Jetson Setup (complete)
- ROS2 Humble installed, workspace at `~/litterbot_ws/`
- Package symlinked from repo, pip deps installed, `colcon build` passing
- Launch confirmed working: `ros2 launch litterbot_bridge litterbot.launch.py`
- Static IP set to `192.168.1.100` via `nmcli`
  - **Gateway was wrong on first attempt** — set to `192.168.1.1` but actual router is at `192.168.1.254`
  - Fixed with: `nmcli connection modify 'Better than the spectrum' ipv4.gateway '192.168.1.254'`
  - DNS also needed to be added: `ipv4.dns '8.8.8.8,8.8.4.4'`
- Jetson sleep disabled permanently: `systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target`
- SSH key-based auth from WSL2 → Jetson working
- `st-flash` and `arm-none-eabi-objcopy` installed on Jetson for remote flashing

#### STM32 Firmware — Bugs Found and Fixed

**Bug 1: LPUART1 word length (9B → 8B)**
- `main.c` had `UART_WORDLENGTH_9B` — invalid for standard 8N1
- Fixed to `UART_WORDLENGTH_8B`

**Bug 2: UART teleop mode not enabled**
- All UART teleop code was commented out in `main.c`
- Fixed: uncommented `Encoder_Init()`, `Odometry_Init()`, `UartCmd_Init()`, `HAL_UART_Receive_IT()` in setup
- Fixed: uncommented `Encoder_Update()`, `Odometry_Update()`, `UartCmd_Task()` in while loop
- Fixed: commented out `ServoTest_Init()` and `ServoTest_Poll()` which conflicted

**Bug 3: `rx_byte` and `HAL_UART_RxCpltCallback` commented out**
- `static uint8_t rx_byte` was commented out — would have caused compile error
- `HAL_UART_RxCpltCallback` was commented out — bytes would never be processed even if received
- Both fixed

**Bug 4 (root cause): `LPUART1_IRQHandler` missing from `stm32l4xx_it.c`**
- This was the actual reason motors didn't respond
- Without this handler, the LPUART1 receive interrupt fired into `Default_Handler` (an infinite loop), **locking up the entire MCU on the first received byte**
- Fixed by adding to `stm32l4xx_it.c`:
  ```c
  extern UART_HandleTypeDef hlpuart1;

  void LPUART1_IRQHandler(void) {
      HAL_UART_IRQHandler(&hlpuart1);
  }
  ```
- Also added `extern UART_HandleTypeDef hlpuart1` so the handler can reference it

#### Remote Flashing Workflow (new)
STM32 is now connected to Jetson, not Windows. New build+flash process:

1. **Build on Windows** (STM32CubeIDE, `Ctrl+B`) — generates `.elf` in `Debug/`
2. **Copy ELF to Jetson** from WSL2:
   ```bash
   scp /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/firmware/stm32/LitterBot/Debug/LitterBot.elf ben@192.168.1.100:/tmp/LitterBot.elf
   ```
3. **Convert ELF → BIN on Jetson**:
   ```bash
   ssh ben@192.168.1.100 "arm-none-eabi-objcopy -O binary /tmp/LitterBot.elf /tmp/LitterBot.bin"
   ```
4. **Flash with st-flash**:
   ```bash
   ssh ben@192.168.1.100 "sudo st-flash --reset write /tmp/LitterBot.bin 0x08000000"
   ```

Or all in one command after a successful build:
```bash
scp /mnt/c/.../Debug/LitterBot.elf ben@192.168.1.100:/tmp/LitterBot.elf && \
ssh ben@192.168.1.100 "arm-none-eabi-objcopy -O binary /tmp/LitterBot.elf /tmp/LitterBot.bin && sudo st-flash --reset write /tmp/LitterBot.bin 0x08000000"
```

#### Running the Stack (Jetson — Real Hardware)
```bash
# SSH in
ssh ben@192.168.1.100

# Start (background, persists after SSH disconnect)
source /opt/ros/humble/setup.bash && source ~/litterbot_ws/install/setup.bash
nohup ros2 launch litterbot_bridge litterbot.launch.py mock_serial:=false serial_port:=/dev/ttyACM0 > /tmp/litterbot.log 2>&1 &

# Check log
cat /tmp/litterbot.log

# UI — open in laptop browser
http://192.168.1.100:8765

# Kill everything
sudo pkill -9 -f ui_server; sudo pkill -9 -f stm32_bridge; sudo fuser -k 8765/tcp
```

---

## Network Info

| Device | IP | Notes |
|--------|-----|-------|
| Jetson | `192.168.1.100` | Static via nmcli |
| Router/Gateway | `192.168.1.254` | NOT .1 |
| WSL2 host (Windows laptop) | `192.168.1.226` | DHCP |

---

## STM32 → Jetson Physical Connection

- NUCLEO-144 ST-Link USB → Jetson USB port
- Appears as `/dev/ttyACM0` on Jetson
- Same USB port used for both flashing (ST-Link) and UART comms (VCP)
- **Important:** When the ROS2 launch is running, it holds `/dev/ttyACM0` open. `st-flash` can still flash because it uses the ST-Link debug interface (separate from the VCP), but to be safe, stop the launch before flashing.

---

## Known Issues / Open Items

- [x] ~~Motor movement not yet confirmed~~ — **CONFIRMED 2026-03-29, all 4 wheels respond to WASD**
- [x] ~~systemd auto-start service not yet set up~~ — **DONE 2026-03-29** (`/etc/systemd/system/litterbot.service`)
- [ ] Scoop sequence placeholder values — need physical measurement before `S` command is safe to use
- [ ] udev rule for persistent `/dev/ttySTM32` device name not yet set up
- [ ] Need to verify motor directions (may need to flip DIR polarity for some wheels)
- [ ] `ben` user needs passwordless sudo for `st-flash` (currently requires sudo password)

---

---

## Session 4 — 2026-03-29: Motor Teleop Confirmed Working

### Root Cause of "motors not moving" — SOLVED

**Bug 5: `st-flash` does not reset the MCU after writing**

- `st-flash write` programs the flash but leaves the MCU in a halted state
- The new firmware never executed until the user pressed the hardware RESET button
- Fix: always use `sudo st-flash --reset write ...` — the `--reset` flag sends a system reset command after flashing
- This was masking ALL previous firmware behavior: UART RX, motor control, LED blinks — nothing was running

**Confirmed working (2026-03-29):**
- All 4 wheels respond to WASD via UI at `http://192.168.1.100:8765`
- UART RX (Jetson → STM32) confirmed: motor commands received and executed
- LD3 heartbeat blink visible on NUCLEO board

### Systemd Auto-Start
ROS2 stack now auto-starts on Jetson boot:
```bash
sudo systemctl status litterbot   # check status
sudo systemctl start litterbot    # start manually
sudo systemctl stop litterbot     # stop
sudo journalctl -u litterbot -f   # follow logs
```

### Updated Flash Command
Always use `--reset`:
```bash
scp /mnt/c/.../Debug/LitterBot.elf ben@192.168.1.100:/tmp/LitterBot.elf && \
ssh ben@192.168.1.100 "arm-none-eabi-objcopy -O binary /tmp/LitterBot.elf /tmp/LitterBot.bin && sudo st-flash --reset write /tmp/LitterBot.bin 0x08000000"
```

---

## Lessons Learned

- **Always use `st-flash --reset write`** — without `--reset`, the MCU stays halted after programming and never runs the new firmware. The ST-Link holds the chip in a debug halt state until an explicit reset is sent or the RESET button is pressed.
- **Always add peripheral IRQ handlers to `stm32l4xx_it.c`** — CubeMX generates the file but leaves peripheral handlers out if not configured in the `.ioc`. The startup file has weak aliases to `Default_Handler`, which is an infinite loop. Missing handler = MCU lockup on first interrupt.
- **CubeMX word length default is wrong for LPUART** — it defaults to 9 bits when UART parity is disabled. Always verify 8N1 manually in the generated code.
- **Test serial loopback before debugging higher layers** — could have caught the IRQ issue earlier by checking if the STM32 echoed anything back on the T command.
- **Router gateway discovery** — `192.168.1.1` is not always the gateway. Use `ping` sweep or check neighbor ARP table if internet doesn't work after setting static IP.
