/**
 * @file    cl57t_stepper.c
 * @brief   CL57T closed-loop stepper motor driver implementation
 *
 * Uses GPIO bit-banging to generate step pulses for the CL57T driver.
 * This is a simple initial implementation suitable for basic positioning.
 * For high-speed or smooth motion profiles, a timer-based approach with
 * acceleration ramps should replace this bit-bang method.
 *
 * Microsecond delays use a DWT (Data Watchpoint and Trace) cycle counter
 * for accurate timing without consuming a hardware timer.
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "cl57t_stepper.h"

/* Private variables ---------------------------------------------------------*/

/** @brief Stepper motor configuration (populated during init) */
static CL57T_Stepper_t stepper;

/** @brief Cached last direction so repeat MoveSteps calls can skip the
 *  50–500 µs DIR setup wait when the direction has not actually changed.
 *  -1 means "unknown" (force a write on first call). */
static int8_t s_last_dir = -1;

/* Private function prototypes -----------------------------------------------*/
static void CL57T_DelayUs(uint32_t us);
static void CL57T_InitDWT(void);

/* Public functions ----------------------------------------------------------*/

void CL57T_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Populate stepper configuration */
    stepper.pul_port = CL57T_PUL_PORT;
    stepper.pul_pin  = CL57T_PUL_PIN;
    stepper.dir_port = CL57T_DIR_PORT;
    stepper.dir_pin  = CL57T_DIR_PIN;

    /* Enable GPIO clocks for PUL (Port D) and DIR (Port F) */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* Initialize pins LOW before configuring as outputs */
    HAL_GPIO_WritePin(stepper.pul_port, stepper.pul_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(stepper.dir_port, stepper.dir_pin, GPIO_PIN_RESET);

    /* Configure PUL pin (PD14) as push-pull output */
    GPIO_InitStruct.Pin   = stepper.pul_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* Fast edges for clean pulses */
    HAL_GPIO_Init(stepper.pul_port, &GPIO_InitStruct);

    /* Configure DIR pin (PF15) as push-pull output */
    GPIO_InitStruct.Pin   = stepper.dir_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* Direction changes are infrequent */
    HAL_GPIO_Init(stepper.dir_port, &GPIO_InitStruct);

    /* Enable DWT cycle counter for microsecond delays */
    CL57T_InitDWT();
}

void CL57T_SetDirection(CL57T_Direction_t dir)
{
    if (dir == CL57T_DIR_CCW) {
        HAL_GPIO_WritePin(stepper.dir_port, stepper.dir_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(stepper.dir_port, stepper.dir_pin, GPIO_PIN_RESET);
    }
    s_last_dir = (int8_t)dir;
}

void CL57T_Step(void)
{
    /* Rising edge -- CL57T steps on the rising edge of PUL */
    HAL_GPIO_WritePin(stepper.pul_port, stepper.pul_pin, GPIO_PIN_SET);

    /* Hold HIGH for minimum pulse width */
    CL57T_DelayUs(CL57T_PULSE_WIDTH_US);

    /* Falling edge */
    HAL_GPIO_WritePin(stepper.pul_port, stepper.pul_pin, GPIO_PIN_RESET);
}

void CL57T_MoveSteps(uint32_t steps, CL57T_Direction_t dir, uint32_t delay_us)
{
    /* Only re-latch direction + eat the setup delay if the direction has
     * actually changed. Back-to-back jog bursts in the same direction then
     * run continuously instead of stuttering every burst. */
    if ((int8_t)dir != s_last_dir) {
        CL57T_SetDirection(dir);
        CL57T_DelayUs(CL57T_DIR_SETUP_US);
    }

    /* Generate step pulses */
    for (uint32_t i = 0; i < steps; i++) {
        CL57T_Step();
        CL57T_DelayUs(delay_us);
    }
}

void CL57T_Jog(CL57T_Direction_t dir, uint32_t delay_us, volatile int *cancel,
               CL57T_StopPredicate_t stop_predicate)
{
    if (delay_us < 5U) {
        delay_us = 5U;
    }

    /* Latch direction only if it actually changed (same cache logic as
     * MoveSteps). When the user taps I then K in quick succession, this
     * eats the full DIR setup delay on the direction change and skips it
     * if they're just re-pressing the same key. */
    if ((int8_t)dir != s_last_dir) {
        CL57T_SetDirection(dir);
        CL57T_DelayUs(CL57T_DIR_SETUP_US);
    }

    /* Tight continuous step loop. The cancel pointer is typically the
     * UART command-ready flag from uart_cmd.c, which the RX ISR sets
     * on any newline. So any incoming command (STEPPER JOG STOP, X,
     * DEV OFF, anything) breaks this loop cleanly — we never deadlock. */
    while (cancel == NULL || *cancel == 0) {
        if (stop_predicate != NULL && stop_predicate()) {
            break;
        }
        CL57T_Step();
        CL57T_DelayUs(delay_us);
    }
}

const CL57T_Stepper_t* CL57T_GetStepper(void)
{
    return &stepper;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize DWT cycle counter for microsecond-resolution delays
 *
 * The DWT (Data Watchpoint and Trace) unit has a free-running cycle counter
 * (CYCCNT) that counts CPU clock cycles. This gives us precise timing
 * without consuming a hardware timer peripheral.
 *
 * On Cortex-M4, the DWT is always present. We just need to:
 *   1. Enable the trace unit (DEMCR.TRCENA)
 *   2. Reset the cycle counter
 *   3. Enable the cycle counter (DWT_CTRL.CYCCNTENA)
 */
static void CL57T_InitDWT(void)
{
    /* Enable trace and debug block (required for DWT) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Reset cycle counter */
    DWT->CYCCNT = 0;

    /* Enable cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  Busy-wait delay with microsecond resolution using DWT
 *
 * Uses the DWT cycle counter to measure elapsed time.
 * This is more accurate than HAL_Delay() (which has 1ms resolution)
 * and doesn't consume a timer peripheral.
 *
 * At 47.333 MHz, one microsecond = ~47.3 cycles.
 *
 * @param  us  Delay duration in microseconds
 */
static void CL57T_DelayUs(uint32_t us)
{
    uint32_t clk_freq = HAL_RCC_GetHCLKFreq();
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (clk_freq / 1000000U) * us;

    while ((DWT->CYCCNT - start) < cycles) {
        /* Busy wait -- the subtraction handles counter wraparound
         * correctly due to unsigned arithmetic */
    }
}
