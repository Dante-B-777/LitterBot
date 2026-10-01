/**
 * @file    deploy_state.h
 * @brief   Deploy/standby state machine for safety gating
 *
 * On boot, the robot is in STANDBY — all motor/arm/scoop commands
 * are rejected by the STM32. The operator must explicitly deploy
 * via the web UI (5-second press-and-hold), which triggers arm
 * homing and transitions to ACTIVE.
 *
 * State machine:
 *   STANDBY ──(D DEPLOY)──► DEPLOYING ──(homing OK)──► ACTIVE
 *      ▲                        │                         │
 *      │                    (homing fail)                  │
 *      └────────────────────────┘         (return to standby)
 *      └─────────────────────────────────────────────────-┘
 *
 * Dev mode: When enabled, all commands pass through regardless
 * of state. Toggle via DEV ON / DEV OFF commands.
 *
 * Always-allowed commands (never gated):
 *   X   — E-stop
 *   Q   — State query
 *   T   — Encoder test (harmless debug)
 *   DEV — Dev mode toggle
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef DEPLOY_STATE_H
#define DEPLOY_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdbool.h>

/* State enumeration ---------------------------------------------------------*/

typedef enum {
    DEPLOY_STANDBY   = 0,   /**< Boot default. Gated commands rejected. */
    DEPLOY_DEPLOYING = 1,   /**< Arm homing in progress. */
    DEPLOY_ACTIVE    = 2    /**< Normal operation. All commands accepted. */
} DeployState_t;

/* Deploy result codes -------------------------------------------------------*/

typedef enum {
    DEPLOY_OK             =  0,   /**< Deploy completed successfully */
    DEPLOY_ERR_HOMING     = -1,   /**< Arm homing failed */
    DEPLOY_ERR_ALREADY    = -2    /**< Already deploying */
} DeployResult_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Initialize deploy state to STANDBY, dev mode off
 */
void DeployState_Init(void);

/**
 * @brief  Get the current deploy state
 */
DeployState_t DeployState_Get(void);

/**
 * @brief  Get the current state as a string ("STANDBY", "DEPLOYING", "ACTIVE")
 */
const char* DeployState_GetStateString(void);

/**
 * @brief  Check if a command character is allowed in the current state
 *
 * Always allowed: X (e-stop), Q (query), T (encoder test)
 * Gated: M (motors), S (scoop), C (calibrate), D (deploy only from STANDBY)
 * Dev mode bypasses all gating.
 *
 * @param  cmd_char  First character of the command line
 * @return true if allowed, false if blocked
 */
bool DeployState_CommandAllowed(char cmd_char);

/**
 * @brief  Check if dev mode is enabled
 */
bool DeployState_IsDevMode(void);

/**
 * @brief  Toggle dev mode on or off
 */
void DeployState_SetDevMode(bool on);

/**
 * @brief  Start the deploy sequence (blocking)
 *
 * Transitions STANDBY → DEPLOYING, runs arm homing, then
 * transitions to ACTIVE on success or back to STANDBY on failure.
 *
 * @param  huart  LPUART1 handle for status messages
 * @return DEPLOY_OK on success, or error code
 */
DeployResult_t DeployState_StartDeploy(UART_HandleTypeDef *huart);

/**
 * @brief  Return to STANDBY state (for re-deploy)
 *
 * Immediately sets state to STANDBY. Caller should stop motors
 * and initiate arm homing separately if needed.
 */
void DeployState_ReturnToStandby(void);

#ifdef __cplusplus
}
#endif

#endif /* DEPLOY_STATE_H */
