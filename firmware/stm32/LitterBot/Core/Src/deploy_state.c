/**
 * @file    deploy_state.c
 * @brief   Deploy/standby state machine implementation
 *
 * See deploy_state.h for full documentation.
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "deploy_state.h"
#include "arm_control.h"

#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/

static volatile DeployState_t g_state    = DEPLOY_STANDBY;
static volatile bool          g_dev_mode = false;

/* Private helpers -----------------------------------------------------------*/

static void print_uart(UART_HandleTypeDef *huart, const char *msg)
{
    HAL_UART_Transmit(huart, (uint8_t *)msg, strlen(msg), 100);
}

/* Public functions ----------------------------------------------------------*/

void DeployState_Init(void)
{
    g_state    = DEPLOY_STANDBY;
    g_dev_mode = false;
}

DeployState_t DeployState_Get(void)
{
    return g_state;
}

const char* DeployState_GetStateString(void)
{
    switch (g_state) {
        case DEPLOY_STANDBY:   return "STANDBY";
        case DEPLOY_DEPLOYING: return "DEPLOYING";
        case DEPLOY_ACTIVE:    return "ACTIVE";
        default:               return "UNKNOWN";
    }
}

bool DeployState_CommandAllowed(char cmd_char)
{
    /* Dev mode bypasses everything */
    if (g_dev_mode) return true;

    /* Always-allowed commands */
    if (cmd_char == 'X') return true;   /* E-stop */
    if (cmd_char == 'Q') return true;   /* State query */
    if (cmd_char == 'T') return true;   /* Encoder test */

    /* All other commands require ACTIVE state */
    return (g_state == DEPLOY_ACTIVE);
}

bool DeployState_IsDevMode(void)
{
    return g_dev_mode;
}

void DeployState_SetDevMode(bool on)
{
    g_dev_mode = on;
}

DeployResult_t DeployState_StartDeploy(UART_HandleTypeDef *huart)
{
    if (g_state == DEPLOY_DEPLOYING) {
        return DEPLOY_ERR_ALREADY;
    }

    g_state = DEPLOY_DEPLOYING;
    print_uart(huart, "DEPLOY: Homing arm...\r\n");

    ArmResult_t result = Arm_Home(huart);

    if (result != ARM_OK) {
        /* Arm homing failed — warn but still go ACTIVE so teleop works.
         * This allows the robot to be operated when arm hardware is not yet
         * connected or calibrated. Re-enable strict homing once arm is tested. */
        char buf[60];
        snprintf(buf, sizeof(buf), "DEPLOY: arm warn %d — proceeding anyway\r\n", (int)result);
        print_uart(huart, buf);
    }

    g_state = DEPLOY_ACTIVE;
    print_uart(huart, "DEPLOY: ACTIVE\r\n");
    return DEPLOY_OK;
}

void DeployState_ReturnToStandby(void)
{
    g_state = DEPLOY_STANDBY;
}
