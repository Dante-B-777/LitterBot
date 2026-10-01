/**
 * @file    odometry.c
 * @brief   Mecanum wheel odometry implementation
 *
 * Converts encoder tick deltas into body-frame velocities using
 * mecanum forward kinematics, then integrates into a global pose
 * using Euler integration (first-order dead reckoning).
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "odometry.h"
#include "encoder.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/

/** @brief Pi constant */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/** @brief 2*Pi for full rotation */
#define TWO_PI (2.0f * M_PI)

/* Private variables ---------------------------------------------------------*/

/** @brief Current pose estimate */
static Odometry_Pose_t pose;

/** @brief Timestamp of last update for dt calculation */
static uint32_t last_update_ms;

/* Public functions ----------------------------------------------------------*/

void Odometry_Init(void)
{
    pose.x     = 0.0f;
    pose.y     = 0.0f;
    pose.theta = 0.0f;
    pose.vx    = 0.0f;
    pose.vy    = 0.0f;
    pose.omega = 0.0f;

    last_update_ms = HAL_GetTick();
}

void Odometry_Update(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - last_update_ms;

    if (elapsed_ms == 0) {
        return;  /* No time has passed, skip */
    }

    float dt = (float)elapsed_ms / 1000.0f;
    last_update_ms = now_ms;

    /* Get encoder deltas (ticks since last Encoder_Update) */
    const Encoder_Data_t *fl = Encoder_Get(ENC_FL);
    const Encoder_Data_t *fr = Encoder_Get(ENC_FR);
    const Encoder_Data_t *rl = Encoder_Get(ENC_RL);
    const Encoder_Data_t *rr = Encoder_Get(ENC_RR);

    if (!fl || !fr || !rl || !rr) {
        return;
    }

    /*
     * Convert tick deltas to wheel angular velocities (rad/s).
     *
     * wheel_rad_per_tick = 2*pi / ENCODER_TICKS_PER_REV
     * w_i = delta_ticks * wheel_rad_per_tick / dt
     */
    float rad_per_tick = TWO_PI / ENCODER_TICKS_PER_REV;

    float w_fl = (float)fl->delta * rad_per_tick / dt;
    float w_fr = (float)fr->delta * rad_per_tick / dt;
    float w_rl = (float)rl->delta * rad_per_tick / dt;
    float w_rr = (float)rr->delta * rad_per_tick / dt;

    /*
     * Mecanum forward kinematics:
     *   vx    = (r/4) * ( w_fl + w_fr + w_rl + w_rr)
     *   vy    = (r/4) * (-w_fl + w_fr + w_rl - w_rr)
     *   omega = (r / (4*(L+W))) * (-w_fl + w_fr - w_rl + w_rr)
     */
    float r = ODOM_WHEEL_RADIUS_M;
    float r_over_4 = r / 4.0f;
    float lw_sum = ODOM_WHEELBASE_M + ODOM_TRACK_WIDTH_M;  /* L + W */

    pose.vx    = r_over_4 * ( w_fl + w_fr + w_rl + w_rr);
    pose.vy    = r_over_4 * (-w_fl + w_fr + w_rl - w_rr);
    pose.omega = (r / (4.0f * lw_sum)) * (-w_fl + w_fr - w_rl + w_rr);

    /*
     * Pose integration (Euler method, body-to-world rotation):
     *   x     += (vx*cos(theta) - vy*sin(theta)) * dt
     *   y     += (vx*sin(theta) + vy*cos(theta)) * dt
     *   theta += omega * dt
     */
    float cos_theta = cosf(pose.theta);
    float sin_theta = sinf(pose.theta);

    pose.x     += (pose.vx * cos_theta - pose.vy * sin_theta) * dt;
    pose.y     += (pose.vx * sin_theta + pose.vy * cos_theta) * dt;
    pose.theta += pose.omega * dt;

    /* Normalize theta to [-pi, pi] */
    while (pose.theta > M_PI)  pose.theta -= TWO_PI;
    while (pose.theta < -M_PI) pose.theta += TWO_PI;
}

const Odometry_Pose_t* Odometry_GetPose(void)
{
    return &pose;
}

void Odometry_Reset(void)
{
    pose.x     = 0.0f;
    pose.y     = 0.0f;
    pose.theta = 0.0f;
    pose.vx    = 0.0f;
    pose.vy    = 0.0f;
    pose.omega = 0.0f;

    last_update_ms = HAL_GetTick();
}
