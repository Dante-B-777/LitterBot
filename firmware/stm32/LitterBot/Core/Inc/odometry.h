/**
 * @file    odometry.h
 * @brief   Mecanum wheel odometry for dead-reckoning pose estimation
 *
 * Computes the robot's position (x, y) and heading (theta) from
 * encoder deltas using mecanum forward kinematics and Euler integration.
 *
 * Mecanum forward kinematics (body velocities from wheel speeds):
 *   vx    = (r/4) * (w_fl + w_fr + w_rl + w_rr)
 *   vy    = (r/4) * (-w_fl + w_fr + w_rl - w_rr)
 *   omega = (r / (4*(L+W))) * (-w_fl + w_fr - w_rl + w_rr)
 *
 * Where:
 *   r   = wheel radius (meters)
 *   L   = half wheelbase (front-to-rear axle center / 2)
 *   W   = half track width (left-to-right wheel center / 2)
 *   w_i = wheel angular velocity (rad/s)
 *
 * Coordinate frame:
 *   x     = forward (positive = robot drives forward)
 *   y     = left (positive = robot strafes left)
 *   theta = heading (positive = counter-clockwise from start)
 *
 * IMPORTANT: The wheelbase and track width constants must be measured
 *            on the actual robot chassis and updated below.
 *
 * Usage:
 *   Odometry_Init();
 *   // in main loop:
 *   Encoder_Update();    // must be called first
 *   Odometry_Update();   // uses encoder deltas
 *   const Odometry_Pose_t *pose = Odometry_GetPose();
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Configurable constants ----------------------------------------------------*/

/** @brief Wheel radius in meters (104mm diameter / 2) */
#define ODOM_WHEEL_RADIUS_M     0.052f

/**
 * @brief Half wheelbase in meters (front-to-rear axle center distance / 2)
 * Measured: 336 mm full wheelbase -> 168 mm half = 0.168 m
 */
#define ODOM_WHEELBASE_M        0.168f

/**
 * @brief Half track width in meters (left-to-right wheel center distance / 2)
 * Measured: 405 mm full track width -> 202.5 mm half = 0.2025 m
 */
#define ODOM_TRACK_WIDTH_M      0.2025f

/* Pose data structure -------------------------------------------------------*/

/**
 * @brief Robot pose and velocity from wheel odometry
 */
typedef struct {
    float x;       /**< Position forward from start (meters) */
    float y;       /**< Position left from start (meters) */
    float theta;   /**< Heading CCW from start (radians) */
    float vx;      /**< Forward velocity (m/s) */
    float vy;      /**< Lateral velocity, left positive (m/s) */
    float omega;   /**< Angular velocity, CCW positive (rad/s) */
} Odometry_Pose_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Initialize odometry module
 *
 * Zeros the pose and velocity estimates. Call after Encoder_Init().
 */
void Odometry_Init(void);

/**
 * @brief  Update odometry from latest encoder data
 *
 * Reads encoder deltas (set by Encoder_Update()), converts to wheel
 * angular velocities, applies mecanum forward kinematics, and
 * integrates pose using Euler method.
 *
 * Must be called after Encoder_Update() each cycle.
 */
void Odometry_Update(void);

/**
 * @brief  Get current pose estimate
 *
 * @return Pointer to internal pose struct (valid until next Odometry_Update)
 */
const Odometry_Pose_t* Odometry_GetPose(void);

/**
 * @brief  Reset pose to origin (0, 0, 0)
 */
void Odometry_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H */
