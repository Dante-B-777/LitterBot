/**
 * @file    calibration_store.c
 * @brief   Persistent calibration storage implementation
 */

#include "calibration_store.h"
#include "scoop_sequence.h"

#include <string.h>

typedef struct {
    uint32_t magic;
    uint32_t extend_steps;
    uint32_t valid_mask;
    uint32_t reserved0;
    uint32_t steps[CAL_POSE_COUNT];
    uint16_t pos[CAL_POSE_COUNT];
    uint16_t reserved1[CAL_POSE_COUNT];
} CalStoreImage_t;

static uint32_t g_extend_steps;
static uint32_t g_valid_mask;
static uint16_t g_pos[CAL_POSE_COUNT];
static uint32_t g_step[CAL_POSE_COUNT];

static void load_defaults(void)
{
    uint32_t i;

    g_extend_steps = SCOOP_EXTEND_STEPS;
    g_valid_mask = 0U;

    for (i = 0; i < CAL_POSE_COUNT; i++) {
        g_pos[i] = SCOOP_POS_HOME;
        g_step[i] = 0U;
    }

    g_pos[CAL_POS_HOME] = SCOOP_POS_HOME;
    g_pos[CAL_POS_OPEN] = SCOOP_POS_OPEN;
    g_pos[CAL_POS_CLOSED] = SCOOP_POS_CLOSED;
    g_pos[CAL_POS_DUMP] = SCOOP_POS_DUMP;
}

static void recompute_max_steps(void)
{
    uint32_t i;
    uint32_t max_steps = 0U;

    for (i = 0; i < CAL_POSE_COUNT; i++) {
        if (g_step[i] > max_steps) {
            max_steps = g_step[i];
        }
    }

    g_extend_steps = (max_steps > 0U) ? max_steps : SCOOP_EXTEND_STEPS;
}

static void write_to_flash(void)
{
    CalStoreImage_t image;
    uint32_t offset;

    memset(&image, 0, sizeof(image));
    image.magic = CAL_MAGIC;
    image.extend_steps = g_extend_steps;
    image.valid_mask = g_valid_mask;
    memcpy(image.steps, g_step, sizeof(g_step));
    memcpy(image.pos, g_pos, sizeof(g_pos));

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = CAL_FLASH_BANK,
        .Page      = CAL_FLASH_PAGE,
        .NbPages   = 1
    };
    uint32_t page_error = 0U;
    HAL_FLASHEx_Erase(&erase, &page_error);

    for (offset = 0; offset < sizeof(image); offset += 8U) {
        uint64_t word = 0ULL;
        memcpy(&word, ((uint8_t *)&image) + offset, 8U);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR + offset, word);
    }

    HAL_FLASH_Lock();
}

void CalStore_Init(void)
{
    CalStoreImage_t image;

    load_defaults();
    memcpy(&image, (const void *)CAL_FLASH_ADDR, sizeof(image));

    if (image.magic != CAL_MAGIC) {
        return;
    }

    g_extend_steps = image.extend_steps;
    g_valid_mask = image.valid_mask & ((1UL << CAL_POSE_COUNT) - 1UL);
    memcpy(g_step, image.steps, sizeof(g_step));
    memcpy(g_pos, image.pos, sizeof(g_pos));
    recompute_max_steps();
}

uint32_t CalStore_GetExtendSteps(void)
{
    return g_extend_steps;
}

uint16_t CalStore_GetServoPos(CalServoPos_t which)
{
    if ((uint32_t)which >= CAL_POSE_COUNT) {
        return g_pos[CAL_POS_HOME];
    }
    return g_pos[which];
}

uint32_t CalStore_GetPoseSteps(CalServoPos_t which)
{
    if ((uint32_t)which >= CAL_POSE_COUNT) {
        return g_step[CAL_POS_HOME];
    }
    return g_step[which];
}

uint32_t CalStore_GetValidMask(void)
{
    return g_valid_mask;
}

uint8_t CalStore_PoseIsSaved(CalServoPos_t which)
{
    if ((uint32_t)which >= CAL_POSE_COUNT) {
        return 0U;
    }
    return (g_valid_mask & (1UL << (uint32_t)which)) ? 1U : 0U;
}

void CalStore_SaveExtendSteps(uint32_t steps)
{
    g_extend_steps = steps;
    write_to_flash();
}

void CalStore_SaveServoPos(CalServoPos_t which, uint16_t pos)
{
    if ((uint32_t)which >= CAL_POSE_COUNT) {
        return;
    }
    g_pos[which] = pos;
    g_valid_mask |= (1UL << (uint32_t)which);
    write_to_flash();
}

void CalStore_SavePose(CalServoPos_t which, uint32_t steps, uint16_t pos)
{
    if ((uint32_t)which >= CAL_POSE_COUNT) {
        return;
    }
    g_step[which] = steps;
    g_pos[which] = pos;
    g_valid_mask |= (1UL << (uint32_t)which);
    recompute_max_steps();
    write_to_flash();
}
