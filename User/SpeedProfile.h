/**
 * @file    SpeedProfile.h
 * @brief   固定路线分段减速模块
 * @details 基于编码器累计里程实现前瞻性分段调速。
 *          在到达弯道前提前减速，过弯后逐步加速。
 *
 *          使用方法:
 *          1. 在 SpeedProfile.c 中修改 s_route[] 路线表
 *          2. 发车前调用 SpeedProfile_Reset() (已集成到 CircleHandler)
 *          3. PID 控制循环自动调用 SpeedProfile_GetTargetSpeed()
 */

#ifndef __SPEED_PROFILE_H__
#define __SPEED_PROFILE_H__

#include "stm32f10x.h"

/**
 * @brief  路段定义结构体
 * @note   distance_mm: 此路段起始里程 (mm, 从发车点 0 算起)
 *         speed:       此路段的目标速度 (编码器单位/2ms)
 */
typedef struct {
    float distance_mm;
    float speed;
} RouteWaypoint_t;

/**
 * @brief  初始化速度规划模块
 */
void SpeedProfile_Init(void);

/**
 * @brief  重置路段索引 (发车时调用, 配合里程计清零)
 */
void SpeedProfile_Reset(void);

/**
 * @brief  获取当前里程对应的目标速度
 * @param  current_distance_mm  当前累计里程 (mm)
 * @return 当前路段的目标速度, 含前瞻减速和加速恢复效果
 */
float SpeedProfile_GetTargetSpeed(float current_distance_mm);

/**
 * @brief  启用/禁用速度规划
 * @param  en  1=启用, 0=禁用 (禁用时返回默认高速)
 */
void SpeedProfile_Enable(uint8_t en);

/**
 * @brief  查询速度规划是否启用
 * @return 1=启用, 0=禁用
 */
uint8_t SpeedProfile_IsEnabled(void);

#endif /* __SPEED_PROFILE_H__ */
