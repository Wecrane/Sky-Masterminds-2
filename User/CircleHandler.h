#ifndef __CIRCLE_HANDLER_H__
#define __CIRCLE_HANDLER_H__

#include "stm32f10x.h"
#include <stdint.h>

/**
 * @file    CircleHandler.h
 * @brief   圆环元素检测与处理模块
 * @details 使用状态机 + 里程触发实现圆环的驶入、绕行、驶出
 *
 *          状态转移:
 *          NORMAL → 里程达到入环阈値 → ENTERING
 *          ENTERING → 里程达到出环偷向阈値 → EXITING
 *          EXITING → 里程达到出环阈値 → NORMAL
 */

/* 圆环状态机状态 */
typedef enum {
	CIRCLE_STATE_NORMAL = 0,    /* 正常巡线 */
	CIRCLE_STATE_ENTERING,      /* 正在驶入圆环 (左側半幅追线) */
	CIRCLE_STATE_IN_CIRCLE,     /* 保留，实际不使用 */
	CIRCLE_STATE_EXITING        /* 正在驶出圆环 (右側半幅追线) */
} CircleState_t;

/**
 * @brief  初始化圆环处理模块
 */
void Circle_Init(void);

/**
 * @brief  重置圆环状态机为 NORMAL
 */
void Circle_Reset(void);

/**
 * @brief  获取当前圆环状态
 * @return 当前状态
 */
CircleState_t Circle_GetState(void);

/**
 * @brief  圆环状态机更新 (在 SysTick 中断中调用, 2ms 周期)
 * @param  adc_values  传感器 ADC 值数组
 */
void Circle_Update(volatile uint16_t *adc_values);

/**
 * @brief  查询当前是否处于圆环处理模式 (非 NORMAL)
 * @return 1=正在处理圆环, 0=正常巡线
 */
uint8_t Circle_IsActive(void);

#endif /* __CIRCLE_HANDLER_H__ */
