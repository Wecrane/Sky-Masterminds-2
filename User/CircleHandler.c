/**
 * @file    CircleHandler.c
 * @brief   圆环元素检测与处理模块实现 (仅左环)
 * @details 基于编码器累计里程触发, 纯里程控制, 无角度积分
 *
 *          流程:
 *          1. 里程达到入环距离 → 左侧半侧追线
 *          2. 里程达到出环偏向距离 → 右侧半侧追线准备出环
 *          3. 里程达到出环距离 → 出环完成, 恢复正常
 *          4. 出环后开始检测停车区域 (十字路口横线)
 *          5. 检测到 3 根横线后立即抱死停车
 *          6. 停车 3s 后退出巡线模式
 */

#include "CircleHandler.h"
#include "BlackPoint_Finder.h"
#include "ADC_get.h"
#include "PID_Controller.h"
#include "Motor_ctr.h"
#include "M3PWM.h"
#include "RGB_Led.h"
#include "Odometer.h"
#include "SpeedProfile.h"
#include <stdio.h>

/* ========================================================================== */
/*                              外部变量引用                                   */
/* ========================================================================== */
extern int16_t position_get;
extern volatile uint32_t g_systick_ms;
extern volatile uint32_t g_lose_time;
extern uint8_t star_car;

/* ========================================================================== */
/*                              可调参数                                       */
/* ========================================================================== */

/* ---- 里程触发阈值 (mm, 根据实测数据) ---- */
/* 实测入环: (39992 + 40060) / 2 = 40026, 提前200mm开始左侧追线 */
#define CIRCLE_ENTRY_DISTANCE_MM   39800.0f

/* 实测出环: 45322, 出环右侧追线结束距离 */
#define CIRCLE_EXIT_DISTANCE_MM    48000.0f

/* 出环引导切换距离: 里程达到此值后切换为右侧追线准备出环 */
#define CIRCLE_EXIT_BIAS_MM      45000.0f

/* 流程时间参数 */
#define PARKING_ENABLE_DELAY_MS  2000    /* 出环后 2s 开始检测停车 */
#define PARKING_STOP_DURATION_MS 3000    /* 停车持续 3s */
#define CROSSLINE_TARGET_COUNT   3       /* 需要检测到的横线数量 */


/* ========================================================================== */
/*                              序列阶段枚举                                   */
/* ========================================================================== */
typedef enum {
	PHASE_LEFT_WAIT = 0,        /* 等待里程达到入环距离 */
	PHASE_LEFT_BIAS,            /* 左侧权重加强 + 角度积分 */
	PHASE_DONE                  /* 左环完成, 等待停车 */
} CirclePhase_t;

/* ========================================================================== */
/*                              内部状态变量                                   */
/* ========================================================================== */
static CircleState_t s_state = CIRCLE_STATE_NORMAL;

/* 序列阶段 */
static CirclePhase_t s_phase = PHASE_LEFT_WAIT;

/* 时序计时 */
static uint32_t s_left_exit_ms = 0;       /* 左环出环时间 */

/* 上一次 star_car 状态 (边沿检测) */
static uint8_t s_was_running = 0;

/* 停车检测 */
static uint8_t s_parking_detection_enabled = 0;
static uint8_t s_crossline_count = 0;
static uint8_t s_on_crossline = 0;        /* 当前正在横线上 (消抖) */
static uint8_t s_parking_stopped = 0;     /* 已触发停车 */
static uint32_t s_parking_stop_ms = 0;    /* 停车开始时间 */

/* 调试打印限流 */
static uint32_t s_last_print_ms = 0;

/* ========================================================================== */
/*                              公有函数实现                                   */
/* ========================================================================== */

void Circle_Init(void)
{
	s_state = CIRCLE_STATE_NORMAL;
	s_phase = PHASE_LEFT_WAIT;
	s_left_exit_ms = 0;
	s_was_running = 0;
	s_parking_detection_enabled = 0;
	s_crossline_count = 0;
	s_on_crossline = 0;
	s_parking_stopped = 0;
	s_parking_stop_ms = 0;
	s_last_print_ms = 0;
}

void Circle_Reset(void)
{
	if(s_state != CIRCLE_STATE_NORMAL)
	{
		printf("[CIRCLE] reset\r\n");
	}
	s_state = CIRCLE_STATE_NORMAL;
	s_phase = PHASE_LEFT_WAIT;
	s_left_exit_ms = 0;
	s_was_running = 0;
	s_parking_detection_enabled = 0;
	s_crossline_count = 0;
	s_on_crossline = 0;
	s_parking_stopped = 0;
	s_last_print_ms = 0;
}

CircleState_t Circle_GetState(void)
{
	return s_state;
}

uint8_t Circle_IsActive(void)
{
	return (s_state != CIRCLE_STATE_NORMAL) ? 1 : 0;
}

/**
 * @brief  检测横线 (十字路口停车区域)
 * @param  adc_values  传感器 ADC 值数组
 * @return 1=检测到横线, 0=未检测到
 * @details 利用传感器圆弧形状特征:
 *          对称外侧传感器对同时检测到黑线 → 横线
 *          检查 (0,15), (1,14), (2,13) 三对, 至少 2 对同时为黑
 */
static uint8_t Circle_DetectCrossLine(volatile uint16_t *adc_values)
{
	uint8_t pair_count = 0;
	
	if(BlackPoint_Finder_IsBlackPoint(0, adc_values[0]) &&
	   BlackPoint_Finder_IsBlackPoint(15, adc_values[15]))
		pair_count++;
	
	if(BlackPoint_Finder_IsBlackPoint(1, adc_values[1]) &&
	   BlackPoint_Finder_IsBlackPoint(14, adc_values[14]))
		pair_count++;
	
	if(BlackPoint_Finder_IsBlackPoint(2, adc_values[2]) &&
	   BlackPoint_Finder_IsBlackPoint(13, adc_values[13]))
		pair_count++;
	
	return (pair_count >= 2) ? 1 : 0;
}

/**
 * @brief  圆环状态机主更新函数
 */
void Circle_Update(volatile uint16_t *adc_values)
{
	BlackPointResult_t half_result;
	
	if(adc_values == NULL) return;
	
	/* ======== 停车状态处理 (最高优先级) ======== */
	if(s_parking_stopped)
	{
		if((g_systick_ms - s_parking_stop_ms) >= PARKING_STOP_DURATION_MS)
		{
			/* 3s 停车完成, 退出巡线模式 */
			s_parking_stopped = 0;
			star_car = 0;
			WheelLock_Disable();
			Motor_StopAll();
			Motor_Disable();
			M3PWM_SetDutyCycle(0);  /* 关闭负压风扇 */
			RGB_SetColor(RGB_COLOR_OFF);
			PID_PositionLoop_Enable(0);
			SpeedPID_ResetState();
			printf("[PKG] done\r\n");
		}
		return;
	}
	
	/* ======== 急停/退出保护 ======== */
	if(!star_car)
	{
		if(s_state != CIRCLE_STATE_NORMAL)
		{
			s_state = CIRCLE_STATE_NORMAL;
		}
		s_was_running = 0;
		return;
	}
	
	/* ======== 启动边沿检测: star_car 0→1 ======== */
	if(!s_was_running)
	{
		s_was_running = 1;
		s_phase = PHASE_LEFT_WAIT;
		s_parking_detection_enabled = 0;
		s_crossline_count = 0;
		s_on_crossline = 0;
		Odometer_Reset();         /* 里程清零, 配合分段减速 */
		SpeedProfile_Reset();     /* 路段索引归零 */
		printf("[CIRCLE] start, entry@%.0fmm\r\n", CIRCLE_ENTRY_DISTANCE_MM);
	}
	
	/* ======== 停车区域检测 (左环出环后 2s 启用) ======== */
	if(s_parking_detection_enabled && s_state == CIRCLE_STATE_NORMAL)
	{
		uint8_t cross = Circle_DetectCrossLine(adc_values);
		
		if(cross)
		{
			if(!s_on_crossline)
			{
				/* 上升沿: 进入横线 */
				s_on_crossline = 1;
				s_crossline_count++;
				
				if(s_crossline_count >= CROSSLINE_TARGET_COUNT)
				{
					/* 达到 3 根横线, 立即抱死停车 */
					s_parking_stopped = 1;
					s_parking_stop_ms = g_systick_ms;
					s_parking_detection_enabled = 0;
					WheelLock_Enable();
					printf("[PKG] line 3/3 -> STOP\r\n");
					return;
				}
				printf("[PKG] line %d/3\r\n", s_crossline_count);
			}
		}
		else
		{
			s_on_crossline = 0;
		}
	}
	
	/* ======== 停车检测启用定时 (左环出环后 2s) ======== */
	if(s_phase == PHASE_DONE && !s_parking_detection_enabled && s_left_exit_ms != 0)
	{
		if((g_systick_ms - s_left_exit_ms) >= PARKING_ENABLE_DELAY_MS)
		{
			s_parking_detection_enabled = 1;
			s_crossline_count = 0;
			s_on_crossline = 0;

		}
	}
	
	/* ======== 状态机 ======== */
	switch(s_state)
	{
		/* ================================================================ */
		case CIRCLE_STATE_NORMAL:
		{
			/* 根据当前阶段决定行为 */
			switch(s_phase)
			{
				case PHASE_LEFT_WAIT:
				/* 等待里程达到入环距离 */
				if(Odometer_GetLocation() >= CIRCLE_ENTRY_DISTANCE_MM)
				{
					s_phase = PHASE_LEFT_BIAS;
					s_state = CIRCLE_STATE_ENTERING;
				printf("[CIRCLE] %.0fmm: enter (left)\r\n", Odometer_GetLocation());
				}
				break;
				
				case PHASE_LEFT_BIAS:
					/* 不应在 NORMAL 状态下出现此阶段 */
					break;
				
				case PHASE_DONE:
					/* 左环已完成, 不再处理 */
					break;
			}
			break;
		}
		
		/* ================================================================ */
		case CIRCLE_STATE_ENTERING:
		{
			/* 左半侧追线: 直线正常走, 岔路时自动选左侧 */
			BlackPoint_Finder_SearchHalf(adc_values, 0, &half_result);
			if(half_result.found)
			{
				position_get = (int16_t)(half_result.precise_position * 10.0f);
				g_lose_time = 0;
			}
			else
			{
				/* 半侧丢线, 全幅兜底 */
				BlackPointResult_t full_result;
				BlackPoint_Finder_Search(adc_values, &full_result);
				if(full_result.found)
				{
					position_get = (int16_t)(full_result.precise_position * 10.0f);
					g_lose_time = 0;
				}
			}
			
			/* 里程达到出环偏向距离 → 切换右侧追线准备出环 */
			{
				float loc = Odometer_GetLocation();
				if(loc >= CIRCLE_EXIT_BIAS_MM)
				{
					s_state = CIRCLE_STATE_EXITING;
					printf("[CIRCLE] %.0fmm: exit (right)\r\n", loc);
				}
			}
			break;
		}
		
		/* ================================================================ */
		case CIRCLE_STATE_EXITING:
		{
			/* 出环引导: 右半侧追线, 岔路时自动选右侧 */
			BlackPoint_Finder_SearchHalf(adc_values, 1, &half_result);
			if(half_result.found)
			{
				position_get = (int16_t)(half_result.precise_position * 10.0f);
				g_lose_time = 0;
			}
			else
			{
				/* 半侧丢线, 全幅兜底 */
				BlackPointResult_t full_result;
				BlackPoint_Finder_Search(adc_values, &full_result);
				if(full_result.found)
				{
					position_get = (int16_t)(full_result.precise_position * 10.0f);
					g_lose_time = 0;
				}
			}
			
			/* 里程达到出环距离 → 出环完成 */
			{
				float loc = Odometer_GetLocation();
				if(loc >= CIRCLE_EXIT_DISTANCE_MM)
				{
					s_state = CIRCLE_STATE_NORMAL;
					s_left_exit_ms = g_systick_ms;
					s_phase = PHASE_DONE;
					printf("[CIRCLE] %.0fmm: done\r\n", loc);
				}
			}
			break;
		}
		
		default:
			s_state = CIRCLE_STATE_NORMAL;
			break;
	}
}
