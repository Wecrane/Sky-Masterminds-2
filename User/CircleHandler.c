/**
 * @file    CircleHandler.c
 * @brief   圆环元素检测与处理模块实现
 * @details 先右环后左环，基于传感器分布特征检测 + 陀螺仪角度积分
 *
 *          流程:
 *          1. 发车后 1s 开始检测右侧入环
 *          2. 右环出环后冷却 13s
 *          3. 开始检测左侧入环
 *          4. 左环出环后不再检测任何环
 *          5. 左环出环 4s 后开始检测停车区域 (十字路口横线)
 *          6. 检测到 3 根横线后立即抱死停车
 *          7. 停车 3s 后退出巡线模式
 */

#include "CircleHandler.h"
#include "BlackPoint_Finder.h"
#include "LSM6DSR_Config.h"
#include "ADC_get.h"
#include "PID_Controller.h"
#include "Motor_ctr.h"
#include "M3PWM.h"
#include "RGB_Led.h"
#include <math.h>
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

/* 入环检测 */
#define ENTRY_CONFIRM_CNT        12      /* 连续帧确认 */

/* 入环偏向引导 */
#define ENTERING_DURATION_MS     2000    /* 半侧追线持续时间 (ms) */

/* 出环角度 */
#define CIRCLE_EXIT_ANGLE        320.0f  /* 累积转角达到此值后开始出环 (度) */

/* 出环偏向引导 */
#define EXIT_DURATION_MS         1000    /* 半侧追线持续时间 (ms) */

/* 陀螺仪积分 */
#define CIRCLE_DT                0.002f
#define CIRCLE_GYRO_DEADZONE     0.003f
#define CIRCLE_RAD_TO_DEG        57.2957795f

/* 流程时间参数 */
#define RIGHT_ENABLE_DELAY_MS    1000    /* 发车后 1s 开始检测右环 */
#define RIGHT_COOLDOWN_MS        4000   /* 右环出环后冷却 4s */
#define PARKING_ENABLE_DELAY_MS  2000    /* 左环出环后 2s 开始检测停车 */
#define PARKING_STOP_DURATION_MS 3000    /* 停车持续 3s */
#define CROSSLINE_TARGET_COUNT   3       /* 需要检测到的横线数量 */

/* ========================================================================== */
/*                              序列阶段枚举                                   */
/* ========================================================================== */
typedef enum {
	PHASE_RIGHT_WAIT = 0,       /* 等待 1s 后开始右环检测 */
	PHASE_RIGHT_READY,          /* 可以检测右环入口 */
	PHASE_RIGHT_COOLDOWN,       /* 右环出环后 4s 冷却 */
	PHASE_LEFT_READY,           /* 可以检测左环入口 */
	PHASE_DONE                  /* 双环完成, 等待停车 */
} CirclePhase_t;

/* ========================================================================== */
/*                              内部状态变量                                   */
/* ========================================================================== */
static CircleState_t s_state = CIRCLE_STATE_NORMAL;

/* 序列阶段 */
static CirclePhase_t s_phase = PHASE_RIGHT_WAIT;
static uint8_t s_current_dir = CIRCLE_DIR_RIGHT;

/* 入环检测消抖 */
static uint8_t  s_entry_confirm_cnt = 0;

/* 角度积分 */
static float s_accumulated_angle = 0.0f;

/* 入环/出环计时 */
static uint32_t s_enter_start_ms = 0;
static uint32_t s_exit_start_ms = 0;

/* 时序计时 */
static uint32_t s_car_start_ms = 0;       /* star_car 启动时间 */
static uint32_t s_right_exit_ms = 0;      /* 右环出环时间 */
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
	s_phase = PHASE_RIGHT_WAIT;
	s_current_dir = CIRCLE_DIR_RIGHT;
	s_entry_confirm_cnt = 0;
	s_accumulated_angle = 0.0f;
	s_enter_start_ms = 0;
	s_exit_start_ms = 0;
	s_car_start_ms = 0;
	s_right_exit_ms = 0;
	s_left_exit_ms = 0;
	s_was_running = 0;
	s_parking_detection_enabled = 0;
	s_crossline_count = 0;
	s_on_crossline = 0;
	s_parking_stopped = 0;
	s_parking_stop_ms = 0;
	s_last_print_ms = 0;
	printf("[CIRCLE] Init: RIGHT then LEFT\r\n");
}

void Circle_Reset(void)
{
	if(s_state != CIRCLE_STATE_NORMAL)
	{
		printf("[CIRCLE] Reset (was %d)\r\n", s_state);
	}
	s_state = CIRCLE_STATE_NORMAL;
	s_phase = PHASE_RIGHT_WAIT;
	s_current_dir = CIRCLE_DIR_RIGHT;
	s_entry_confirm_cnt = 0;
	s_accumulated_angle = 0.0f;
	s_car_start_ms = 0;
	s_right_exit_ms = 0;
	s_left_exit_ms = 0;
	s_was_running = 0;
	s_parking_detection_enabled = 0;
	s_crossline_count = 0;
	s_on_crossline = 0;
	s_parking_stopped = 0;
	s_last_print_ms = 0;
}

void Circle_SetDirection(uint8_t dir)
{
	(void)dir; /* 方向由阶段自动决定 */
}

uint8_t Circle_GetDirection(void)
{
	return s_current_dir;
}

CircleState_t Circle_GetState(void)
{
	return s_state;
}

uint8_t Circle_IsActive(void)
{
	return (s_state != CIRCLE_STATE_NORMAL) ? 1 : 0;
}

float Circle_GetAccumulatedAngle(void)
{
	return fabsf(s_accumulated_angle);
}

/**
 * @brief  检测左环入口特征
 * @param  adc_values  传感器 ADC 值数组
 * @return 1=符合左环入口特征, 0=不符合
 */
static uint8_t Circle_DetectLeftEntry(volatile uint16_t *adc_values)
{
	uint8_t left_count = 0;
	uint8_t right_count = 0;
	uint8_t center_count = 0;
	uint8_t i;
	
	/* 左侧5路 */
	for(i = 0; i <= 4; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			left_count++;
	}
	
	/* 右侧4路 */
	for(i = 12; i <= 15; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			right_count++;
	}
	
	/* 中间区域 */
	for(i = 6; i <= 9; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			center_count++;
	}
	
	return (left_count >= 3 && right_count == 0 && center_count >= 1) ? 1 : 0;
}

/**
 * @brief  检测右环入口特征 (左环的镜像)
 * @param  adc_values  传感器 ADC 值数组
 * @return 1=符合右环入口特征, 0=不符合
 */
static uint8_t Circle_DetectRightEntry(volatile uint16_t *adc_values)
{
	uint8_t right_count = 0;
	uint8_t left_count = 0;
	uint8_t center_count = 0;
	uint8_t i;
	
	/* 右侧5路 (11~15) */
	for(i = 11; i <= 15; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			right_count++;
	}
	
	/* 左侧4路 (0~3) */
	for(i = 0; i <= 3; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			left_count++;
	}
	
	/* 中间区域 (6~9) */
	for(i = 6; i <= 9; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			center_count++;
	}
	
	return (right_count >= 3 && left_count == 0 && center_count >= 1) ? 1 : 0;
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
	float gz_filtered;
	
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
			printf("[PARKING] 3s done, exit line-following\r\n");
		}
		return;
	}
	
	/* ======== 急停/退出保护 ======== */
	if(!star_car)
	{
		if(s_state != CIRCLE_STATE_NORMAL)
		{
			s_state = CIRCLE_STATE_NORMAL;
			s_entry_confirm_cnt = 0;
			s_accumulated_angle = 0.0f;
		}
		s_was_running = 0;
		return;
	}
	
	/* ======== 启动边沿检测: star_car 0→1 ======== */
	if(!s_was_running)
	{
		s_was_running = 1;
		s_car_start_ms = g_systick_ms;
		s_phase = PHASE_RIGHT_WAIT;
		s_parking_detection_enabled = 0;
		s_crossline_count = 0;
		s_on_crossline = 0;
		printf("[CIRCLE] Car started, waiting 1s for right circle\r\n");
	}
	
	/* ======== 停车区域检测 (左环出环后 4s 启用) ======== */
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
				printf("[PARKING] Crossline %d/%d\r\n", s_crossline_count, CROSSLINE_TARGET_COUNT);
				
				if(s_crossline_count >= CROSSLINE_TARGET_COUNT)
				{
					/* 达到 3 根横线, 立即抱死停车 */
					s_parking_stopped = 1;
					s_parking_stop_ms = g_systick_ms;
					s_parking_detection_enabled = 0;
					WheelLock_Enable();
					printf("[PARKING] 3 lines reached, STOP!\r\n");
					return;
				}
			}
		}
		else
		{
			s_on_crossline = 0;
		}
	}
	
	/* ======== 停车检测启用定时 (左环出环后 4s) ======== */
	if(s_phase == PHASE_DONE && !s_parking_detection_enabled && s_left_exit_ms != 0)
	{
		if((g_systick_ms - s_left_exit_ms) >= PARKING_ENABLE_DELAY_MS)
		{
			s_parking_detection_enabled = 1;
			s_crossline_count = 0;
			s_on_crossline = 0;
			printf("[PARKING] Detection enabled\r\n");
		}
	}
	
	/* ======== 陀螺仪死区滤波 ======== */
	gz_filtered = LSE6DSR_data.gz_rads;
	if(gz_filtered > -CIRCLE_GYRO_DEADZONE && gz_filtered < CIRCLE_GYRO_DEADZONE)
	{
		gz_filtered = 0.0f;
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
				case PHASE_RIGHT_WAIT:
					/* 等待发车后 1s */
					if((g_systick_ms - s_car_start_ms) >= RIGHT_ENABLE_DELAY_MS)
					{
						s_phase = PHASE_RIGHT_READY;
						printf("[CIRCLE] Right circle detection enabled\r\n");
					}
					break;
				
				case PHASE_RIGHT_READY:
					/* 检测右环入口 */
					if(Circle_DetectRightEntry(adc_values))
					{
						s_entry_confirm_cnt++;
						if(s_entry_confirm_cnt >= ENTRY_CONFIRM_CNT)
						{
							s_state = CIRCLE_STATE_ENTERING;
							s_current_dir = CIRCLE_DIR_RIGHT;
							s_enter_start_ms = g_systick_ms;
							s_accumulated_angle = 0.0f;
							s_entry_confirm_cnt = 0;
							printf("[CIRCLE] >> ENTERING (RIGHT)\r\n");
						}
					}
					else
					{
						s_entry_confirm_cnt = 0;
					}
					break;
				
				case PHASE_RIGHT_COOLDOWN:
					/* 右环出环后冷却 13s */
					if((g_systick_ms - s_right_exit_ms) >= RIGHT_COOLDOWN_MS)
					{
						s_phase = PHASE_LEFT_READY;
						printf("[CIRCLE] Right cooldown done, left detection enabled\r\n");
					}
					break;
				
				case PHASE_LEFT_READY:
					/* 检测左环入口 */
					if(Circle_DetectLeftEntry(adc_values))
					{
						s_entry_confirm_cnt++;
						if(s_entry_confirm_cnt >= ENTRY_CONFIRM_CNT)
						{
							s_state = CIRCLE_STATE_ENTERING;
							s_current_dir = CIRCLE_DIR_LEFT;
							s_enter_start_ms = g_systick_ms;
							s_accumulated_angle = 0.0f;
							s_entry_confirm_cnt = 0;
							printf("[CIRCLE] >> ENTERING (LEFT)\r\n");
						}
					}
					else
					{
						s_entry_confirm_cnt = 0;
					}
					break;
				
				case PHASE_DONE:
					/* 双环已完成, 不再检测入环 */
					break;
			}
			break;
		}
		
		/* ================================================================ */
		case CIRCLE_STATE_ENTERING:
		{
			/* 入环引导: 右环→右半侧追线(side=1), 左环→左半侧追线(side=0) */
			uint8_t enter_side = (s_current_dir == CIRCLE_DIR_RIGHT) ? 1 : 0;
			BlackPoint_Finder_SearchHalf(adc_values, enter_side, &half_result);
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
			
			/* 积分角度 */
			s_accumulated_angle += gz_filtered * CIRCLE_DT * CIRCLE_RAD_TO_DEG;
			
			/* 入环引导时间到 → IN_CIRCLE */
			if((g_systick_ms - s_enter_start_ms) >= ENTERING_DURATION_MS)
			{
				s_state = CIRCLE_STATE_IN_CIRCLE;
			}
			break;
		}
		
		/* ================================================================ */
		case CIRCLE_STATE_IN_CIRCLE:
		{
			/* 正常全幅巡线 + 角度积分 */
			s_accumulated_angle += gz_filtered * CIRCLE_DT * CIRCLE_RAD_TO_DEG;
			
			/* 角度够了 → EXITING */
			if(fabsf(s_accumulated_angle) >= CIRCLE_EXIT_ANGLE)
			{
				s_state = CIRCLE_STATE_EXITING;
				s_exit_start_ms = g_systick_ms;
			}
			break;
		}
		
		/* ================================================================ */
		case CIRCLE_STATE_EXITING:
		{
			/* 角度继续积分 (调试用) */
			s_accumulated_angle += gz_filtered * CIRCLE_DT * CIRCLE_RAD_TO_DEG;
			
			/* 出环引导: 右环→左半侧追线(side=0), 左环→右半侧追线(side=1) */
			{
				uint8_t exit_side = (s_current_dir == CIRCLE_DIR_RIGHT) ? 0 : 1;
				BlackPoint_Finder_SearchHalf(adc_values, exit_side, &half_result);
			}
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
			
			/* 出环引导时间到 → 完成 */
			if((g_systick_ms - s_exit_start_ms) >= EXIT_DURATION_MS)
			{
				s_state = CIRCLE_STATE_NORMAL;
				s_accumulated_angle = 0.0f;
				
				if(s_current_dir == CIRCLE_DIR_RIGHT)
				{
					/* 右环出环 → 进入冷却阶段 */
					s_right_exit_ms = g_systick_ms;
					s_phase = PHASE_RIGHT_COOLDOWN;
					printf("[CIRCLE] << RIGHT DONE, cooldown\r\n");
				}
				else
				{
					/* 左环出环 → 完成, 启动停车倒计时 */
					s_left_exit_ms = g_systick_ms;
					s_phase = PHASE_DONE;
					printf("[CIRCLE] << LEFT DONE, parking in 4s\r\n");
				}
			}
			break;
		}
		
		default:
			s_state = CIRCLE_STATE_NORMAL;
			break;
	}
}
