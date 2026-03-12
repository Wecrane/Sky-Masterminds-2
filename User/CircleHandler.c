/**
 * @file    CircleHandler.c
 * @brief   圆环元素检测与处理模块实现 (仅左环)
 * @details 基于传感器分布特征检测 + 陀螺仪角度积分
 *
 *          流程:
 *          1. 发车后 COOLDOWN_SECONDS 秒冷却 (每秒打印倒计时)
 *          2. 冷却结束后加强左侧权重 + 开始角度积分
 *          3. 角度偏移超过 30° 后恢复默认权重 (已进入环内)
 *          4. 累计 330° 后加强右侧权重准备出环 (持续 2s)
 *          5. 2s 后恢复正常权重并降速到基础速度 90
 *          6. 2s 后开始检测停车区域 (十字路口横线)
 *          7. 检测到 3 根横线后立即抱死停车
 *          8. 停车 3s 后退出巡线模式
 */

#include "CircleHandler.h"
#include "BlackPoint_Finder.h"
#include "LSM6DSR_Config.h"
#include "ADC_get.h"
#include "PID_Controller.h"
#include "Motor_ctr.h"
#include "M3PWM.h"
#include "RGB_Led.h"
#include "ABEncoder.h"
#include "Odometer.h"
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

/* 冷却时间 (秒), 修改此处即可调整冷却时长 */
#define COOLDOWN_SECONDS         20
#define COOLDOWN_MS              (COOLDOWN_SECONDS * 1000)

/* 入环角度阈值: 偏移超过此角度后恢复默认权重 */
#define CIRCLE_ENTER_ANGLE       30.0f   /* 度 */

/* 出环角度 */
#define CIRCLE_EXIT_ANGLE        330.0f  /* 累积转角达到此值后开始出环 (度) */

/* 出环偏向引导 */
#define EXIT_DURATION_MS         2000    /* 出环右侧权重持续 2s */

/* 陀螺仪积分 */
#define CIRCLE_DT                0.002f
#define CIRCLE_GYRO_DEADZONE     0.003f
#define CIRCLE_RAD_TO_DEG        57.2957795f

/* 流程时间参数 */
#define PARKING_ENABLE_DELAY_MS  2000    /* 出环后 2s 开始检测停车 */
#define PARKING_STOP_DURATION_MS 3000    /* 停车持续 3s */
#define CROSSLINE_TARGET_COUNT   3       /* 需要检测到的横线数量 */

/* 出环后降速 */
#define POST_EXIT_BASE_SPEED     90.0f   /* 出环后基础速度 */


/* ========================================================================== */
/*                              序列阶段枚举                                   */
/* ========================================================================== */
typedef enum {
	PHASE_LEFT_WAIT = 0,        /* 冷却倒计时 */
	PHASE_LEFT_BIAS,            /* 冷却结束, 左侧权重加强 + 角度积分, 等待 30° */
	PHASE_DONE                  /* 左环完成, 等待停车 */
} CirclePhase_t;

/* ========================================================================== */
/*                              内部状态变量                                   */
/* ========================================================================== */
static CircleState_t s_state = CIRCLE_STATE_NORMAL;

/* 序列阶段 */
static CirclePhase_t s_phase = PHASE_LEFT_WAIT;
static uint8_t s_current_dir = CIRCLE_DIR_LEFT;

/* 角度积分 */
static float s_accumulated_angle = 0.0f;

/* 出环计时 */
static uint32_t s_exit_start_ms = 0;

/* 时序计时 */
static uint32_t s_car_start_ms = 0;       /* star_car 启动时间 */
static uint32_t s_left_exit_ms = 0;       /* 左环出环时间 */

/* 上一次 star_car 状态 (边沿检测) */
static uint8_t s_was_running = 0;

/* 编码器角度融合 (参考 Odometer) */
static int32_t s_last_left_cnt = 0;
static int32_t s_last_right_cnt = 0;

/* 停车检测 */
static uint8_t s_parking_detection_enabled = 0;
static uint8_t s_crossline_count = 0;
static uint8_t s_on_crossline = 0;        /* 当前正在横线上 (消抖) */
static uint8_t s_parking_stopped = 0;     /* 已触发停车 */
static uint32_t s_parking_stop_ms = 0;    /* 停车开始时间 */

/* 调试打印限流 */
static uint32_t s_last_print_ms = 0;

/* 倒计时打印 */
static uint32_t s_last_countdown_s = 0;   /* 上次打印的秒数 */

/* ========================================================================== */
/*                              公有函数实现                                   */
/* ========================================================================== */

void Circle_Init(void)
{
	s_state = CIRCLE_STATE_NORMAL;
	s_phase = PHASE_LEFT_WAIT;
	s_current_dir = CIRCLE_DIR_LEFT;
	s_accumulated_angle = 0.0f;
	s_exit_start_ms = 0;
	s_car_start_ms = 0;
	s_left_exit_ms = 0;
	s_was_running = 0;
	s_parking_detection_enabled = 0;
	s_crossline_count = 0;
	s_on_crossline = 0;
	s_parking_stopped = 0;
	s_parking_stop_ms = 0;
	s_last_print_ms = 0;
	s_last_countdown_s = 0;
	s_last_left_cnt = left_ecoder_cnt;
	s_last_right_cnt = right_ecoder_cnt;
	printf("[CIRCLE] Init: LEFT only, cooldown=%ds\r\n", COOLDOWN_SECONDS);
}

void Circle_Reset(void)
{
	if(s_state != CIRCLE_STATE_NORMAL)
	{
		printf("[CIRCLE] Reset (was %d)\r\n", s_state);
	}
	s_state = CIRCLE_STATE_NORMAL;
	s_phase = PHASE_LEFT_WAIT;
	s_current_dir = CIRCLE_DIR_LEFT;
	s_accumulated_angle = 0.0f;
	s_car_start_ms = 0;
	s_left_exit_ms = 0;
	s_was_running = 0;
	s_parking_detection_enabled = 0;
	s_crossline_count = 0;
	s_on_crossline = 0;
	s_parking_stopped = 0;
	s_last_print_ms = 0;
	s_last_countdown_s = 0;
	s_last_left_cnt = left_ecoder_cnt;
	s_last_right_cnt = right_ecoder_cnt;
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
	float delta_theta;
	
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
		s_phase = PHASE_LEFT_WAIT;
		s_last_countdown_s = 0;
		s_parking_detection_enabled = 0;
		s_crossline_count = 0;
		s_on_crossline = 0;
		printf("[CIRCLE] Car started, waiting %ds\r\n", COOLDOWN_SECONDS);
	}
	
	/* ======== 冷却期间每秒打印倒计时 ======== */
	if(s_phase == PHASE_LEFT_WAIT)
	{
		uint32_t elapsed_s = (g_systick_ms - s_car_start_ms) / 1000;
		if(elapsed_s > s_last_countdown_s && elapsed_s <= (uint32_t)COOLDOWN_SECONDS)
		{
			s_last_countdown_s = elapsed_s;
			printf("[CIRCLE] Countdown: %d s\r\n", (int)elapsed_s);
		}
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
	
	/* ======== 停车检测启用定时 (左环出环后 2s) ======== */
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
	
	/* ======== 角度增量计算 (编码器差速, 参考 Odometer) ======== */
	{
		int32_t left_delta  = left_ecoder_cnt  - s_last_left_cnt;
		int32_t right_delta = right_ecoder_cnt - s_last_right_cnt;
		s_last_left_cnt  = left_ecoder_cnt;
		s_last_right_cnt = right_ecoder_cnt;
		
		float left_dist  = (float)left_delta  / ODOM_ENCODER_PULSE_PER_MM;
		float right_dist = (float)right_delta / ODOM_ENCODER_PULSE_PER_MM;
		
		/* 编码器差速 → 角度变化 (度) */
		delta_theta = (right_dist - left_dist) / ODOM_WHEEL_BASE_MM
		              * CIRCLE_RAD_TO_DEG;
		
		/* 陀螺仪读数 (仅调试打印用) */
		gz_filtered = LSE6DSR_data.gz_rads;
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
					/* 等待冷却结束 */
					if((g_systick_ms - s_car_start_ms) >= COOLDOWN_MS)
					{
						s_phase = PHASE_LEFT_BIAS;
						s_state = CIRCLE_STATE_ENTERING;
						s_accumulated_angle = 0.0f;
						s_last_left_cnt = left_ecoder_cnt;
						s_last_right_cnt = right_ecoder_cnt;
						printf("[CIRCLE] %ds done, left bias ON, angle tracking start\r\n", COOLDOWN_SECONDS);
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
			
			/* 积分角度 (编码器+陀螺仪融合) */
			s_accumulated_angle += delta_theta;
			
			/* 每500ms打印角度, 方便调试 */
			if((g_systick_ms - s_last_print_ms) >= 500)
			{
				s_last_print_ms = g_systick_ms;
				printf("[ENTERING] angle=%.1f gz=%.4f\r\n", s_accumulated_angle, gz_filtered);
			}
			
			/* 角度超过 30° → 已进入环内, 恢复默认权重 */
			if(fabsf(s_accumulated_angle) >= CIRCLE_ENTER_ANGLE)
			{
				s_state = CIRCLE_STATE_IN_CIRCLE;
				printf("[CIRCLE] >> IN_CIRCLE at %.1f deg, left bias OFF\r\n", fabsf(s_accumulated_angle));
			}
			break;
		}
		
		/* ================================================================ */
		case CIRCLE_STATE_IN_CIRCLE:
		{
			/* 正常全幅巡线 + 角度积分 (编码器+陀螺仪融合) */
			s_accumulated_angle += delta_theta;
			
			/* 每500ms打印角度, 方便调试 */
			if((g_systick_ms - s_last_print_ms) >= 500)
			{
				s_last_print_ms = g_systick_ms;
				printf("[IN_CIRCLE] angle=%.1f\r\n", s_accumulated_angle);
			}
			
			/* 角度够了 (330°) → EXITING, 加强右侧权重 */
			if(fabsf(s_accumulated_angle) >= CIRCLE_EXIT_ANGLE)
			{
				s_state = CIRCLE_STATE_EXITING;
				s_exit_start_ms = g_systick_ms;
				printf("[CIRCLE] >> EXITING at %.1f deg, right bias 2s\r\n", fabsf(s_accumulated_angle));
			}
			break;
		}
		
		/* ================================================================ */
		case CIRCLE_STATE_EXITING:
		{
			/* 角度继续积分 (调试用, 编码器+陀螺仪融合) */
			s_accumulated_angle += delta_theta;
			
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
			
			/* 出环引导 2s 到 → 恢复正常权重, 降速到 90 */
			if((g_systick_ms - s_exit_start_ms) >= EXIT_DURATION_MS)
			{
				s_state = CIRCLE_STATE_NORMAL;
				s_accumulated_angle = 0.0f;
				
				/* 恢复正常权重, 降速到 90, 2s 后启动停车检测 */
				s_left_exit_ms = g_systick_ms;
				s_phase = PHASE_DONE;
				PID_SetLineSpeedParams(POST_EXIT_BASE_SPEED, POST_EXIT_BASE_SPEED);
				printf("[CIRCLE] << LEFT DONE, weight restored, speed=90, parking in 2s\r\n");
			}
			break;
		}
		
		default:
			s_state = CIRCLE_STATE_NORMAL;
			break;
	}
}
