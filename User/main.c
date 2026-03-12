/**
 * @file    main.c
 * @brief   巡线小车主程序
 * @details 实现以下功能:
 *          - K1: 启动/停止巡线模式
 *          - K2: 启动/停止蓝牙遥控模式
 *          - K3: 打印传感器状态 (调试用)
 *          - K4: 急停/解除急停
 *          
 *          控制架构:
 *          - 巡线模式: 位置环 + 速度环双环 PID 控制
 *          - 蓝牙模式: 仅速度环控制, 接收蓝牙指令
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "M3PWM.h"
#include "Motor_ctr.h"
#include "ABEncoder.h"
#include "ADC_get.h"
#include "BTComm.h"
#include "Uart_Config.h"
#include "stdio.h"
#include "math.h"
#include "RGB_Led.h"
#include "LSM6DSR_Config.h"
#include "pose.h"
#include "Key_Scan.h"
#include "system_stm32f10x.h"
#include "BlackPoint_Finder.h"
#include "PID_Controller.h"
#include "Odometer.h"
#include "CircleHandler.h"
#include "SpeedProfile.h"

/* ========================================================================== */
/*                              外部变量引用                                   */
/* ========================================================================== */
extern float add_angle;
extern float add_angle_num;
extern int16_t position_get;
extern int16_t speed_left;
extern int16_t speed_right;
extern BlackPointResult_t result_BlackPoint;
extern volatile uint32_t g_systick_ms;
extern volatile uint32_t g_lose_time;            /* 丢线计数器 */
extern volatile uint32_t g_main_loop_watchdog;   /* 主循环看门狗 */

/* ========================================================================== */
/*                              全局变量定义                                   */
/* ========================================================================== */
uint8_t star_car = 0;                        /* 巡线模式启动标志: 0=停止, 1=运行 */
volatile uint8_t g_bt_key_control_mode = 0;  /* 蓝牙遥控模式标志 */
float BDI_V = 0;                             /* 电池电压 (V) */

/* ========================================================================== */
/*                          低电压警告相关定义                                  */
/* ========================================================================== */
#define LOW_BATTERY_THRESHOLD     7.4f       /* 低电压阈值 (V), 2S锂电池最低安全电压 */
#define LOW_BATTERY_RECOVER_TH    7.6f       /* 电压恢复阈值 (V), 滞回防抖 */
#define LOW_BATTERY_LED_PERIOD_MS 300        /* LED闪烁周期 (ms) */
#define LOW_BATTERY_MSG_PERIOD_MS 5000       /* 低电压消息发送周期 (ms) */
#define LOW_BATTERY_DEBOUNCE_CNT  2500        /* 消抖计数, 约5000ms连续低电压才触发 */

static uint8_t g_low_battery_flag = 0;       /* 低电压标志 */
static uint32_t g_low_battery_led_timer = 0; /* LED闪烁定时器 */
static uint32_t g_low_battery_msg_timer = 0; /* 消息发送定时器 */
static uint8_t g_low_battery_led_state = 0;  /* LED当前状态: 0=灭, 1=亮 */
static uint16_t g_low_battery_debounce = 0;  /* 低电压消抖计数器 */

/* ========================================================================== */
/*                           光电管校准相关定义                                 */
/* ========================================================================== */
static uint8_t g_calibrating = 0;              /* 校准状态: 0=未校准, 1=正在校准 */
static uint16_t g_cal_min[SENSOR_COUNT];       /* 校准期间各路最小值 */
static uint16_t g_cal_max[SENSOR_COUNT];       /* 校准期间各路最大值 */

/* ========================================================================== */
/*                              系统初始化                                     */
/* ========================================================================== */

/**
 * @brief  配置 SysTick 定时器
 * @note   2ms 中断周期, 用于 PID 控制和传感器采样
 */
static void SysTick_Init(void)
{
	SysTick_Config(SystemCoreClock / 500);
}

/* ========================================================================== */
/*                           蓝牙遥控辅助函数                                  */
/* ========================================================================== */

/**
 * @brief  将速度百分比转换为目标速度值
 * @param  speed_percent  速度百分比 (0-100)
 * @return 目标速度 (编码器单位)
 */
static float BT_SpeedPercentToTargetSpeed(uint8_t speed_percent)
{
	if(speed_percent > 100) speed_percent = 100;
	return (230.0f * (float)speed_percent) / 100.0f;
}

/**
 * @brief  将速度百分比转换为 PWM 占空比上限
 * @param  speed_percent  速度百分比 (0-100)
 * @return PWM 占空比
 * @note   速度环需要足够的 PWM 裕量来克服负载达到目标速度，
 *         因此 duty_cap 应该足够大，让速度环有调节空间
 */
static uint16_t BT_SpeedPercentToDuty(uint8_t speed_percent)
{
	if(speed_percent == 0) return 0;
	/* 只要有目标速度，就给足够大的 PWM 裕量让速度环工作 */
	return 8000;  /* 与速度环 output_max 一致 */
}

/**
 * @brief  应用蓝牙运动指令
 * @param  state  蓝牙状态结构体指针
 * @note   包含软启动斜坡控制, 防止电机突然加速
 *         IDLE状态时启用轮子锁定，防止因重力滑落
 */
static void BT_ApplyMotion(const BT_State_t *state)
{
	static uint16_t ramp_duty = 0;
	static uint32_t last_ms = 0;
	static Motion_Mode_t last_mode = MOTION_IDLE;

	if(state == NULL) return;

	/* 急停状态: 立即停止所有电机，禁用锁定 */
	if(state->emergency_stop)
	{
		ramp_duty = 0;
		last_mode = MOTION_IDLE;
		WheelLock_Disable();
		Motor_StopAll();
		Motor_Disable();
		PID_SetBluetoothTarget(0.0f, 0.0f, 0);
		SpeedPID_ResetState();
		return;
	}

	/* 空闲状态: 启用轮子锁定，保持位置不动 */
	if(state->motion_mode == MOTION_IDLE)
	{
		/* 从运动状态切换到空闲，启用锁定 */
		if(last_mode != MOTION_IDLE || !WheelLock_IsEnabled())
		{
			ramp_duty = 0;
			SpeedPID_ResetState();
			WheelLock_Enable();  /* 记录当前位置并启用锁定 */
		}
		last_mode = MOTION_IDLE;
		PID_SetBluetoothTarget(0.0f, 0.0f, 0);
		/* 锁定控制在 PID_Control_Update 中执行 */
		return;
	}

	/* 进入运动状态: 禁用轮子锁定 */
	if(WheelLock_IsEnabled())
	{
		WheelLock_Disable();
	}

	/* 模式切换: 重置斜坡 */
	if(state->motion_mode != last_mode)
	{
		ramp_duty = 0;
		last_mode = state->motion_mode;
		SpeedPID_ResetState();
	}

	/* 计算目标速度和占空比上限 */
	float target_speed = BT_SpeedPercentToTargetSpeed(state->motion_speed);
	uint16_t duty_cap = BT_SpeedPercentToDuty(state->motion_speed);

	/* 软启动斜坡控制 */
	uint32_t now_ms = g_systick_ms;
	uint32_t elapsed_ms = now_ms - last_ms;
	if(elapsed_ms > 100) elapsed_ms = 2;  /* 防止异常大的时间间隔 */
	last_ms = now_ms;

	uint32_t max_delta = elapsed_ms * 50U;
	if(max_delta == 0U) max_delta = 50U;

	if(ramp_duty < duty_cap)
	{
		uint16_t delta = (duty_cap - ramp_duty) > max_delta ? max_delta : (duty_cap - ramp_duty);
		ramp_duty += delta;
	}
	else if(ramp_duty > duty_cap)
	{
		uint16_t delta = (ramp_duty - duty_cap) > max_delta ? max_delta : (ramp_duty - duty_cap);
		ramp_duty -= delta;
	}

	Motor_Enable();

	/* 根据运动模式设置左右轮目标速度 */
	float left_target = 0.0f;
	float right_target = 0.0f;

	switch(state->motion_mode)
	{
		case MOTION_FORWARD:
			left_target = target_speed;
			right_target = target_speed;
			break;
		case MOTION_BACKWARD:
			left_target = -target_speed;
			right_target = -target_speed;
			break;
		case MOTION_TURN_LEFT:
			left_target = -target_speed * 0.5f;
			right_target = target_speed * 0.5f;
			break;
		case MOTION_TURN_RIGHT:
			left_target = target_speed * 0.5f;
			right_target = -target_speed * 0.5f;
			break;
		case MOTION_IDLE:
		default:
			Motor_StopAll();
			return;
	}

	/* 传递目标值给 PID 模块, 实际输出在 SysTick 中断中执行 */
	PID_SetBluetoothTarget(left_target, right_target, ramp_duty);
}

/* ========================================================================== */
/*                              按键事件处理                                   */
/* ========================================================================== */

/**
 * @brief  处理按键事件
 * @param  event  按键事件指针
 */
static void HandleKeyEvent(Key_Event_t *event)
{
	if(event == NULL) return;

	switch(event->key_id)
	{
		case KEY_NONE:
			break;

		/* K1: 巡线模式启停 */
		case KEY_K1:
			/* 键控模式下不允许启动巡线 */
			if(g_bt_key_control_mode) break;
			
			if(star_car)
			{
				/* 停止巡线 */
				star_car = 0;
				Motor_StopAll();
				Motor_Disable();
				M3PWM_SetDutyCycle(0);  /* 停止负压风扇 */
				PID_PositionLoop_Enable(0);
				SpeedPID_ResetState();
				Circle_Reset();  /* 重置圆环状态机 */
				RGB_SetColor(RGB_COLOR_OFF);
			}
			else
			{
				/* 启动巡线 - 自动清除急停状态 */
				if(BT_IsEmergencyStopped())
				{
					BT_ClearEmergencyStop();
				}
				
				/* 禁用轮子锁定 (可能从键控模式切换过来) */
				WheelLock_Disable();
				
				RGB_SetColor(RGB_COLOR_G);
				star_car = 1;
				g_lose_time = 0;  /* 清零丢线计数 */
				SpeedPID_ResetState();
				PositionPID_ResetState();
				Circle_Reset();  /* 重置圆环状态机 */
				Odometer_Reset();  /* 重置里程计，以当前位置为坐标原点 */
				Motor_Enable();
				/* 负压风扇: 使用蓝牙设置的转速，默认20% */
				{
					uint8_t vac_speed = BT_GetVacuumSpeed();
					if(vac_speed == 0) vac_speed = 0;  /* 默认10% */
					uint16_t duty = (1000 * vac_speed) / 100;
					M3PWM_SetDutyCycle(duty);
				}
				M3PWM_Start();
				PID_PositionLoop_Enable(1);
			}
			break;

		/* K2: 蓝牙遥控模式启停 */
		case KEY_K2:
			/* 巡线模式下不允许进入蓝牙模式 */
			if(star_car) break;
			
			if(g_bt_key_control_mode)
			{
				/* 退出蓝牙模式 */
				g_bt_key_control_mode = 0;
				BT_HandleKeyControlModeChange(0);
				WheelLock_Disable();  /* 禁用轮子锁定 */
				Motor_StopAll();
				Motor_Disable();
				SpeedPID_ResetState();
				RGB_SetColor(RGB_COLOR_OFF);
			}
			else
			{
				/* 进入蓝牙模式 - 自动清除急停状态 */
				if(BT_IsEmergencyStopped())
				{
					BT_ClearEmergencyStop();
				}
				
				g_bt_key_control_mode = 1;
				star_car = 0;
				PID_PositionLoop_Enable(0);
				BT_HandleKeyControlModeChange(1);
				SpeedPID_ResetState();
				Motor_Enable();
				WheelLock_Enable();  /* 启用轮子锁定 (默认锁定状态) */
				RGB_SetColor(RGB_COLOR_B);
			}
			break;

		/* K3: 光电管校准 (仅空闲状态可用) */
		case KEY_K3:
			if(star_car || g_bt_key_control_mode || BT_IsEmergencyStopped())
			{
				printf("[CAL] ERR: only available in idle state\r\n");
				break;
			}
			if(!g_calibrating)
			{
				/* 第一次按下: 开始校准 */
				uint8_t ci;
				g_calibrating = 1;
				for(ci = 0; ci < SENSOR_COUNT; ci++)
				{
					g_cal_min[ci] = 4095;  /* 12-bit ADC最大值 */
					g_cal_max[ci] = 0;
				}
				printf("[CAL] Calibration started. Move sensors over black and white...\r\n");
				RGB_SetColor(RGB_COLOR_B);  /* 蓝色提示校准中 */
			}
			else
			{
				/* 第二次按下: 结束校准, 计算并应用阈值 */
				uint8_t ci;
				g_calibrating = 0;
				printf("\r\n[CAL] Calibration finished!\r\n");
				printf("CH  |  Min  |  Max  | Threshold\r\n");
				printf("----|-------|-------|----------\r\n");
				for(ci = 0; ci < SENSOR_COUNT; ci++)
				{
					/* 阈值偏向黑色(min)侧35%, 给白色留更多抖动裕度 */
					uint16_t thr = g_cal_min[ci] + (uint16_t)((g_cal_max[ci] - g_cal_min[ci]) * 0.35f);
					BlackPoint_Finder_SetThreshold(ci, thr);
					printf("%2d  |  %4d |  %4d |   %4d\r\n", ci, g_cal_min[ci], g_cal_max[ci], thr);
				}
				printf("[CAL] Thresholds applied!\r\n");
				RGB_SetColor(RGB_COLOR_OFF);
			}
			break;

		/* K4: 急停/解除急停 */
		case KEY_K4:
			if(BT_IsEmergencyStopped())
			{
				/* 解除急停 */
				BT_ClearEmergencyStop();
				SpeedPID_ResetState();
				RGB_SetColor(RGB_COLOR_OFF);
			}
			else
			{
				/* 触发急停 */
				RGB_SetColor(RGB_COLOR_R);
				star_car = 0;
				g_bt_key_control_mode = 0;
				BT_HandleKeyControlModeChange(0);
				SpeedPID_ResetState();
				Circle_Reset();
				BT_EmergencyStop();
			}
			break;
	}
}

/* ========================================================================== */
/*                              主函数                                         */
/* ========================================================================== */

int main(void)
{
	/* ====== 外设初始化 ====== */
	RGB_Init();
	LSM6DSR_Init();
	M3PWM_Init();
	SysTick_Init();
	Motor_Init();
	M3PWM_Start();
	ABEncoder_Init();
	BlackPoint_Finder_Init();
	MuxADC_Init();
	Key_Scan_Init();
	Uart2_Init(115200);
	BT_Init();
	PID_Init();
	Odometer_Init();  /* 初始化里程计模块 */
	SpeedProfile_Init();  /* 初始化分段减速模块 */
	Circle_Init();    /* 初始化圆环检测模块 */
	
	/* ====== 初始状态: 等待按键启动 ====== */
	PID_PositionLoop_Enable(0);

	/* ====== 主循环 ====== */
	while(1)
	{
		/* 喂狗: 主循环正在运行 */
		g_main_loop_watchdog = g_systick_ms;
		
		/* 按键扫描与蓝牙处理 */
		Key_Scan_Update();
		BT_Process();
		
		/* 读取电池电压
		 * 校准说明: 真实7.36V时显示7.76V, 偏高0.4V
		 * 以低电压(7.4V附近)为基准调整系数: 0.0038 * (7.36/7.76) ≈ 0.00360 */
		BDI_V = (float)g_mux_adc_values[16] * 0.003605f;
		
		/* ====== 光电管校准: 持续采样更新最小/最大值 ====== */
		if(g_calibrating)
		{
			uint8_t ci;
			for(ci = 0; ci < SENSOR_COUNT; ci++)
			{
				uint16_t val = g_mux_adc_values[ci];
				if(val < g_cal_min[ci]) g_cal_min[ci] = val;
				if(val > g_cal_max[ci]) g_cal_max[ci] = val;
			}
		}
		
		/* ====== 低电压警告检测 (带消抖和滞回) ====== */
		if(BDI_V > 0.5f && BDI_V < LOW_BATTERY_THRESHOLD)
		{
			/* 电压低于阈值, 增加消抖计数 */
			if(g_low_battery_debounce < LOW_BATTERY_DEBOUNCE_CNT)
			{
				g_low_battery_debounce++;
			}
			else if(!g_low_battery_flag)
			{
				/* 连续低电压达到阈值, 确认低电压状态 */
				g_low_battery_flag = 1;
				g_low_battery_msg_timer = 0;  /* 立即发送第一条警告 */
			}
		}
		else if(BDI_V >= LOW_BATTERY_RECOVER_TH)
		{
			/* 电压高于恢复阈值, 清除消抖计数 */
			g_low_battery_debounce = 0;
			
			if(g_low_battery_flag)
			{
				/* 电压恢复正常, 清除低电压标志 */
				g_low_battery_flag = 0;
				RGB_SetColor(RGB_COLOR_OFF);
				printf("Battery voltage recovered: %.2fV\r\n", BDI_V);
			}
		}
		
		/* 低电压状态下的LED闪烁和消息发送 */
		if(g_low_battery_flag)
		{
			/* LED红色闪烁 (最醒目) */
			if((g_systick_ms - g_low_battery_led_timer) >= LOW_BATTERY_LED_PERIOD_MS)
			{
				g_low_battery_led_timer = g_systick_ms;
				g_low_battery_led_state = !g_low_battery_led_state;
				
				if(g_low_battery_led_state)
				{
					RGB_SetColor(RGB_COLOR_R);  /* 红色亮 */
				}
				else
				{
					RGB_SetColor(RGB_COLOR_OFF);  /* 灭 */
				}
			}
			
			/* 定期发送低电压警告消息 */
			if((g_systick_ms - g_low_battery_msg_timer) >= LOW_BATTERY_MSG_PERIOD_MS)
			{
				g_low_battery_msg_timer = g_systick_ms;
				printf("[WARNING] Low Battery! Voltage: %.2fV (Threshold: %.1fV)\r\n", BDI_V, LOW_BATTERY_THRESHOLD);
				printf("Please charge immediately to avoid over-discharge!\r\n");
			}
		}
		
		/* 处理按键事件 */
		Key_Event_t *event = Key_GetEvent();
		if(event != NULL)
		{
			HandleKeyEvent(event);
		}

		/* 蓝牙遥控模式: 应用运动指令 */
		if(g_bt_key_control_mode)
		{
			const BT_State_t *state = BT_GetState();
			BT_ApplyMotion(state);
		}
	}
}
