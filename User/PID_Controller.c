/**
 * @file    PID_Controller.c
 * @brief   双环 PID 控制器实现：速度环(内环) + 位置环(外环)
 * @details 本模块实现巡线小车的核心控制算法：
 *          - 速度环：增量式 PID，控制电机转速，带前馈补偿
 *          - 位置环：位置式 PID，根据传感器偏差修正左右轮差速
 *          - 支持蓝牙遥控模式和本地巡线模式切换
 */

#include "PID_Controller.h"
#include "LSM6DSR_Config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* ========================================================================== */
/*                              外部变量引用                                   */
/* ========================================================================== */
extern int16_t position_get;           /* 传感器检测到的黑线位置 (放大10倍) */
extern volatile uint32_t g_systick_ms; /* 系统滴答计时器 (毫秒) */
extern int32_t left_ecoder_cnt;        /* 左轮编码器累积计数 */
extern int32_t right_ecoder_cnt;       /* 右轮编码器累积计数 */

/* ========================================================================== */
/*                              内部状态变量                                   */
/* ========================================================================== */

/* 左右轮独立速度环控制器 (考虑两侧电机特性差异) */
static SpeedPID_Controller_t g_speed_pid_left;
static SpeedPID_Controller_t g_speed_pid_right;

/* 巡线速度参数 (可通过串口动态调整) */
static float g_line_speed_base = 130.0f;   /* 基础速度 (默认130) */
static float g_line_speed_range = 30.0f;  /* 速度变化范围 (默认30) */

/* 位置环控制器 (巡线偏差修正) */
static PositionPID_Controller_t g_position_pid;

/* 最近一次 PID 计算结果缓存 (供主循环读取用于调试/日志) */
static float g_latest_target_left = 0.0f;
static float g_latest_target_right = 0.0f;
static float g_latest_out_left = 0.0f;
static float g_latest_out_right = 0.0f;

/* 控制模式开关 */
static uint8_t g_position_loop_enabled = 0;  /* 位置环使能标志 */
static uint8_t g_gyro_damping_enabled = 1;   /* 陀螺仪阻尼使能 (抑制转弯过冲) */

/* 蓝牙遥控模式目标值缓存 (主循环写入, 中断读取) */
static float g_bt_target_left = 0.0f;
static float g_bt_target_right = 0.0f;
static uint16_t g_bt_duty_cap = 0;

/* ========================================================================== */
/*                          轮子锁定 (Position Hold) 状态变量                   */
/* ========================================================================== */
static uint8_t g_wheel_lock_enabled = 0;       /* 锁定使能标志 */
static uint8_t g_wheel_lock_braking = 0;       /* 刹车阶段标志 (进入锁定前先刹车) */
static int32_t g_lock_target_left = 0;         /* 左轮锁定目标位置 */
static int32_t g_lock_target_right = 0;        /* 右轮锁定目标位置 */
static float g_lock_last_error_left = 0.0f;    /* 锁定PD历史误差 */
static float g_lock_last_error_right = 0.0f;
static float g_lock_integral_left = 0.0f;      /* 锁定积分项 (抵抗重力) */
static float g_lock_integral_right = 0.0f;
static uint8_t g_lock_stable_count = 0;        /* 稳定计数器 (用于刹车阶段) */
static uint16_t g_lock_brake_timeout = 0;      /* 刹车超时计数器 */

/* 锁定PID参数 - 带积分项以抵抗重力 */
#define LOCK_KP           8.0f      /* 位置误差比例系数 (降低以减少过冲) */
#define LOCK_KI           0.3f      /* 位置误差积分系数 (消除重力导致的稳态误差) */
#define LOCK_KD           1.0f      /* 位置误差微分系数 */
#define LOCK_KV           100.0f    /* 速度阻尼系数 (增大以抑制振荡) */
#define LOCK_OUTPUT_MAX   8000.0f   /* 锁定PWM输出上限 */
#define LOCK_INTEGRAL_MAX 8000.0f   /* 积分限幅 (增大以抵抗重力) */
#define LOCK_DEADZONE     100       /* 位置死区 (增大以覆盖齿轮间隙) */
#define LOCK_BRAKE_SPEED_TH  8      /* 刹车阶段速度阈值 (低于此值认为已停止) */
#define LOCK_BRAKE_TIMEOUT  100     /* 刹车超时计数 (防止在墙上永远不能进入锁定) */

/* ========================================================================== */
/*                           速度环 PID 实现                                   */
/* ========================================================================== */

/**
 * @brief  初始化速度环 PID 控制器
 * @param  controller  控制器结构体指针
 * @param  kp          比例系数
 * @param  ki          积分系数 (增量式中用于累加误差)
 * @param  kd          微分系数 (当前未使用)
 * @param  kf          前馈系数 (消除目标速度变化时的相位滞后)
 * @param  output_max  输出上限 (PWM 占空比)
 * @param  output_min  输出下限 (负值表示反转)
 */
void SpeedPID_Init(SpeedPID_Controller_t *controller, float kp, float ki, float kd, float kf,
				   float output_max, float output_min)
{
	if(controller == NULL) return;

	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
	controller->param.kf = kf;
	controller->param.output_max = output_max;
	controller->param.output_min = output_min;
	controller->param.integral_max = output_max * 0.5f;
	controller->param.integral_min = output_min * 0.5f;

	controller->last_error = 0.0f;
	controller->last_output = 0.0f;
	controller->integral = 0.0f;
}

/**
 * @brief  速度环 PID 单步计算 (增量式)
 * @param  controller     控制器结构体指针
 * @param  target_speed   目标速度 (编码器单位)
 * @param  current_speed  当前实际速度 (编码器单位)
 * @return 输出 PWM 占空比 (正值正转, 负值反转)
 * 
 * @details 增量式 PID 公式:
 *          Output = LastOutput + Kp*DeltaErr + Ki*Err
 *          其中 DeltaErr = Error - LastError
 *          
 *          特性:
 *          - 死区处理: 误差 < 2 时忽略, 避免频繁微调
 *          - 斜率限制: 单次误差变化限幅 ±300, 防止 PWM 突变
 *          - 前馈补偿: 加速目标响应, 消除相位滞后
 *          - 起步死区: 克服静摩擦, 保证低速启动
 */
float SpeedPID_Step(SpeedPID_Controller_t *controller, float target_speed, float current_speed)
{
	if(controller == NULL) return 0.0f;

	float error = target_speed - current_speed;
    
	/* 死区处理: 小误差不响应, 避免噪声引起的频繁调节 */
	if(fabsf(error) < 2.0f) error = 0.0f;

	/* 误差变化率限幅 (Slew Rate Limiter)
	 * 防止测量噪声导致的 PWM 大幅跳变 */
	float d_error = error - controller->last_error;
	if(d_error > 300.0f) d_error = 300.0f;
	if(d_error < -300.0f) d_error = -300.0f;

	/* 增量式 PID 核心计算 */
	float p_term = controller->param.kp * d_error;
	float i_term = controller->param.ki * error;
	float delta_output = p_term + i_term;

	/* 增量限幅: 防止单次输出变化过大 */
	if(delta_output > 1500.0f) delta_output = 1500.0f;
	if(delta_output < -1500.0f) delta_output = -1500.0f;

	/* 累加得到新的 PID 输出分量 */
	float pid_component = controller->last_output + delta_output;

	/* 输出饱和限幅 */
	if(pid_component > controller->param.output_max)
		pid_component = controller->param.output_max;
	else if(pid_component < controller->param.output_min)
		pid_component = controller->param.output_min;

	/* 更新历史状态 */
	controller->last_error = error;
	
	/* 累积值限幅: 使用 output_max 的一定比例，而非硬编码值
	 * 爬墙时需要更大的 PID 累积输出来克服重力负载 */
	float pid_accum_limit = controller->param.output_max * 0.9f;  /* 允许 PID 累积到 output_max 的 90% */
	if(pid_component > pid_accum_limit) pid_component = pid_accum_limit;
	if(pid_component < -pid_accum_limit) pid_component = -pid_accum_limit;
	controller->last_output = pid_component;

	/* 前馈补偿: 加速目标跟踪响应 */
	float feed_forward = target_speed * controller->param.kf;
	float new_output = pid_component + feed_forward;

	/* 起步死区补偿: 克服电机静摩擦
	 * 只有在电机接近静止时才补偿到最小启动占空比 */
	#define MOTOR_DEADZONE 1400.0f
	#define MOTOR_RUNNING_THRESHOLD 15.0f
	
	if(fabsf(new_output) > 50.0f && fabsf(new_output) < MOTOR_DEADZONE)
	{
		if(fabsf(current_speed) < MOTOR_RUNNING_THRESHOLD)
		{
			new_output = (new_output > 0.0f) ? MOTOR_DEADZONE : -MOTOR_DEADZONE;
		}
	}

	/* 最终输出限幅 */
	if(new_output > controller->param.output_max)
		new_output = controller->param.output_max;
	else if(new_output < controller->param.output_min)
		new_output = controller->param.output_min;

	return new_output;
}

/**
 * @brief  设置速度环 PID 参数
 */
void SpeedPID_SetParam(SpeedPID_Controller_t *controller, float kp, float ki, float kd)
{
	if(controller == NULL) return;
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
}

/**
 * @brief  设置速度环前馈系数
 */
void SpeedPID_SetFeedForward(SpeedPID_Controller_t *controller, float kf)
{
	if(controller == NULL) return;
	controller->param.kf = kf;
}

/**
 * @brief  重置速度环 PID 控制器状态 (清零历史值与积分项)
 */
void SpeedPID_Reset(SpeedPID_Controller_t *controller)
{
	if(controller == NULL) return;
	controller->last_error = 0.0f;
	controller->last_output = 0.0f;
	controller->integral = 0.0f;
}

/* ========================================================================== */
/*                           位置环 PID 实现                                   */
/* ========================================================================== */

/**
 * @brief  初始化位置环 PID 控制器
 * @param  controller       控制器结构体指针
 * @param  kp               比例系数
 * @param  ki               积分系数
 * @param  kd               微分系数
 * @param  gyro_kd          陀螺仪阻尼系数 (用于抑制转弯过冲)
 * @param  output_max       输出上限 (叠加到左右轮差速)
 * @param  output_min       输出下限
 * @param  target_position  目标位置 (传感器阵列中心, 通常为 7.5)
 */
void PositionPID_Init(PositionPID_Controller_t *controller, float kp, float ki, float kd, float gyro_kd,
						float output_max, float output_min, float target_position)
{
	if(controller == NULL) return;

	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
	controller->param.gyro_kd = gyro_kd;
	controller->param.output_max = output_max;
	controller->param.output_min = output_min;
	controller->param.integral_max = output_max * 0.5f;
	controller->param.integral_min = output_min * 0.5f;
	controller->param.target_position = target_position;

	controller->last_error = 0.0f;
	controller->integral = 0.0f;
}

/**
 * @brief  位置环 PID 单步计算 (位置式)
 * @param  controller        控制器结构体指针
 * @param  current_position  当前黑线位置 (0~15, 中心约 7.5)
 * @return 偏差修正量 (正值: 左轮减速/右轮加速; 负值: 反之)
 * 
 * @details 位置式 PID 公式:
 *          Output = Kp*Err + Ki*∫Err + Kd*dErr + GyroKd*AngularVelocity
 *          
 *          陀螺仪阻尼项用于抑制转弯过冲, 提高稳定性
 */
float PositionPID_Calculate(PositionPID_Controller_t *controller, float current_position)
{
	if(controller == NULL) return 0.0f;

	float error = current_position - controller->param.target_position;
	
	/* 比例项 */
	float p_term = controller->param.kp * error;
	
	/* 积分项 (带抗饱和限幅) */
	controller->integral += error;
	if(controller->integral > controller->param.integral_max)
		controller->integral = controller->param.integral_max;
	else if(controller->integral < controller->param.integral_min)
		controller->integral = controller->param.integral_min;
	float i_term = controller->param.ki * controller->integral;
	
	/* 微分项 */
	float d_term = controller->param.kd * (error - controller->last_error);
	
	/* 陀螺仪阻尼项: 利用 Z 轴角速度抑制转弯过冲 */
	float gyro_term = controller->param.gyro_kd * LSE6DSR_data.gz_rads;
	if(gyro_term > 7000.0f) gyro_term = 7000.0f;
	else if(gyro_term < -7000.0f) gyro_term = -7000.0f;

	float output = p_term + i_term + d_term + gyro_term;
	
	/* 输出限幅 */
	if(output > controller->param.output_max)
		output = controller->param.output_max;
	else if(output < controller->param.output_min)
		output = controller->param.output_min;
	
	controller->last_error = error;
	return output;
}

/**
 * @brief  设置位置环 PID 参数
 */
void PositionPID_SetParam(PositionPID_Controller_t *controller, float kp, float ki, float kd)
{
	if(controller == NULL) return;
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
}

/**
 * @brief  设置位置环目标位置
 */
void PositionPID_SetTarget(PositionPID_Controller_t *controller, float target_position)
{
	if(controller == NULL) return;
	controller->param.target_position = target_position;
}

/**
 * @brief  重置位置环 PID 控制器状态
 */
void PositionPID_Reset(PositionPID_Controller_t *controller)
{
	if(controller == NULL) return;
	controller->last_error = 0.0f;
	controller->integral = 0.0f;
}

/**
 * @brief  设置位置环全局参数 (运行时可调, 用于蓝牙调参)
 */
void PositionPID_SetGlobalParams(float kp, float ki, float kd, float gyro_kd)
{
	g_position_pid.param.kp = kp;
	g_position_pid.param.ki = ki;
	g_position_pid.param.kd = kd;
	g_position_pid.param.gyro_kd = gyro_kd;
}

/* ========================================================================== */
/*                         电机控制封装函数                                    */
/* ========================================================================== */

/**
 * @brief  根据速度目标值设置电机 (自动处理方向)
 * @param  motor_id      电机编号 (MOTOR_L 或 MOTOR_R)
 * @param  speed_target  目标占空比 (正值正转, 负值反转)
 */
void Motor_SetSpeedWithDirection(uint8_t motor_id, float speed_target)
{
	uint16_t duty = 0;
	uint8_t direction = MOTOR_DIR_FORWARD;

	if(speed_target < 0.0f)
	{
		direction = MOTOR_DIR_BACKWARD;
		duty = (uint16_t)(-speed_target);
	}
	else
	{
		direction = MOTOR_DIR_FORWARD;
		duty = (uint16_t)speed_target;
	}

	if(duty > MOTOR_DUTY_MAX)
		duty = MOTOR_DUTY_MAX;

	Motor_SetDirection(motor_id, direction);
	Motor_SetSpeed(motor_id, duty);
}

/**
 * @brief  重置双轮速度环状态
 */
void SpeedPID_ResetState(void)
{
	SpeedPID_Reset(&g_speed_pid_left);
	SpeedPID_Reset(&g_speed_pid_right);
}

/**
 * @brief  重置位置环状态
 */
void PositionPID_ResetState(void)
{
	PositionPID_Reset(&g_position_pid);
}

/**
 * @brief  同时设置双轮速度环 PID 参数
 */
void SpeedPID_SetParamBoth(float kp, float ki, float kd)
{
	SpeedPID_SetParam(&g_speed_pid_left, kp, ki, kd);
	SpeedPID_SetParam(&g_speed_pid_right, kp, ki, kd);
}

/**
 * @brief  同时设置双轮速度环前馈系数
 */
void SpeedPID_SetFeedForwardBoth(float kf)
{
	SpeedPID_SetFeedForward(&g_speed_pid_left, kf);
	SpeedPID_SetFeedForward(&g_speed_pid_right, kf);
}

/**
 * @brief  双轮速度环手动更新 (不直接驱动电机)
 * @param  target_left   左轮目标速度
 * @param  target_right  右轮目标速度
 * @param  out_left      输出: 左轮 PWM 占空比
 * @param  out_right     输出: 右轮 PWM 占空比
 */
void SpeedPID_ManualUpdateDual(float target_left, float target_right, float *out_left, float *out_right)
{
	if(out_left == NULL || out_right == NULL) return;

	float outL = SpeedPID_Step(&g_speed_pid_left, target_left, (float)speed_left);
	float outR = SpeedPID_Step(&g_speed_pid_right, target_right, (float)speed_right);

	/* 输出限幅 */
	if(outL > (float)MOTOR_DUTY_MAX) outL = (float)MOTOR_DUTY_MAX;
	if(outL < -(float)MOTOR_DUTY_MAX) outL = -(float)MOTOR_DUTY_MAX;
	if(outR > (float)MOTOR_DUTY_MAX) outR = (float)MOTOR_DUTY_MAX;
	if(outR < -(float)MOTOR_DUTY_MAX) outR = -(float)MOTOR_DUTY_MAX;

	/* 更新全局监控变量 */
	g_latest_target_left = target_left;
	g_latest_target_right = target_right;
	g_latest_out_left = outL;
	g_latest_out_right = outR;

	*out_left = outL;
	*out_right = outR;
}

/**
 * @brief  执行速度闭环并驱动电机 (无位置环)
 * @param  target_left   左轮目标速度
 * @param  target_right  右轮目标速度
 * @param  duty_cap      PWM 输出上限 (软启动斜坡)
 */
void SpeedPID_RunClosedLoop(float target_left, float target_right, uint16_t duty_cap)
{
	float outL = 0.0f, outR = 0.0f;
	SpeedPID_ManualUpdateDual(target_left, target_right, &outL, &outR);

	if(duty_cap > MOTOR_DUTY_MAX) duty_cap = MOTOR_DUTY_MAX;
	if(outL > duty_cap) outL = (float)duty_cap;
	if(outL < -(float)duty_cap) outL = -(float)duty_cap;
	if(outR > duty_cap) outR = (float)duty_cap;
	if(outR < -(float)duty_cap) outR = -(float)duty_cap;

	Motor_SetSpeedWithDirection(MOTOR_L, outL);
	Motor_SetSpeedWithDirection(MOTOR_R, outR);
}

/**
 * @brief  获取最近一次 PID 计算结果 (用于串口调试输出)
 */
void SpeedPID_GetLatest(float *target_left, float *target_right, float *out_left, float *out_right)
{
	if(target_left) *target_left = g_latest_target_left;
	if(target_right) *target_right = g_latest_target_right;
	if(out_left) *out_left = g_latest_out_left;
	if(out_right) *out_right = g_latest_out_right;
}

/**
 * @brief  设置蓝牙遥控模式目标值
 * @note   实际 PWM 输出在 SysTick 中断中执行
 */
void PID_SetBluetoothTarget(float target_left, float target_right, uint16_t duty_cap)
{
	g_bt_target_left = target_left;
	g_bt_target_right = target_right;
	g_bt_duty_cap = duty_cap;
}

/* ========================================================================== */
/*                         PID 初始化与主控制循环                              */
/* ========================================================================== */

/**
 * @brief  初始化 PID 控制器模块
 * @note   设置速度环和位置环的默认参数, 参数已通过实际测试调优
 */
void PID_Init(void)
{
	/* 速度环参数: 左右轮独立 (考虑电机特性差异)
	 * Kp=2.6/2.75, Ki=0.17/0.18, Kf=10 (前馈消除相位滞后) */
	SpeedPID_Init(&g_speed_pid_left,  2.60f, 0.173f, 0.0f, 10.0f, 8000.0f, -8000.0f);
	SpeedPID_Init(&g_speed_pid_right, 2.75f, 0.183f, 0.0f, 10.0f, 8000.0f, -8000.0f);
	
	/* 位置环参数
	 * 目标位置 7.5 (16 路传感器中心) */
	PositionPID_Init(&g_position_pid, 425.0f, 1.5f, 3000.0f, 100.0f, 9000.0f, -9000.0f, 7.5f);

	/* 默认启用位置环 (巡线模式) */
	PID_PositionLoop_Enable(1);
}

extern uint8_t star_car;
extern volatile uint8_t g_bt_key_control_mode;

/**
 * @brief  PID 控制主循环 (在 SysTick 中断中调用, 周期 2ms)
 * 
 * @details 控制优先级:
 *          1. 轮子锁定模式: 使用位置PD控制保持位置不动
 *          2. 蓝牙遥控模式: 直接使用蓝牙设定的目标速度
 *          3. 巡线模式: 位置环 + 速度环双环控制
 */
void PID_Control_Update(void)
{
	/* ====== 轮子锁定模式 (最高优先级) ====== */
	if(g_wheel_lock_enabled)
	{
		WheelLock_Update();
		return;
	}
	
	/* ====== 蓝牙遥控模式 ====== */
	if(g_bt_key_control_mode)
	{
		float outL = 0.0f, outR = 0.0f;
		
		SpeedPID_ManualUpdateDual(g_bt_target_left, g_bt_target_right, &outL, &outR);

		/* 蓝牙起步补偿: 起步阶段强制 1500 PWM 以确保迅速启动 */
		#define BT_START_PWM 2000.0f
		#define BT_START_SPEED_THRESHOLD 15.0f
		if(fabsf(g_bt_target_left) > 1.0f && fabsf((float)speed_left) < BT_START_SPEED_THRESHOLD)
		{
			if(fabsf(outL) < BT_START_PWM)
				outL = (g_bt_target_left >= 0.0f) ? BT_START_PWM : -BT_START_PWM;
		}
		if(fabsf(g_bt_target_right) > 1.0f && fabsf((float)speed_right) < BT_START_SPEED_THRESHOLD)
		{
			if(fabsf(outR) < BT_START_PWM)
				outR = (g_bt_target_right >= 0.0f) ? BT_START_PWM : -BT_START_PWM;
		}

		/* 应用软启动限幅 */
		float limit = (float)g_bt_duty_cap;
		if(limit > (float)MOTOR_DUTY_MAX) limit = (float)MOTOR_DUTY_MAX;
		
		if(outL > limit) outL = limit;
		if(outL < -limit) outL = -limit;
		if(outR > limit) outR = limit;
		if(outR < -limit) outR = -limit;

		Motor_SetSpeedWithDirection(MOTOR_L, outL);
		Motor_SetSpeedWithDirection(MOTOR_R, outR);
		return;
	}

	/* ====== 位置环未启用时不控制电机 ====== */
	if(!g_position_loop_enabled)
	{
		return;
	}

	/* ====== 巡线模式 (双环控制) ====== */
	float current_position = (float)position_get / 10.0f;
	float position_correction = PositionPID_Calculate(&g_position_pid, current_position);
	
	float speed_out_l = 0.0f;
	float speed_out_r = 0.0f;
	
	static uint8_t first_set = 0;
	static float start_speed = 0.0f;

	/* 根据偏差动态调整基础速度: 偏差越大速度越慢 */
	float deviation = fabsf(current_position - 7.5f);
	float i_speed = g_line_speed_base - (fminf(deviation, 3.0f) / 3.0f) * g_line_speed_range;

	if(star_car)
	{
		/* 软启动: 速度逐渐爬升 */
		if(start_speed < i_speed && first_set == 0)
		{
			start_speed += 0.5f;
		}
		else
		{
			start_speed = i_speed;
			first_set = 1;
		}
		
		speed_out_l = SpeedPID_Step(&g_speed_pid_left, start_speed, (float)speed_left);
		speed_out_r = SpeedPID_Step(&g_speed_pid_right, start_speed, (float)speed_right);
	}
	else
	{
		/* 停车状态: 重置所有状态 */
		start_speed = 0.0f;
		first_set = 0;
		SpeedPID_Reset(&g_speed_pid_left);
		SpeedPID_Reset(&g_speed_pid_right);
	}

	/* 叠加位置环修正量 */
	float left_output = speed_out_l + position_correction;
	float right_output = speed_out_r - position_correction;
	
	/* 更新监控变量 */
	g_latest_target_left = start_speed;
	g_latest_target_right = start_speed;
	g_latest_out_left = left_output;
	g_latest_out_right = right_output;
	
	Motor_SetSpeedWithDirection(MOTOR_L, left_output);
	Motor_SetSpeedWithDirection(MOTOR_R, right_output);
}

/* ========================================================================== */
/*                         控制模式开关接口                                    */
/* ========================================================================== */

void PID_PositionLoop_Enable(uint8_t enable)
{
	g_position_loop_enabled = enable ? 1 : 0;
}

uint8_t PID_PositionLoop_IsEnabled(void)
{
	return g_position_loop_enabled;
}

void PID_GyroDamping_Enable(uint8_t enable)
{
	g_gyro_damping_enabled = enable ? 1 : 0;
}

uint8_t PID_GyroDamping_IsEnabled(void)
{
	return g_gyro_damping_enabled;
}

/* ========================================================================== */
/*                          轮子锁定 (Position Hold) 功能                      */
/* ========================================================================== */

/**
 * @brief  启用轮子锁定模式
 * @note   先进入刹车阶段，等轮子速度降低后再记录锁定位置
 *         这样可以避免从高速运动状态突然锁定导致的振荡
 */
void WheelLock_Enable(void)
{
	/* 检查当前速度，如果还在运动则先进入刹车阶段 */
	if(abs(speed_left) > LOCK_BRAKE_SPEED_TH || abs(speed_right) > LOCK_BRAKE_SPEED_TH)
	{
		/* 进入刹车阶段 */
		g_wheel_lock_braking = 1;
		g_wheel_lock_enabled = 1;
		g_lock_stable_count = 0;
		g_lock_brake_timeout = 0;  /* 重置刹车超时计数器 */
		Motor_Enable();
		return;
	}
	
	/* 速度已经很低，直接记录当前位置作为锁定目标 */
	g_lock_target_left = left_ecoder_cnt;
	g_lock_target_right = right_ecoder_cnt;
	
	/* 清除历史误差和积分项 */
	g_lock_last_error_left = 0.0f;
	g_lock_last_error_right = 0.0f;
	g_lock_integral_left = 0.0f;
	g_lock_integral_right = 0.0f;
	
	/* 使能锁定，跳过刹车阶段 */
	g_wheel_lock_braking = 0;
	g_wheel_lock_enabled = 1;
	g_lock_stable_count = 0;
	
	/* 确保电机使能 */
	Motor_Enable();
}

/**
 * @brief  禁用轮子锁定模式
 */
void WheelLock_Disable(void)
{
	g_wheel_lock_enabled = 0;
	g_wheel_lock_braking = 0;
	g_lock_stable_count = 0;
}

/**
 * @brief  检查轮子锁定是否启用
 */
uint8_t WheelLock_IsEnabled(void)
{
	return g_wheel_lock_enabled;
}

/**
 * @brief  轮子锁定PD控制更新 (在SysTick中调用)
 * @note   使用位置PD控制 + 速度阻尼保持轮子在目标位置
 *         P项提供恢复力，D项+速度阻尼防止振荡
 */
void WheelLock_Update(void)
{
	if(!g_wheel_lock_enabled) return;
	
	/* ====== 刹车阶段: 等待轮子速度降低 ====== */
	if(g_wheel_lock_braking)
	{
		/* 使用纯速度阻尼进行刹车 (反向力与速度成正比) */
		float brake_left = -LOCK_KV * (float)speed_left;
		float brake_right = -LOCK_KV * (float)speed_right;
		
		/* 刹车输出限幅 */
		if(brake_left > LOCK_OUTPUT_MAX) brake_left = LOCK_OUTPUT_MAX;
		if(brake_left < -LOCK_OUTPUT_MAX) brake_left = -LOCK_OUTPUT_MAX;
		if(brake_right > LOCK_OUTPUT_MAX) brake_right = LOCK_OUTPUT_MAX;
		if(brake_right < -LOCK_OUTPUT_MAX) brake_right = -LOCK_OUTPUT_MAX;
		
		Motor_SetSpeedWithDirection(MOTOR_L, brake_left);
		Motor_SetSpeedWithDirection(MOTOR_R, brake_right);
		
		/* 刹车超时计数 */
		g_lock_brake_timeout++;
		
		/* 检查速度是否已经足够低，或者刹车超时 */
		if((abs(speed_left) <= LOCK_BRAKE_SPEED_TH && abs(speed_right) <= LOCK_BRAKE_SPEED_TH) ||
		   g_lock_brake_timeout >= LOCK_BRAKE_TIMEOUT)
		{
			g_lock_stable_count++;
			/* 连续多次检测到低速或超时，进入锁定 (约10ms) */
			if(g_lock_stable_count >= 5)
			{
				/* 刹车完成，记录当前位置作为锁定目标 */
				g_lock_target_left = left_ecoder_cnt;
				g_lock_target_right = right_ecoder_cnt;
				g_lock_last_error_left = 0.0f;
				g_lock_last_error_right = 0.0f;
				g_lock_integral_left = 0.0f;
				g_lock_integral_right = 0.0f;
				g_wheel_lock_braking = 0;  /* 退出刹车阶段，进入锁定保持 */
			}
		}
		else
		{
			g_lock_stable_count = 0;  /* 速度还比较高，重置计数 */
		}
		return;
	}
	
	/* ====== 锁定保持阶段: 位置PID + 速度阻尼 ====== */
	
	/* 计算位置误差 */
	float error_left = (float)(g_lock_target_left - left_ecoder_cnt);
	float error_right = (float)(g_lock_target_right - right_ecoder_cnt);
	
	/* P项 (死区处理) */
	float p_left = 0.0f, p_right = 0.0f;
	if(fabsf(error_left) >= LOCK_DEADZONE)
		p_left = LOCK_KP * error_left;
	if(fabsf(error_right) >= LOCK_DEADZONE)
		p_right = LOCK_KP * error_right;
	
	/* I项 (只在死区外累积，死区内缓慢衰减) */
	if(fabsf(error_left) >= LOCK_DEADZONE)
	{
		g_lock_integral_left += error_left;
	}
	else
	{
		/* 死区内: 积分缓慢衰减但不清零 (保留一部分以抵抗重力) */
		g_lock_integral_left *= 0.995f;
	}
	if(fabsf(error_right) >= LOCK_DEADZONE)
	{
		g_lock_integral_right += error_right;
	}
	else
	{
		g_lock_integral_right *= 0.995f;
	}
	if(g_lock_integral_left > LOCK_INTEGRAL_MAX) g_lock_integral_left = LOCK_INTEGRAL_MAX;
	if(g_lock_integral_left < -LOCK_INTEGRAL_MAX) g_lock_integral_left = -LOCK_INTEGRAL_MAX;
	if(g_lock_integral_right > LOCK_INTEGRAL_MAX) g_lock_integral_right = LOCK_INTEGRAL_MAX;
	if(g_lock_integral_right < -LOCK_INTEGRAL_MAX) g_lock_integral_right = -LOCK_INTEGRAL_MAX;
	float i_left = LOCK_KI * g_lock_integral_left;
	float i_right = LOCK_KI * g_lock_integral_right;
	
	/* D项 */
	float d_error_left = error_left - g_lock_last_error_left;
	float d_error_right = error_right - g_lock_last_error_right;
	
	/* 速度阻尼项 (直接使用编码器速度反馈，比微分项更稳定) */
	float velocity_damping_left = -LOCK_KV * (float)speed_left;
	float velocity_damping_right = -LOCK_KV * (float)speed_right;
	
	/* 综合输出 = 位置P + 位置I + 位置D + 速度阻尼 */
	float output_left = p_left + i_left + LOCK_KD * d_error_left + velocity_damping_left;
	float output_right = p_right + i_right + LOCK_KD * d_error_right + velocity_damping_right;
	
	/* 输出限幅 */
	if(output_left > LOCK_OUTPUT_MAX) output_left = LOCK_OUTPUT_MAX;
	if(output_left < -LOCK_OUTPUT_MAX) output_left = -LOCK_OUTPUT_MAX;
	if(output_right > LOCK_OUTPUT_MAX) output_right = LOCK_OUTPUT_MAX;
	if(output_right < -LOCK_OUTPUT_MAX) output_right = -LOCK_OUTPUT_MAX;
	
	/* 更新历史误差 */
	g_lock_last_error_left = error_left;
	g_lock_last_error_right = error_right;
	
	/* 应用到电机 */
	Motor_SetSpeedWithDirection(MOTOR_L, output_left);
	Motor_SetSpeedWithDirection(MOTOR_R, output_right);
}

/* ========================================================================== */
/*                         巡线速度参数接口                                     */
/* ========================================================================== */

/**
 * @brief  设置巡线速度参数
 * @param  base_speed   基础速度 (原先固定为 80)
 * @param  speed_range  速度变化范围 (原先固定为 30)
 * @note   公式: i_speed = base_speed - (deviation/3) * speed_range
 */
void PID_SetLineSpeedParams(float base_speed, float speed_range)
{
	g_line_speed_base = base_speed;
	g_line_speed_range = speed_range;
}

/**
 * @brief  获取巡线速度参数
 * @param  base_speed   输出: 基础速度
 * @param  speed_range  输出: 速度变化范围
 */
void PID_GetLineSpeedParams(float *base_speed, float *speed_range)
{
	if(base_speed) *base_speed = g_line_speed_base;
	if(speed_range) *speed_range = g_line_speed_range;
}

/**
 * @brief  获取当前 PID 运行状态 (用于串口调试)
 * @param  target_left   输出: 左轮目标速度
 * @param  target_right  输出: 右轮目标速度
 * @param  actual_left   输出: 左轮实际速度
 * @param  actual_right  输出: 右轮实际速度
 * @param  pwm_left      输出: 左轮 PWM 输出
 * @param  pwm_right     输出: 右轮 PWM 输出
 */
void PID_GetRunStatus(float *target_left, float *target_right, 
                      int16_t *actual_left, int16_t *actual_right,
                      float *pwm_left, float *pwm_right)
{
	if(target_left) *target_left = g_latest_target_left;
	if(target_right) *target_right = g_latest_target_right;
	if(actual_left) *actual_left = speed_left;
	if(actual_right) *actual_right = speed_right;
	if(pwm_left) *pwm_left = g_latest_out_left;
	if(pwm_right) *pwm_right = g_latest_out_right;
}
