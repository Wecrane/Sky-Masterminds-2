#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include "stm32f10x.h"
#include "Motor_ctr.h"
#include "ABEncoder.h"
#include "BlackPoint_Finder.h"
#include <math.h>

/**
 * @brief PID控制器使用说明
 * 
 * 控制架构：
 * 1. 速度环（内环）：控制平均速度，反馈 = (speed_left + speed_right) / 2
 * 2. 位置环（外环）：输出偏差值，直接叠加到速度环输出上
 * 
 * 使用流程：
 * 1. 计算速度环：
 *    - 目标速度：target_speed（固定值）
 *    - 反馈速度：avg_speed = (speed_left + speed_right) / 2
 *    - 速度环输出：speed_output = SpeedPID_Calculate(&speed_pid, target_speed, avg_speed)
 * 
 * 2. 计算位置环：
 *    - 当前位置：current_position（寻点位置）
 *    - 位置环输出：position_correction = PositionPID_Calculate(&position_pid, current_position)
 * 
 * 3. 最终输出（注意极性）：
 *    - left_output = speed_output - position_correction  （位置偏右时，position_correction为正，左轮减速）
 *    - right_output = speed_output + position_correction  （位置偏右时，position_correction为正，右轮加速）
 * 
 * 极性说明：
 * - 位置偏右（position > target）：position_correction > 0，左轮减速、右轮加速，修正回中心
 * - 位置偏左（position < target）：position_correction < 0，左轮加速、右轮减速，修正回中心
 */

/* ========================================================================== */
/*                           速度环 PID 参数                                   */
/* ========================================================================== */
// 增量式PID参数（用于电机速度控制）
typedef struct
{
	float kp;           // 比例系数
	float ki;           // 积分系数
	float kd;           // 微分系数
	float kf;           // 前馈系数
	float output_max;   // 输出上限（占空比，0-10000）
	float output_min;   // 输出下限（占空比，-10000到0，负值为后退）
	float integral_max; // 积分限幅（防止积分饱和）
	float integral_min; // 积分下限
} SpeedPID_Param_t;

// 速度环PID控制器结构体
typedef struct
{
	SpeedPID_Param_t param;  // PID参数
	float last_error;        // 上一次误差
	float last_output;       // 上一次输出
	float integral;          // 积分项累积
} SpeedPID_Controller_t;

// ==================== 位置环PID参数 ====================
// 位置式PID参数（用于寻点位置控制）
typedef struct
{
	float kp;           // 比例系数
	float ki;           // 积分系数
	float kd;           // 微分系数
	float gyro_kd;
	float output_max;   // 输出上限（偏差值，叠加到速度环输出）
	float output_min;   // 输出下限（偏差值，叠加到速度环输出）
	float integral_max; // 积分限幅
	float integral_min; // 积分下限
	float target_position; // 目标位置（寻点中心位置，通常为SENSOR_COUNT/2）
} PositionPID_Param_t;

// 位置环PID控制器结构体
typedef struct
{
	PositionPID_Param_t param;  // PID参数
	float last_error;            // 上一次误差
	float integral;              // 积分项累积
} PositionPID_Controller_t;

// ==================== 速度环PID函数 ====================
/**
 * @brief 初始化速度环PID控制器
 * @param controller: 速度环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 * @param output_max: 输出上限（占空比，0-10000）
 * @param output_min: 输出下限（占空比，-10000到0）
 */
void SpeedPID_Init(SpeedPID_Controller_t *controller, float kp, float ki, float kd, float kf, 
                   float output_max, float output_min);

/**
 * @brief 速度环PID计算（增量式）
 * @param controller: 速度环PID控制器指针
 * @param target_speed: 目标速度（编码器值）
 * @param current_speed: 当前速度（编码器值）
 * @return 输出值（占空比，0-10000为正转，负值为后退）
 */
float SpeedPID_Calculate(SpeedPID_Controller_t *controller, float target_speed, float current_speed);

/**
 * @brief 设置速度环PID参数
 * @param controller: 速度环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 */
void SpeedPID_SetParam(SpeedPID_Controller_t *controller, float kp, float ki, float kd);

/**
 * @brief 设置速度环前馈系数
 */
void SpeedPID_SetFeedForward(SpeedPID_Controller_t *controller, float kf);

/**
 * @brief 重置速度环PID控制器（清零积分项和历史值）
 * @param controller: 速度环PID控制器指针
 */
void SpeedPID_Reset(SpeedPID_Controller_t *controller);

// ==================== 速度环辅助（键控模式） ====================
// 重置速度环状态（清零积分与历史值）
void SpeedPID_ResetState(void);
// 独立双轮速度环更新，输出左右轮占空比（带正负方向）
void SpeedPID_ManualUpdateDual(float target_left, float target_right, float *out_left, float *out_right);
// 直接执行闭环并下发到电机（无位置环）
void SpeedPID_RunClosedLoop(float target_left, float target_right, uint16_t duty_cap);
// 设置双轮统一PID参数
void SpeedPID_SetParamBoth(float kp, float ki, float kd);
// 设置双轮统一前馈系数
void SpeedPID_SetFeedForwardBoth(float kf);
// 获取最近一次速度目标与输出（用于记录）
void SpeedPID_GetLatest(float *target_left, float *target_right, float *out_left, float *out_right);

// ==================== 位置环PID函数 ====================
/**
 * @brief 初始化位置环PID控制器
 * @param controller: 位置环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 * @param output_max: 输出上限（偏差值，叠加到速度环输出）
 * @param output_min: 输出下限（偏差值，叠加到速度环输出）
 * @param target_position: 目标位置（寻点中心位置，通常为SENSOR_COUNT/2）
 */
void PositionPID_Init(PositionPID_Controller_t *controller, float kp, float ki, float kd, float gyro_kd,
                      float output_max, float output_min, float target_position);

/**
 * @brief 位置环PID计算（位置式）
 * @param controller: 位置环PID控制器指针
 * @param current_position: 当前位置（寻点位置，左边为0）
 * @return 输出偏差值（直接叠加到速度环输出上，左轮减、右轮加）
 * @note 位置偏右时输出为正，左轮减速、右轮加速；位置偏左时输出为负，左轮加速、右轮减速
 */
float PositionPID_Calculate(PositionPID_Controller_t *controller, float current_position);

/**
 * @brief 设置位置环PID参数
 * @param controller: 位置环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 */
void PositionPID_SetParam(PositionPID_Controller_t *controller, float kp, float ki, float kd);

/**
 * @brief 设置位置环目标位置
 * @param controller: 位置环PID控制器指针
 * @param target_position: 目标位置（寻点中心位置，通常为SENSOR_COUNT/2）
 */
void PositionPID_SetTarget(PositionPID_Controller_t *controller, float target_position);

/**
 * @brief 重置位置环PID控制器（清零积分项和历史值）
 * @param controller: 位置环PID控制器指针
 */
void PositionPID_Reset(PositionPID_Controller_t *controller);

// ==================== 电机控制辅助函数 ====================
/**
 * @brief 根据速度目标值设置电机（自动处理方向和速度）
 * @param motor_id: 电机编号（MOTOR_L 或 MOTOR_R）
 * @param speed_target: 速度目标值（占空比，0-10000为正转，负值为后退）
 */
void Motor_SetSpeedWithDirection(uint8_t motor_id, float speed_target);
void PID_Init(void);
// 速度调试模式：仅速度环（位置环可选禁用）
void PID_Control_Update(void);
void PID_PositionLoop_Enable(uint8_t enable);
uint8_t PID_PositionLoop_IsEnabled(void);

/**
 * @brief 启用/禁用陀螺仪阻尼
 * @param enable: 1=启用, 0=禁用（架空测试时使用）
 */
void PID_GyroDamping_Enable(uint8_t enable);

/**
 * @brief 检查陀螺仪阻尼是否启用
 * @return 1=启用, 0=禁用
 */
uint8_t PID_GyroDamping_IsEnabled(void);

/**
 * @brief 设置蓝牙遥控模式下的目标速度
 * @param target_left   左轮目标速度 (编码器单位)
 * @param target_right  右轮目标速度 (编码器单位)  
 * @param duty_cap      PWM 输出限幅值 (软启动斜坡上限)
 * @note 实际 PWM 输出在 SysTick 中断中执行，避免主循环阻塞
 */
void PID_SetBluetoothTarget(float target_left, float target_right, uint16_t duty_cap);

/**
 * @brief 设置位置环全局 PID 参数 (运行时可调)
 * @param kp       比例系数
 * @param ki       积分系数
 * @param kd       微分系数
 * @param gyro_kd  陀螺仪阻尼系数 (用于抑制转弯过冲)
 */
void PositionPID_SetGlobalParams(float kp, float ki, float kd, float gyro_kd);

/**
 * @brief 重置位置环状态 (清零积分与误差历史)
 */
void PositionPID_ResetState(void);

/* ========================================================================== */
/*                          轮子锁定 (Position Hold) 功能                       */
/* ========================================================================== */

/**
 * @brief 启用轮子锁定模式
 * @note  记录当前编码器位置作为目标，使用PID闭环保持轮子不动
 *        适用于蓝牙模式下的IDLE状态，防止因重力滑动
 */
void WheelLock_Enable(void);

/**
 * @brief 禁用轮子锁定模式
 * @note  释放轮子锁定，允许正常运动
 */
void WheelLock_Disable(void);

/**
 * @brief 检查轮子锁定是否启用
 * @return 1=启用, 0=禁用
 */
uint8_t WheelLock_IsEnabled(void);

/**
 * @brief 轮子锁定控制更新 (在SysTick中调用)
 * @note  当锁定启用时，使用PID控制保持轮子在锁定位置
 */
void WheelLock_Update(void);

/* ========================================================================== */
/*                         巡线速度参数接口                                     */
/* ========================================================================== */

/**
 * @brief  设置巡线速度参数 (运行时可调)
 * @param  base_speed   最高速度 (直线/缓弯, 默认 190)
 * @param  min_speed    最低速度 (直角弯, 默认 100)
 * @note   双通道二次曲线: 陀螺仪(主) + 偏差(辅) 叠加减速
 */
void PID_SetLineSpeedParams(float base_speed, float min_speed);

/**
 * @brief  获取巡线速度参数
 * @param  base_speed   输出: 最高速度
 * @param  min_speed    输出: 最低速度
 */
void PID_GetLineSpeedParams(float *base_speed, float *min_speed);

/**
 * @brief  设置陀螺仪满减速角速度 (rad/s)
 */
void PID_SetLineSpeedRateK(float omega_max);

/**
 * @brief  获取陀螺仪满减速角速度
 */
float PID_GetLineSpeedRateK(void);

/**
 * @brief  设置陀螺仪/偏差减速权重
 * @param  gyro_w  陀螺仪权重 (0~1, 默认 0.7)
 * @param  dev_w   偏差权重 (0~1, 默认 0.3)
 */
void PID_SetSpeedWeights(float gyro_w, float dev_w);

/**
 * @brief  获取陀螺仪/偏差减速权重
 */
void PID_GetSpeedWeights(float *gyro_w, float *dev_w);

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
                      float *pwm_left, float *pwm_right);

#endif /* __PID_CONTROLLER_H__ */

