/**
 * @file    Odometer.h
 * @brief   里程计模块 - 累计行驶距离与坐标定位
 * @details 基于编码器和陀螺仪实现:
 *          - 累计行驶距离 (location)
 *          - 二维坐标定位 (x, y)
 *          - 航向角跟踪 (theta)
 *          
 * 使用方法:
 *          1. 在初始化时调用 Odometer_Init()
 *          2. 在 SysTick 中断中调用 Odometer_Update()
 *          3. 调用 Odometer_Reset() 重置坐标系到当前位置
 *          4. 使用 Odometer_GetXXX() 函数获取位置信息
 *          
 * 校准方法:
 *          1. 测量小车直线行驶一段已知距离 (如 1000mm)
 *          2. 读取编码器累计脉冲数
 *          3. 计算: ENCODER_PULSE_PER_MM = 脉冲数 / 实际距离(mm)
 *          4. 更新 ODOM_ENCODER_PULSE_PER_MM 宏定义
 *          
 *          或者使用自动校准:
 *          1. 调用 Odometer_StartCalibration() 开始校准
 *          2. 手动推动小车直线行驶已知距离
 *          3. 调用 Odometer_EndCalibration(distance_mm) 传入实际距离
 *          4. 读取校准结果用于更新代码
 */

#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "stm32f10x.h"

/* ========================================================================== */
/*                              校准参数配置                                   */
/* ========================================================================== */

/**
 * @brief  编码器每毫米脉冲数
 * @note   需要根据实际测量校准:
 *         - 编码器线数 (如 13线)
 *         - 电机减速比 (如 30:1)
 *         - 轮胎直径 (如 42mm)
 *         
 *         理论计算:
 *         pulse_per_rev = 编码器线数 × 4 × 减速比 = 13 × 4 × 30 = 1560
 *         wheel_circumference = π × 直径 = 3.14159 × 42 = 131.95mm
 *         ENCODER_PULSE_PER_MM = 1560 / 131.95 ≈ 11.82
 *         
 *         实际校准更准确!
 */
#define ODOM_ENCODER_PULSE_PER_MM   38.897f

/**
 * @brief  左右轮轴距 (单位: mm)
 * @note   两轮中心之间的距离，用于差速转向计算
 *         需要测量实际值并更新
 */
#define ODOM_WHEEL_BASE_MM          155.0f

/* ========================================================================== */
/*                              数据结构定义                                   */
/* ========================================================================== */

/**
 * @brief  里程计数据结构
 */
typedef struct {
    float location;     /* 累计行驶距离 (mm)，始终为正，表示总里程 */
    float x;            /* X坐标 (mm)，以 Reset 点为原点 */
    float y;            /* Y坐标 (mm)，以 Reset 点为原点 */
    float theta;        /* 航向角 (度)，以 Reset 时的方向为 0 度 */
} Odometer_Data_t;

/**
 * @brief  校准状态结构
 */
typedef struct {
    uint8_t active;              /* 校准是否激活 */
    int32_t start_left_cnt;      /* 校准开始时左轮编码器值 */
    int32_t start_right_cnt;     /* 校准开始时右轮编码器值 */
} Odometer_Calibration_t;

/* ========================================================================== */
/*                              函数声明                                       */
/* ========================================================================== */

/**
 * @brief  初始化里程计模块
 * @note   系统启动时调用一次
 */
void Odometer_Init(void);

/**
 * @brief  更新里程计数据
 * @note   在 SysTick 中断中调用，每 2ms 执行一次
 *         必须在 ABEncoder_UpdateSpeed() 之后调用
 */
void Odometer_Update(void);

/**
 * @brief  重置坐标系
 * @note   将当前位置设为坐标原点 (0, 0)
 *         当前航向设为 0 度
 *         累计里程清零
 *         通常在巡线模式启动时调用
 */
void Odometer_Reset(void);

/**
 * @brief  仅重置坐标，保留累计里程
 * @note   将当前位置设为坐标原点 (0, 0)
 *         当前航向设为 0 度
 *         累计里程不变
 */
void Odometer_ResetCoordinate(void);

/**
 * @brief  获取累计行驶距离
 * @return 累计行驶距离 (mm)
 */
float Odometer_GetLocation(void);

/**
 * @brief  获取当前 X 坐标
 * @return X 坐标 (mm)
 */
float Odometer_GetX(void);

/**
 * @brief  获取当前 Y 坐标
 * @return Y 坐标 (mm)
 */
float Odometer_GetY(void);

/**
 * @brief  获取当前航向角
 * @return 航向角 (度)，范围 -180 ~ 180
 */
float Odometer_GetTheta(void);

/**
 * @brief  获取完整的里程计数据
 * @param  data  输出数据结构指针
 */
void Odometer_GetData(Odometer_Data_t *data);

/* ========================================================================== */
/*                              校准相关函数                                   */
/* ========================================================================== */

/**
 * @brief  开始编码器校准
 * @note   调用此函数后，手动推动小车直线行驶一段已知距离
 */
void Odometer_StartCalibration(void);

/**
 * @brief  结束编码器校准并计算参数
 * @param  actual_distance_mm  实际行驶距离 (mm)
 * @return 计算得到的 pulse_per_mm 值，需要更新到 ODOM_ENCODER_PULSE_PER_MM
 */
float Odometer_EndCalibration(float actual_distance_mm);

/**
 * @brief  检查校准是否激活
 * @return 1=激活, 0=未激活
 */
uint8_t Odometer_IsCalibrating(void);

/**
 * @brief  获取校准过程中的累计脉冲数 (用于调试)
 * @return 校准开始后的平均脉冲数
 */
int32_t Odometer_GetCalibrationPulseCount(void);

/* ========================================================================== */
/*                              陀螺仪融合选项                                 */
/* ========================================================================== */

/**
 * @brief  启用/禁用陀螺仪航向融合
 * @param  enable  1=使用陀螺仪, 0=仅用编码器
 * @note   陀螺仪可以提高转向精度，但需要正确校准
 */
void Odometer_EnableGyroFusion(uint8_t enable);

/**
 * @brief  设置陀螺仪融合权重
 * @param  weight  陀螺仪权重 (0.0~1.0)，默认 0.9
 * @note   weight=1.0 完全信任陀螺仪，weight=0.0 完全信任编码器
 */
void Odometer_SetGyroFusionWeight(float weight);

#endif /* __ODOMETER_H */
