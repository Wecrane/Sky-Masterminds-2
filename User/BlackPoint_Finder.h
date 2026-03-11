#ifndef __BLACKPOINT_FINDER_H__
#define __BLACKPOINT_FINDER_H__

#include "stm32f10x.h"
#include <stdint.h>
#include <float.h>
#include "stdio.h"

// 传感器数量（可根据实际硬件调整）
#define SENSOR_COUNT 16

// 黑点判断阈值百分比（40%）
#define BLACK_POINT_THRESHOLD_PERCENT 0.4f

/**
 * @brief 传感器配置结构体
 * @param min_value: 最小值（白纸时的ADC值）
 * @param max_value: 最大值（最黑时的ADC值）
 */
typedef struct
{
	uint16_t min_value;  // 最小值
	uint16_t max_value;  // 最大值
} SensorConfig_t;

/**
 * @brief 寻点结果结构体
 * @param found: 是否找到黑点（1=找到，0=未找到）
 * @param position: 黑点位置（传感器索引，0~SENSOR_COUNT-1）
 * @param precise_position: 精确位置（浮点数，精度提高10倍，例如7.5表示在传感器7和8之间）
 */
typedef struct
{
	uint8_t found;           // 是否找到黑点
	uint8_t position;        // 黑点位置（整数）
	float precise_position;  // 精确位置（浮点数，精度提高10倍）
} BlackPointResult_t;

/**
 * @brief 初始化寻点模块
 * @note 设置默认的最小值和最大值参数，可根据实际硬件调整
 */
void BlackPoint_Finder_Init(void);

/**
 * @brief 设置指定传感器的最小值和最大值
 * @param sensor_idx: 传感器索引（0~SENSOR_COUNT-1）
 * @param min_value: 最小值
 * @param max_value: 最大值
 */
void BlackPoint_Finder_SetSensorConfig(uint8_t sensor_idx, uint16_t min_value, uint16_t max_value);

/**
 * @brief 获取指定传感器的配置
 * @param sensor_idx: 传感器索引（0~SENSOR_COUNT-1）
 * @param min_value: 输出最小值指针
 * @param max_value: 输出最大值指针
 */
void BlackPoint_Finder_GetSensorConfig(uint8_t sensor_idx, uint16_t *min_value, uint16_t *max_value);

/**
 * @brief 寻点主函数：遍历所有点找归一化最小值，用左右两点插值提高精度
 * @param adc_values: ADC值数组（长度至少为SENSOR_COUNT）
 * @param result: 输出结果结构体指针
 * @return 返回找到的黑点精确位置（浮点数），如果未找到则保持上一个精确位置
 * @note 算法流程：
 *       1. 对所有点进行归一化处理
 *       2. 找到归一化后的最小值
 *       3. 验证最小值是否比平均值低20%
 *       4. 如果满足条件，用最小值左右两边的点进行插值计算精确位置（精度提高10倍）
 */
float BlackPoint_Finder_Search(volatile uint16_t *adc_values, BlackPointResult_t *result);

/**
 * @brief 判断指定传感器是否为黑点
 * @param sensor_idx: 传感器索引（0~SENSOR_COUNT-1）
 * @param adc_value: ADC值
 * @return 1=黑点，0=白点
 */
uint8_t BlackPoint_Finder_IsBlackPoint(uint8_t sensor_idx, uint16_t adc_value);

/**
 * @brief 使用动态阈值判断所有传感器的黑白状态
 * @param adc_values: ADC值数组（长度至少为SENSOR_COUNT）
 * @param black_flags: 输出黑白状态数组（1=黑点，0=白点）
 * @note 使用当前帧的动态范围进行判断，适应高度变化
 */
void BlackPoint_Finder_GetBlackFlags_Dynamic(volatile uint16_t *adc_values, uint8_t *black_flags);

/**
 * @brief 获取上一次找到的黑点位置
 * @return 黑点位置（0~SENSOR_COUNT-1），如果从未找到则返回中间位置
 */
uint8_t BlackPoint_Finder_GetLastPosition(void);

/**
 * @brief 重置上一次位置为中间位置
 */
void BlackPoint_Finder_ResetLastPosition(void);

/**
 * @brief 获取上一次黑线位置计算的总权重
 * @return 权重总和（0表示未检测到黑线）
 */
float BlackPoint_Finder_GetLastWeightSum(void);

/**
 * @brief 设置指定传感器的阈值（用于校准）
 * @param sensor_idx: 传感器索引（0~SENSOR_COUNT-1）
 * @param threshold: 新阈值
 */
void BlackPoint_Finder_SetThreshold(uint8_t sensor_idx, uint16_t threshold);

/**
 * @brief 获取指定传感器的当前阈值
 * @param sensor_idx: 传感器索引（0~SENSOR_COUNT-1）
 * @return 当前阈值
 */
uint16_t BlackPoint_Finder_GetThreshold(uint8_t sensor_idx);

/**
 * @brief 设置模拟黑点模式
 * @param enable: 1=启用模拟模式, 0=禁用（使用真实ADC）
 */
void BlackPoint_Finder_SetSimulateMode(uint8_t enable);

/**
 * @brief 检查是否处于模拟模式
 * @return 1=模拟模式, 0=正常模式
 */
uint8_t BlackPoint_Finder_IsSimulateMode(void);

/**
 * @brief 设置模拟黑点位置
 * @param channels: 检测到黑线的通道数组
 * @param count: 通道数量
 * @note 默认中间两路(7,8)检测到黑线
 */
void BlackPoint_Finder_SetSimulateChannels(const uint8_t *channels, uint8_t count);

/**
 * @brief 获取模拟ADC值数组（用于位置环计算）
 * @return 模拟的ADC值数组指针
 */
volatile uint16_t* BlackPoint_Finder_GetSimulateADC(void);

/* ========================================================================== */
/*                          路口/圆环检测辅助函数                               */
/* ========================================================================== */

/**
 * @brief 路口检测结果结构体
 */
typedef struct {
	uint8_t is_junction;       /* 是否检测到路口 (黑线异常宽或出现分叉) */
	uint8_t is_cross;          /* 是否为十字路口 (左右两侧都有线) */
	uint8_t black_count;       /* 当前帧中检测到的黑点数量 */
	uint8_t seg_count;         /* 黑线段数 (1=普通线, 2=分叉/路口) */
	uint8_t left_has_line;     /* 左半侧 (0~6) 是否有黑线 */
	uint8_t right_has_line;    /* 右半侧 (9~15) 是否有黑线 */
	uint8_t center_has_line;   /* 中间区域 (6~9) 是否有黑线 */
} JunctionInfo_t;

/**
 * @brief  分析当前传感器帧的路口特征
 * @param  adc_values  ADC 值数组
 * @param  info        输出路口信息
 */
void BlackPoint_Finder_AnalyzeJunction(volatile uint16_t *adc_values, JunctionInfo_t *info);

/**
 * @brief  半侧追线：只取指定半侧传感器的加权中心
 * @param  adc_values  ADC 值数组
 * @param  side        0=左半侧(0~7), 1=右半侧(8~15)
 * @param  result      输出结果
 * @return 精确位置 (与 BlackPoint_Finder_Search 一致)
 */
float BlackPoint_Finder_SearchHalf(volatile uint16_t *adc_values, uint8_t side, BlackPointResult_t *result);

#endif // __BLACKPOINT_FINDER_H__

