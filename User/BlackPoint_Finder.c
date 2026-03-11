#include "BlackPoint_Finder.h"
#include <math.h>

// 传感器配置数组 (保留用于兼容性)
static SensorConfig_t sensor_config[SENSOR_COUNT];

// 传感器独立阈值 = Black_Max + (White_Min - Black_Max) * 0.35
// 全白(取最小): 527,516,508,516,586,602,594,587,575,546,597,554,546,496,529,524
// 全黑(取最大): 358,321,280,299,339,342,336,336,335,306,363,337,322,301,330,318
static uint16_t SENSOR_THRESHOLDS[SENSOR_COUNT] = {
    417, 389, 360, 375, 425, 433, 426, 424,
    419, 390, 445, 413, 400, 369, 400, 390
};

// 状态记录用于迟滞 (0=White, 1=Black)
static uint8_t sensor_states[SENSOR_COUNT] = {0};
#define HYSTERESIS 20

// 上一次找到的黑点位置
static uint8_t last_position = 7; // (SENSOR_COUNT - 1) / 2 = 7 (向下取整)
static float last_precise_position = 7.5f; // 精确中心位置
static float g_last_weight_sum = 0.0f;  // 上一次位置计算的总权重

/**
 * @brief 初始化寻点模块
 */
void BlackPoint_Finder_Init(void)
{
	uint8_t i;
	// 初始化所有传感器的默认配置
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		sensor_config[i].min_value = 100; 
		sensor_config[i].max_value = 3500;
		sensor_states[i] = 0;
	}
	
	// 初始化上一次位置为中间
	last_position = 7;
	last_precise_position = 7.5f;
}

/**
 * @brief 设置指定传感器的最小值和最大值
 */
void BlackPoint_Finder_SetSensorConfig(uint8_t sensor_idx, uint16_t min_value, uint16_t max_value)
{
	if(sensor_idx < SENSOR_COUNT)
	{
		sensor_config[sensor_idx].min_value = min_value;
		sensor_config[sensor_idx].max_value = max_value;
	}
}

/**
 * @brief 获取指定传感器的配置
 */
void BlackPoint_Finder_GetSensorConfig(uint8_t sensor_idx, uint16_t *min_value, uint16_t *max_value)
{
	if(sensor_idx < SENSOR_COUNT && min_value != NULL && max_value != NULL)
	{
		*min_value = sensor_config[sensor_idx].min_value;
		*max_value = sensor_config[sensor_idx].max_value;
	}
}

/**
 * @brief 判断指定传感器是否为黑点 (独立阈值+迟滞)
 * @return 1=黑点，0=白点
 */
uint8_t BlackPoint_Finder_IsBlackPoint(uint8_t sensor_idx, uint16_t adc_value)
{
	uint16_t threshold;
    uint16_t lower, upper;
	
	if(sensor_idx >= SENSOR_COUNT)
		return 0;
	
    threshold = SENSOR_THRESHOLDS[sensor_idx];
    
    // 迟滞判断: Black < (Thr - Hys), White > (Thr + Hys)
    // 根据数据: Black值低, White值高
    lower = (threshold > HYSTERESIS) ? (threshold - HYSTERESIS) : 0;
    upper = threshold + HYSTERESIS;
    
    if (adc_value < lower) {
        sensor_states[sensor_idx] = 1; // 变黑
    } else if (adc_value > upper) {
        sensor_states[sensor_idx] = 0; // 变白
    }
    // 否则保持上一状态
    
	return sensor_states[sensor_idx];
}

/**
 * @brief 获取所有通道的黑白状态 (使用IsBlackPoint的统一逻辑)
 */
void BlackPoint_Finder_GetBlackFlags_Dynamic(volatile uint16_t *adc_values, uint8_t *black_flags)
{
	uint8_t i;
	
	if(adc_values == NULL || black_flags == NULL)
		return;
	
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		black_flags[i] = BlackPoint_Finder_IsBlackPoint(i, adc_values[i]);
	}
}

/**
 * @brief 寻点主函数：基于独立阈值判断找黑线
 * @param adc_values: ADC值数组
 * @param result: 输出结果结构体指针
 * @return 返回找到的黑点精确位置（浮点数）
 * @note 使用 IsBlackPoint 独立判断每一路，完全摒弃整体排序逻辑
 */
float BlackPoint_Finder_Search(volatile uint16_t *adc_values, BlackPointResult_t *result)
{
	uint8_t i;
    float pos_sum = 0.0f;
    float weight_sum = 0.0f;
    uint8_t black_count = 0;
	float precise_pos = 0.0f;
	
	if(adc_values == NULL || result == NULL)
	{
		if(result != NULL)
		{
			result->found = 0;
			result->position = last_position;
			result->precise_position = last_precise_position;
		}
		return last_precise_position;
	}
	
    // 遍历所有传感器，独立判断
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		// 使用独立阈值判断是否黑线
        if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
		{
			black_count++;
            
            // 简单的加权计算位置
            // 这里使用基础权重 100 + 黑度差值 (Threshold - Value)
            float diff = 0.0f;
            uint16_t thr = SENSOR_THRESHOLDS[i];
            
            if(thr > adc_values[i]) {
                diff = (float)(thr - adc_values[i]);
            }
            
			float weight = 100.0f + diff; 
			pos_sum += (float)i * weight;
			weight_sum += weight;
		}
	}
	
	// 保存本次权重总和（黑线未检测到时为0）
	g_last_weight_sum = weight_sum;
	
	// 没有找到黑线
	if(black_count == 0)
	{
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		return last_precise_position;
	}
	
	// 计算精确位置
	if(weight_sum > 0.1f)
	{
		precise_pos = pos_sum / weight_sum;
	}
	
	// 边缘噪声过滤: 只有最外侧1个传感器检测到黑色(单点),
	// 且与上次位置跨越几乎全幅, 才视为地图外地面噪声
	// 注意: 直角弯时线会合法地从中心跳到边缘, 不能误杀
	if(black_count == 1)
	{
		uint8_t all_left_edge = (precise_pos < 0.5f);
		uint8_t all_right_edge = (precise_pos > (float)(SENSOR_COUNT - 1) - 0.5f);
		float jump = fabsf(precise_pos - last_precise_position);
		// 仅单点+最外侧+跨越几乎全幅(13通道以上)才过滤
		if(jump > 13.0f)
		{
			if(all_left_edge || all_right_edge)
			{
				result->found = 0;
				result->position = last_position;
				result->precise_position = last_precise_position;
				return last_precise_position;
			}
		}
	}
	
	// 限制范围
	if(precise_pos < 0.0f) precise_pos = 0.0f;
	if(precise_pos > (float)(SENSOR_COUNT - 1)) precise_pos = (float)(SENSOR_COUNT - 1);
	
	// 更新结果
	result->found = 1;
	result->position = (uint8_t)(precise_pos + 0.5f);
	result->precise_position = precise_pos;
	last_position = result->position;
	last_precise_position = precise_pos;
	
	return precise_pos;
}

/**
 * @brief 获取上一次找到的黑点位置
 */
uint8_t BlackPoint_Finder_GetLastPosition(void)
{
	return last_position;
}

/**
 * @brief 重置上一次位置为中间位置
 */
void BlackPoint_Finder_ResetLastPosition(void)
{
	last_position = SENSOR_COUNT / 2;
}

/**
 * @brief 获取上一次黑线位置计算的总权重
 * @return 权重总和（0表示未检测到黑线）
 */
float BlackPoint_Finder_GetLastWeightSum(void)
{
	return g_last_weight_sum;
}

void BlackPoint_Finder_SetThreshold(uint8_t sensor_idx, uint16_t threshold)
{
	if(sensor_idx < SENSOR_COUNT)
	{
		SENSOR_THRESHOLDS[sensor_idx] = threshold;
	}
}

uint16_t BlackPoint_Finder_GetThreshold(uint8_t sensor_idx)
{
	if(sensor_idx < SENSOR_COUNT)
		return SENSOR_THRESHOLDS[sensor_idx];
	return 0;
}

// ==================== 模拟黑点测试功能 ====================
static uint8_t g_simulate_mode = 0;                           // 模拟模式开关
static volatile uint16_t g_simulate_adc[SENSOR_COUNT];        // 模拟ADC值
static uint8_t g_simulate_channels[SENSOR_COUNT];             // 模拟黑点通道
static uint8_t g_simulate_channel_count = 0;                  // 模拟黑点通道数量

/**
 * @brief 更新模拟ADC值
 * @note 黑点通道设为最小值（检测到黑线），其他通道设为最大值（白色）
 */
static void BlackPoint_Finder_UpdateSimulateADC(void)
{
	uint8_t i, j;
	uint8_t is_black;
	
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		is_black = 0;
		for(j = 0; j < g_simulate_channel_count; j++)
		{
			if(g_simulate_channels[j] == i)
			{
				is_black = 1;
				break;
			}
		}
		
		if(is_black)
		{
			// 黑点：设置为略高于最小值（模拟检测到黑线）
			g_simulate_adc[i] = sensor_config[i].min_value + 10;
		}
		else
		{
			// 白点：设置为接近最大值（模拟白纸）
			g_simulate_adc[i] = sensor_config[i].max_value - 10;
		}
	}
}

/**
 * @brief 设置模拟黑点模式
 */
void BlackPoint_Finder_SetSimulateMode(uint8_t enable)
{
	g_simulate_mode = enable ? 1 : 0;
	
	if(enable && g_simulate_channel_count == 0)
	{
		// 默认中间两路(7,8)检测到黑线
		g_simulate_channels[0] = 7;
		g_simulate_channels[1] = 8;
		g_simulate_channel_count = 2;
	}
	
	if(enable)
	{
		BlackPoint_Finder_UpdateSimulateADC();
	}
}

/**
 * @brief 检查是否处于模拟模式
 */
uint8_t BlackPoint_Finder_IsSimulateMode(void)
{
	return g_simulate_mode;
}

/**
 * @brief 设置模拟黑点位置
 */
void BlackPoint_Finder_SetSimulateChannels(const uint8_t *channels, uint8_t count)
{
	uint8_t i;
	
	if(channels == NULL || count == 0)
	{
		// 默认中间两路
		g_simulate_channels[0] = 7;
		g_simulate_channels[1] = 8;
		g_simulate_channel_count = 2;
	}
	else
	{
		if(count > SENSOR_COUNT) count = SENSOR_COUNT;
		
		for(i = 0; i < count; i++)
		{
			if(channels[i] < SENSOR_COUNT)
			{
				g_simulate_channels[i] = channels[i];
			}
		}
		g_simulate_channel_count = count;
	}
	
	// 更新模拟ADC值
	BlackPoint_Finder_UpdateSimulateADC();
}

/**
 * @brief 获取模拟ADC值数组
 */
volatile uint16_t* BlackPoint_Finder_GetSimulateADC(void)
{
	return g_simulate_adc;
}

/* ========================================================================== */
/*                          路口/圆环检测辅助函数                               */
/* ========================================================================== */

/**
 * @brief  分析当前传感器帧的路口特征
 */
void BlackPoint_Finder_AnalyzeJunction(volatile uint16_t *adc_values, JunctionInfo_t *info)
{
	uint8_t i;
	uint8_t black_flags[SENSOR_COUNT];
	uint8_t black_count = 0;
	uint8_t seg_count = 0;
	uint8_t in_segment = 0;
	uint8_t left_count = 0;
	uint8_t right_count = 0;
	
	if(adc_values == NULL || info == NULL) return;
	
	/* 获取所有传感器黑白状态 */
	BlackPoint_Finder_GetBlackFlags_Dynamic(adc_values, black_flags);
	
	/* 统计黑点数量和线段数量 */
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		if(black_flags[i])
		{
			black_count++;
			if(!in_segment)
			{
				seg_count++;
				in_segment = 1;
			}
			/* 左半侧: 0~6, 右半侧: 9~15 */
			if(i <= 6) left_count++;
			if(i >= 9) right_count++;
		}
		else
		{
			in_segment = 0;
		}
	}
	
	/* 统计中间区域 (6~9) */
	{
		uint8_t center_count = 0;
		uint8_t ci;
		for(ci = 6; ci <= 9; ci++)
		{
			if(black_flags[ci]) center_count++;
		}
		info->center_has_line = (center_count >= 1) ? 1 : 0;
	}
	
	info->black_count = black_count;
	info->seg_count = seg_count;
	info->left_has_line = (left_count >= 2) ? 1 : 0;
	info->right_has_line = (right_count >= 2) ? 1 : 0;
	
	/* 路口判定: 黑点数 >= 6 或出现两段以上分离的黑线 */
	info->is_junction = (black_count >= 6 || seg_count >= 2) ? 1 : 0;
	
	/* 十字判定: 路口 + 左右两侧都有线 + 中间有线 */
	info->is_cross = (info->is_junction && info->left_has_line && info->right_has_line && info->center_has_line) ? 1 : 0;
}

/**
 * @brief  半侧追线：只取指定半侧传感器的加权中心
 */
float BlackPoint_Finder_SearchHalf(volatile uint16_t *adc_values, uint8_t side, BlackPointResult_t *result)
{
	uint8_t i;
	uint8_t start_idx, end_idx;
	float pos_sum = 0.0f;
	float weight_sum = 0.0f;
	uint8_t black_count = 0;
	float precise_pos;
	
	if(adc_values == NULL || result == NULL)
	{
		if(result != NULL)
		{
			result->found = 0;
			result->position = 7;
			result->precise_position = 7.5f;
		}
		return 7.5f;
	}
	
	if(side == 0)
	{
		start_idx = 0;
		end_idx = 8;    /* 左半侧 0~7 */
	}
	else
	{
		start_idx = 8;
		end_idx = 16;   /* 右半侧 8~15 */
	}
	
	for(i = start_idx; i < end_idx; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
		{
			float diff = 0.0f;
			uint16_t thr = SENSOR_THRESHOLDS[i];
			if(thr > adc_values[i])
			{
				diff = (float)(thr - adc_values[i]);
			}
			float weight = 100.0f + diff;
			pos_sum += (float)i * weight;
			weight_sum += weight;
			black_count++;
		}
	}
	
	if(black_count == 0)
	{
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		return last_precise_position;
	}
	
	precise_pos = pos_sum / weight_sum;
	if(precise_pos < 0.0f) precise_pos = 0.0f;
	if(precise_pos > (float)(SENSOR_COUNT - 1)) precise_pos = (float)(SENSOR_COUNT - 1);
	
	result->found = 1;
	result->position = (uint8_t)(precise_pos + 0.5f);
	result->precise_position = precise_pos;
	last_position = result->position;
	last_precise_position = precise_pos;
	
	return precise_pos;
}

