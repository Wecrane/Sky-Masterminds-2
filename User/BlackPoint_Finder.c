#include "BlackPoint_Finder.h"
#include <math.h>

// 传感器配置数组 (保留用于兼容性)
static SensorConfig_t sensor_config[SENSOR_COUNT];

// 传感器独立阈值 (Min_White + Max_Black) / 2
// 全白: 429,465,432,464,389,449,388,451,390,361,390,357,394,421,377,420
// 全黑: 349,347,273,338,281,286,325,337,296,284,373,263,313,289,292,345
static const uint16_t SENSOR_THRESHOLDS[SENSOR_COUNT] = {
    389, 406, 352, 401, 335, 367, 356, 394,
    343, 322, 381, 310, 353, 355, 334, 382
};

// 状态记录用于迟滞 (0=White, 1=Black)
static uint8_t sensor_states[SENSOR_COUNT] = {0};
#define HYSTERESIS 20

// 上一次找到的黑点位置
static uint8_t last_position = 7; // (SENSOR_COUNT - 1) / 2 = 7 (向下取整)
static float last_precise_position = 7.5f; // 精确中心位置

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

