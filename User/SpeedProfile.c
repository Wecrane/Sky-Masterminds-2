/**
 * @file    SpeedProfile.c
 * @brief   固定路线分段减速模块实现
 * @details 根据编码器累计里程 (Odometer location) 前瞻性调整目标速度。
 *
 *          原理:
 *          - 维护一张按里程排序的"路线速度表" s_route[]
 *          - 当接近低速路段时, 提前 DECEL_LOOKAHEAD_MM 开始线性减速
 *          - 离开低速路段后, 经过 ACCEL_RAMP_MM 线性加速回高速
 *          - 陀螺仪/偏差减速机制仍然在此基础上叠加生效 (双重保险)
 *
 *          调参步骤:
 *          1. 让小车以低速 (如90) 跑完全程, 通过串口打印里程计值
 *          2. 记录各弯道处的里程读数, 填入 s_route[] 表
 *          3. 调整 DECEL_LOOKAHEAD_MM (前瞻距离) 和 ACCEL_RAMP_MM (回速距离)
 *
 *          速度曲线示意:
 *
 *            速度
 *             ^
 *             |  ___________                    ___________
 *             | |           \                  /           |
 *             | |    直道    \     弯道       /    直道    |
 *             | |             \______________/             |
 *             |________________________________________________> 里程
 *                            ^               ^
 *                       前瞻减速开始      加速恢复结束
 */

#include "SpeedProfile.h"
#include <stdio.h>

/* ========================================================================== */
/*                              可调参数                                       */
/* ========================================================================== */

/** 前瞻减速距离: 在到达弯道前多少mm开始减速
 *  越大 → 越提前、越平滑, 但直道末段速度更低
 *  建议: 150~300mm, 根据实际最高速度和刹车能力调整 */
#define DECEL_LOOKAHEAD_MM   200.0f

/** 加速恢复距离: 离开弯道后多少mm恢复高速
 *  越大 → 加速越缓, 越稳定
 *  建议: 80~200mm */
#define ACCEL_RAMP_MM        100.0f

/** 默认高速速度 (直线段) */
#define SPEED_HIGH           190.0f

/** 默认低速速度 (直角弯) */
#define SPEED_LOW            140.0f

/** 小圆环降速 */
#define SPEED_SMALL_CIRCLE   160.0f

/** 出环后降速 */
#define SPEED_POST_CIRCLE    110.0f

/* ========================================================================== */
/*               路线速度表 (实测数据)                                         */
/* ========================================================================== */
/**
 * 实测里程 (两次取平均):
 *   第1个直角弯: (10480 + 10434) / 2 ≈ 10457 mm
 *   第2个直角弯: (28449 + 28285) / 2 ≈ 28367 mm
 *   圆环入环:    (39992 + 40060) / 2 ≈ 40026 mm (由CircleHandler处理)
 *   圆环出环:    48000 mm (由CircleHandler处理)
 */
static const RouteWaypoint_t s_route[] = {
    /*  里程(mm)        速度              说明                    */
    {      0.0f,    SPEED_HIGH     },  /* 起步直道               */
    {  10300.0f,    SPEED_LOW      },  /* 第1个直角弯             */
    {  11300.0f,    SPEED_HIGH     },  /* 弯后直道                */
    {  17400.0f,    SPEED_SMALL_CIRCLE }, /* 小圆环减速             */
    {  20300.0f,    SPEED_HIGH     },  /* 小圆环后恢复高速        */
    {  28700.0f,    SPEED_LOW      },  /* 第2个直角弯             */
    {  29700.0f,    SPEED_HIGH     },  /* 弯后直道                */
    {  33500.0f,    SPEED_SMALL_CIRCLE }, /* 34000-36000mm降速到160  */
    {  34500.0f,    SPEED_HIGH     },  /* 恢复高速 → 圆环入口前   */
    {  39600.0f,    SPEED_SMALL_CIRCLE }, /* 圆环内速度 160         */
    {  48000.0f,    SPEED_POST_CIRCLE }, /* 出环后降速到110        */
};

#define ROUTE_COUNT  (sizeof(s_route) / sizeof(s_route[0]))

/* ========================================================================== */
/*                              内部状态                                       */
/* ========================================================================== */
static uint8_t  s_enabled     = 1;
static uint16_t s_current_idx = 0;  /* 当前所在路段索引 (递增查找优化) */
static uint16_t s_last_printed_idx = 0xFFFF; /* 上次打印的路段索引 (避免重复打印) */

/* 状态枚举: 用于打印状态变化 */
typedef enum {
    SPD_STATE_CRUISING = 0,  /* 巡航 (恒速) */
    SPD_STATE_DECELING,      /* 前瞻减速中 */
    SPD_STATE_ACCELING       /* 加速恢复中 */
} SpdState_t;
static SpdState_t s_last_state = SPD_STATE_CRUISING;

/* ========================================================================== */
/*                              公有函数                                       */
/* ========================================================================== */

void SpeedProfile_Init(void)
{
    s_current_idx = 0;
    s_enabled = 1;
}

void SpeedProfile_Reset(void)
{
    s_current_idx = 0;
    s_last_printed_idx = 0xFFFF;
    s_last_state = SPD_STATE_CRUISING;
}

void SpeedProfile_Enable(uint8_t en)
{
    s_enabled = en ? 1 : 0;
}

uint8_t SpeedProfile_IsEnabled(void)
{
    return s_enabled;
}

float SpeedProfile_GetTargetSpeed(float d)
{
    if (!s_enabled || ROUTE_COUNT == 0)
        return SPEED_HIGH;

    /* 里程复位瞬间可能出现负值, 直接返回高速 */
    if (d < 0.0f) return SPEED_HIGH;

    /* 顺序查找当前路段 (利用里程单调递增, 从上次位置开始找) */
    while (s_current_idx > 0 && d < s_route[s_current_idx].distance_mm)
        s_current_idx--;

    while (s_current_idx + 1 < ROUTE_COUNT &&
           d >= s_route[s_current_idx + 1].distance_mm)
        s_current_idx++;

    float current_speed = s_route[s_current_idx].speed;

    if (s_current_idx != s_last_printed_idx)
        s_last_printed_idx = s_current_idx;

    /* ---- 前瞻减速: 下一路段速度更低时, 提前开始线性减速 ---- */
    if (s_current_idx + 1 < ROUTE_COUNT)
    {
        float next_speed = s_route[s_current_idx + 1].speed;
        float next_dist  = s_route[s_current_idx + 1].distance_mm;
        float remaining  = next_dist - d;

        if (next_speed < current_speed &&
            remaining < DECEL_LOOKAHEAD_MM && remaining > 0.0f)
        {
            if (s_last_state != SPD_STATE_DECELING)
            {
                s_last_state = SPD_STATE_DECELING;
                printf("[SPD] decel %.0f->%.0f @%.0fmm\r\n", current_speed, next_speed, d);
            }
            /* 线性插值: remaining 从 DECEL_LOOKAHEAD_MM→0 时速度从 current→next */
            float ratio = remaining / DECEL_LOOKAHEAD_MM;
            return next_speed + (current_speed - next_speed) * ratio;
        }
    }

    /* ---- 加速恢复: 刚从低速路段进入高速路段时, 线性加速 ---- */
    if (s_current_idx > 0)
    {
        float prev_speed  = s_route[s_current_idx - 1].speed;
        float seg_start   = s_route[s_current_idx].distance_mm;
        float dist_in_seg = d - seg_start;

        if (prev_speed < current_speed &&
            dist_in_seg < ACCEL_RAMP_MM && dist_in_seg >= 0.0f)
        {
            if (s_last_state != SPD_STATE_ACCELING)
            {
                s_last_state = SPD_STATE_ACCELING;
                printf("[SPD] accel %.0f->%.0f @%.0fmm\r\n", prev_speed, current_speed, d);
            }
            /* 线性插值: dist_in_seg 从 0→ACCEL_RAMP_MM 时速度从 prev→current */
            float ratio = dist_in_seg / ACCEL_RAMP_MM;
            return prev_speed + (current_speed - prev_speed) * ratio;
        }
    }

    if (s_last_state != SPD_STATE_CRUISING)
    {
        s_last_state = SPD_STATE_CRUISING;
        printf("[SPD] cruise=%.0f @%.0fmm\r\n", current_speed, d);
    }
    return current_speed;
}
