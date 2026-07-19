#ifndef __ROUTE_POINTS_H__
#define __ROUTE_POINTS_H__

/**
 * @file    RoutePoints.h
 * @brief   全局路线里程点统一配置
 * @details
 *  1) 本文件集中维护“所有与里程相关的触发位置”(单位:mm)
 *  2) CircleHandler / SpeedProfile 仅引用这里的宏，不在模块内写硬编码
 *  3) 若要调赛道参数，只改本文件即可，避免多文件数值漂移
 */

/* ========================================================================== */
/* 调参说明                                                                    */
/* ========================================================================== */
/*
 * - 所有里程均以发车后 Odometer_Reset() 的 0 点为基准。
 * - 宏命名约定:
 *     ENTER  : 某路段/元素开始点
 *     EXIT   : 某路段/元素结束点
 *     TRIGGER: 状态机切换触发点
 *     BIAS   : 出环导向切换点(左右侧追线切换)
 * - 建议按“先大后小”原则调参:
 *     先调圆环三个触发点，再细调速度分段点。
 */



/* ========================================================================== */
/* 速度规划里程点 (SpeedProfile 使用, 按赛道顺序)                              */
/* ========================================================================== */
/* 发车直道起点 */
#define ROUTE_START_MM                     0.0f

/* 第1个直角弯: 进入/离开 */
#define ROUTE_TURN1_ENTER_MM              9500.0f
#define ROUTE_TURN1_EXIT_MM               10500.0f

/* 小圆环1: 降速进入/恢复 */
#define ROUTE_SMALL_CIRCLE1_ENTER_MM      14000.0f
#define ROUTE_SMALL_CIRCLE1_EXIT_MM       17500.0f

/* 第2个直角弯: 进入/离开 */
#define ROUTE_TURN2_ENTER_MM              25500.0f
#define ROUTE_TURN2_EXIT_MM               27000.0f

/* 小圆环2: 降速进入/恢复 */
#define ROUTE_SMALL_CIRCLE2_ENTER_MM      30000.0f
#define ROUTE_SMALL_CIRCLE2_EXIT_MM       33000.0f

/* ========================================================================== */
/* 大圆环检测窗口 (CircleHandler 使用)                                         */
/* ========================================================================== */
/* 大圆环入环检测开始: 切到左侧追线 */
#define ROUTE_CIRCLE_ENTRY_DETECT_START_MM 36000.0f

/* 大圆环入环检测结束: 恢复正常追线 */
#define ROUTE_CIRCLE_ENTRY_DETECT_STOP_MM  38000.0f

/* 大圆环出环检测开始: 切到右侧追线 */
#define ROUTE_CIRCLE_EXIT_DETECT_START_MM  40000.0f

/* 大圆环出环检测结束: 恢复正常追线并标记圆环完成 */
#define ROUTE_CIRCLE_EXIT_DETECT_STOP_MM   42000.0f

/* 大圆环降速起点: 从入环检测开始就降速 */
#define ROUTE_CIRCLE_SPEED_SLOW_MM         ROUTE_CIRCLE_ENTRY_DETECT_START_MM

/* 出大圆环后降速点: 继续保持低速，直到停车 */
#define ROUTE_POST_CIRCLE_SPEED_MM         ROUTE_CIRCLE_EXIT_DETECT_STOP_MM

#endif /* __ROUTE_POINTS_H__ */