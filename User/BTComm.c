#include "BTComm.h"
#include "Uart_Config.h"
#include "Motor_ctr.h"
#include "M3PWM.h"
#include "PID_Controller.h"
#include "BlackPoint_Finder.h"
#include "ADC_get.h"
#include "RGB_Led.h"
#include "Odometer.h"
#include "ABEncoder.h"
#include "CircleHandler.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*=== 外部变量引用（用于模式切换） ===*/
extern uint8_t star_car;                        /* 巡线模式标志 */
extern volatile uint8_t g_bt_key_control_mode;  /* 蓝牙遥控模式标志 */
extern int16_t position_get;                    /* 传感器检测到的黑线位置 (放大10倍) */
extern volatile uint32_t g_systick_ms;          /* 系统滴答计时器 (毫秒) */

/*=== 接收缓冲区和状态 ===*/
static char g_cmd_buffer[BT_CMD_BUFFER_SIZE];
static uint8_t g_cmd_index = 0;
static uint8_t g_cmd_overflow = 0;  // 命令溢出标志，溢出后等待换行再恢复

/*=== 当前运动状态 ===*/
static Motion_Mode_t g_motion_mode = MOTION_IDLE;
static uint8_t g_motion_speed = 0;        // 当前运动速度 0-100
static uint8_t g_base_speed = 30;         // 基础速度（默认30%）
static uint8_t g_vacuum_enabled = 0;      // 吸盘是否启用
static uint8_t g_vacuum_speed = 0;       // 吸盘转速 0-100
static uint8_t g_key_control_mode = 0;    // 键控模式标志 0=OFF, 1=ON
static uint8_t g_emergency_stop = 0;      // 急停标志 0=正常, 1=急停

/*=== 实时调试数据打印状态 ===*/
static uint8_t g_live_debug = 0;            /* 实时数据打印开关: 0=关, 1=开 */
static uint32_t g_live_debug_last_ms = 0;   /* 上次打印时间戳 (ms) */
#define LIVE_DEBUG_INTERVAL_MS  100         /* 打印间隔 (100ms) */

/*=== 光电管调试打印状态 ===*/
static uint8_t g_sensor_debug = 0;            /* 光电管调试开关: 0=关, 1=开 */
static uint32_t g_sensor_debug_last_ms = 0;   /* 上次打印时间戳 */
#define SENSOR_DEBUG_INTERVAL_MS  50           /* 打印间隔 (50ms, 高频) */

/*=== 吸盘默认转速配置 ===*/
#define VACUUM_DEFAULT_SPEED  0          // 吸盘默认转速百分比

/**
 * @brief 初始化蓝牙通信模块
 */
void BT_Init(void)
{
    g_cmd_index = 0;
    g_cmd_overflow = 0;
    memset(g_cmd_buffer, 0, sizeof(g_cmd_buffer));
    g_motion_mode = MOTION_IDLE;
    g_motion_speed = 0;
    g_base_speed = 30;
    g_vacuum_enabled = 0;
    g_vacuum_speed = VACUUM_DEFAULT_SPEED;
    g_key_control_mode = 0;
    g_emergency_stop = 0;
}

/**
 * @brief 发送响应字符串
 */
static void BT_SendResponse(const char *str)
{
    Uart2_SendBuf((uint8_t*)str, strlen(str));
}

/**
 * @brief 处理急停命令（全局生效）
 */
void BT_EmergencyStop(void)
{
    g_emergency_stop = 1;
    g_motion_mode = MOTION_IDLE;
    g_motion_speed = 0;
    
    // 退出巡线模式和键控模式
    star_car = 0;
    g_bt_key_control_mode = 0;
    g_key_control_mode = 0;
    
    // 重置圆环状态机
    Circle_Reset();
    
    // 禁用轮子锁定
    WheelLock_Disable();
    
    // 立即停止所有电机
    Motor_StopAll();
    Motor_Disable();
    
    // 停止负压电机
    M3PWM_SetDutyCycle(0);
    M3PWM_Stop();
    g_vacuum_enabled = 0;
    
    // 设置LED为红色表示急停状态
    RGB_SetColor(RGB_COLOR_R);
    
    BT_SendResponse("OK:ESTOP\r\n");
}

/**
 * @brief 清除急停状态
 */
void BT_ClearEmergencyStop(void)
{
    g_emergency_stop = 0;
    
    // 关闭急停LED
    RGB_SetColor(RGB_COLOR_OFF);
    
    BT_SendResponse("OK:ESTOP_CLR\r\n");
}

/**
 * @brief 检查是否处于急停状态
 */
uint8_t BT_IsEmergencyStopped(void)
{
    return g_emergency_stop;
}

/**
 * @brief 负压电机控制（全局生效）
 */
void BT_VacuumControl(uint8_t enable)
{
    g_vacuum_enabled = (enable > 0) ? 1 : 0;
    
    if(g_vacuum_enabled)
    {
        // 确保有转速设置
        if(g_vacuum_speed == 0)
        {
            g_vacuum_speed = VACUUM_DEFAULT_SPEED;
        }
        uint16_t duty = (1000 * g_vacuum_speed) / 100;
        M3PWM_SetDutyCycle(duty);
        M3PWM_Start();
        BT_SendResponse("OK:VON\r\n");
    }
    else
    {
        M3PWM_SetDutyCycle(0);
        M3PWM_Stop();
        BT_SendResponse("OK:VOFF\r\n");
    }
}

/**
 * @brief 设置负压电机转速（全局生效）
 */
void BT_VacuumSetSpeed(uint8_t speed)
{
    if(speed > 100) speed = 100;
    g_vacuum_speed = speed;
    
    if(g_vacuum_enabled)
    {
        uint16_t duty = (1000 * speed) / 100;
        M3PWM_SetDutyCycle(duty);
    }
    
    char buf[32];
    sprintf(buf, "OK:VS=%d\r\n", speed);
    BT_SendResponse(buf);
}

/**
 * @brief 处理停止移动命令
 */
static void BT_HandleStop(void)
{
    g_motion_mode = MOTION_IDLE;
    g_motion_speed = 0;
    BT_SendResponse("OK:STOP\r\n");
}

/**
 * @brief 处理前进命令
 */
static void BT_HandleMoveForward(uint8_t speed)
{
    // 检查急停状态
    if(g_emergency_stop)
    {
        BT_SendResponse("ERR:ESTOP_ACTIVE\r\n");
        return;
    }
    
    // 检查键控模式（方向控制需要在键控模式下）
    if(!g_key_control_mode)
    {
        BT_SendResponse("ERR:NOT_KEY_MODE\r\n");
        return;
    }
    
    g_motion_mode = MOTION_FORWARD;
    g_motion_speed = (speed > 0) ? speed : g_base_speed;
    
    char buf[32];
    sprintf(buf, "OK:FWD=%d\r\n", g_motion_speed);
    BT_SendResponse(buf);
}

/**
 * @brief 处理后退命令
 */
static void BT_HandleMoveBackward(uint8_t speed)
{
    if(g_emergency_stop)
    {
        BT_SendResponse("ERR:ESTOP_ACTIVE\r\n");
        return;
    }
    
    if(!g_key_control_mode)
    {
        BT_SendResponse("ERR:NOT_KEY_MODE\r\n");
        return;
    }
    
    g_motion_mode = MOTION_BACKWARD;
    g_motion_speed = (speed > 0) ? speed : g_base_speed;
    
    char buf[32];
    sprintf(buf, "OK:BWD=%d\r\n", g_motion_speed);
    BT_SendResponse(buf);
}

/**
 * @brief 处理左转命令
 */
static void BT_HandleTurnLeft(uint8_t speed)
{
    if(g_emergency_stop)
    {
        BT_SendResponse("ERR:ESTOP_ACTIVE\r\n");
        return;
    }
    
    if(!g_key_control_mode)
    {
        BT_SendResponse("ERR:NOT_KEY_MODE\r\n");
        return;
    }
    
    g_motion_mode = MOTION_TURN_LEFT;
    g_motion_speed = (speed > 0) ? speed : g_base_speed;
    
    char buf[32];
    sprintf(buf, "OK:TL=%d\r\n", g_motion_speed);
    BT_SendResponse(buf);
}

/**
 * @brief 处理右转命令
 */
static void BT_HandleTurnRight(uint8_t speed)
{
    if(g_emergency_stop)
    {
        BT_SendResponse("ERR:ESTOP_ACTIVE\r\n");
        return;
    }
    
    if(!g_key_control_mode)
    {
        BT_SendResponse("ERR:NOT_KEY_MODE\r\n");
        return;
    }
    
    g_motion_mode = MOTION_TURN_RIGHT;
    g_motion_speed = (speed > 0) ? speed : g_base_speed;
    
    char buf[32];
    sprintf(buf, "OK:TR=%d\r\n", g_motion_speed);
    BT_SendResponse(buf);
}

/**
 * @brief 设置基础速度
 */
static void BT_HandleSetBaseSpeed(uint8_t speed)
{
    if(speed > 250) speed = 250;
    g_base_speed = speed;
    
    char buf[32];
    sprintf(buf, "OK:SPD=%d\r\n", speed);
    BT_SendResponse(buf);
}

/**
 * @brief 查询当前状态
 */
static void BT_HandleQueryStatus(void)
{
    char buffer[128];
    
    sprintf(buffer, "MODE:%s,SPD:%d,BASE:%d,VAC:%s,VS:%d,KEY:%s,ESTOP:%s\r\n",
            (g_motion_mode == MOTION_IDLE) ? "IDLE" :
            (g_motion_mode == MOTION_FORWARD) ? "FWD" :
            (g_motion_mode == MOTION_BACKWARD) ? "BWD" :
            (g_motion_mode == MOTION_TURN_LEFT) ? "TL" : "TR",
            g_motion_speed,
            g_base_speed,
            g_vacuum_enabled ? "ON" : "OFF",
            g_vacuum_speed,
            g_key_control_mode ? "ON" : "OFF",
            g_emergency_stop ? "YES" : "NO");
    
    BT_SendResponse(buffer);
}

/**
 * @brief 处理键控模式开关
 */
static void BT_HandleKeyControlMode(uint8_t enabled)
{
    g_key_control_mode = (enabled > 0) ? 1 : 0;
    
    if(g_key_control_mode)
    {
        // 进入键控模式，清除急停，停止当前运动
        g_emergency_stop = 0;
        g_motion_mode = MOTION_IDLE;
        g_motion_speed = 0;
        
        // 同步全局模式变量
        g_bt_key_control_mode = 1;
        star_car = 0;  // 退出巡线模式
        
        // 设置LED为蓝色表示键控模式
        RGB_SetColor(RGB_COLOR_B);
        
        BT_SendResponse("OK:KEY=ON\r\n");
    }
    else
    {
        // 退出键控模式，停止运动
        g_motion_mode = MOTION_IDLE;
        g_motion_speed = 0;
        
        // 同步全局模式变量
        g_bt_key_control_mode = 0;
        
        // 关闭LED
        RGB_SetColor(RGB_COLOR_OFF);
        
        BT_SendResponse("OK:KEY=OFF\r\n");
    }
}

/**
 * @brief 处理键控模式切换（硬件K2直接调用）
 */
void BT_HandleKeyControlModeChange(uint8_t enabled)
{
    BT_HandleKeyControlMode(enabled);
}

/**
 * @brief 解析并执行字符串命令
 * @param cmd: 命令字符串（不含换行符）
 */
static void BT_ParseCommand(const char *cmd)
{
    // 去掉可能的空格
    while(*cmd == ' ') cmd++;
    
    // 检查空命令
    if(strlen(cmd) == 0) return;
    
    // 解析参数（如果有的话）
    int param = 0;
    char cmd_part[16] = {0};
    const char *colon = strchr(cmd, ':');
    
    if(colon != NULL)
    {
        // 有参数的命令
        int cmd_len = colon - cmd;
        if(cmd_len > 15) cmd_len = 15;
        strncpy(cmd_part, cmd, cmd_len);
        cmd_part[cmd_len] = 0;
        param = atoi(colon + 1);
    }
    else
    {
        // 无参数的命令
        strncpy(cmd_part, cmd, 15);
    }
    
    // 转换为大写方便比较
    for(int i = 0; cmd_part[i]; i++)
    {
        if(cmd_part[i] >= 'a' && cmd_part[i] <= 'z')
        {
            cmd_part[i] -= 32;
        }
    }
    
    // ========== 全局命令（任何时候都生效） ==========
    
    // 急停命令 - 最高优先级
    if(strcmp(cmd_part, "ESTOP") == 0 || strcmp(cmd_part, "E") == 0)
    {
        BT_EmergencyStop();
        return;
    }
    
    // 清除急停
    if(strcmp(cmd_part, "CLR") == 0 || strcmp(cmd_part, "CLEAR") == 0)
    {
        BT_ClearEmergencyStop();
        return;
    }
    
    // 负压开
    if(strcmp(cmd_part, "VON") == 0)
    {
        BT_VacuumControl(1);
        return;
    }
    
    // 负压关
    if(strcmp(cmd_part, "VOFF") == 0)
    {
        BT_VacuumControl(0);
        return;
    }
    
    // 负压转速
    if(strcmp(cmd_part, "VS") == 0)
    {
        BT_VacuumSetSpeed((uint8_t)param);
        return;
    }
    
    // 基础速度设置
    if(strcmp(cmd_part, "SPD") == 0)
    {
        BT_HandleSetBaseSpeed((uint8_t)param);
        return;
    }

    // PID参数设置：PID:kp,ki,kd[,kf]
    if(strcmp(cmd_part, "PID") == 0)
    {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f, kf = 0.0f;
        int args = 0;
        if(colon)
        {
             args = sscanf(colon + 1, "%f,%f,%f,%f", &kp, &ki, &kd, &kf);
        }

        if(args == 3)
        {
            SpeedPID_SetParamBoth(kp, ki, kd);
            SpeedPID_ResetState();
            char buf[48];
            sprintf(buf, "OK:PID=%.2f,%.2f,%.2f\r\n", kp, ki, kd);
            BT_SendResponse(buf);
        }
        else if(args == 4)
        {
            SpeedPID_SetParamBoth(kp, ki, kd);
            SpeedPID_SetFeedForwardBoth(kf);
            SpeedPID_ResetState();
            char buf[64];
            sprintf(buf, "OK:PID=%.2f,%.2f,%.2f,KF=%.2f\r\n", kp, ki, kd, kf);
            BT_SendResponse(buf);
        }
        else
        {
            BT_SendResponse("ERR:PID_FMT\r\n");
        }
        return;
    }
    
    // 位置环PPID参数设置：PPID:kp,ki,kd,gyro_kd
    if(strcmp(cmd_part, "PPID") == 0)
    {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f, gyro_kd = 0.0f;
        int args = 0;
        if(colon)
        {
             args = sscanf(colon + 1, "%f,%f,%f,%f", &kp, &ki, &kd, &gyro_kd);
        }

        if(args == 4)
        {
            PositionPID_SetGlobalParams(kp, ki, kd, gyro_kd);
            char buf[64];
            sprintf(buf, "OK:PPID=%.1f,%.1f,%.1f,%.1f\r\n", kp, ki, kd, gyro_kd);
            BT_SendResponse(buf);
        }
        else
        {
            BT_SendResponse("ERR:PPID_FMT\r\n");
        }
        return;
    }
    
    // 单独设置 KF
    if(strcmp(cmd_part, "KF") == 0)
    {
        float kf = 0.0f;
        if(colon && (sscanf(colon + 1, "%f", &kf) == 1))
        {
            SpeedPID_SetFeedForwardBoth(kf);
            char buf[32];
            sprintf(buf, "OK:KF=%.2f\r\n", kf);
            BT_SendResponse(buf);
        }
        else
        {
             BT_SendResponse("ERR:KF_FMT\r\n");
        }
        return;
    }
    
    // 陀螺仪阻尼开关: GYRO:1=开启, GYRO:0=关闭（架空测试时禁用）
    if(strcmp(cmd_part, "GYRO") == 0)
    {
        PID_GyroDamping_Enable((uint8_t)param);
        char buf[32];
        sprintf(buf, "OK:GYRO=%s\r\n", PID_GyroDamping_IsEnabled() ? "ON" : "OFF");
        BT_SendResponse(buf);
        return;
    }
    
    // 模拟黑点测试命令: BLK:1,2,3 表示1,2,3路检测到黑线
    // BLK:OFF 关闭模拟模式
    if(strcmp(cmd_part, "BLK") == 0)
    {
        if(colon)
        {
            const char *params = colon + 1;
            
            // 检查是否关闭模拟模式
            if(strcmp(params, "OFF") == 0 || strcmp(params, "off") == 0 || strcmp(params, "0") == 0)
            {
                BlackPoint_Finder_SetSimulateMode(0);
                BT_SendResponse("OK:BLK=OFF\r\n");
            }
            else
            {
                // 解析通道号列表: 1,2,3 或 9,10
                uint8_t channels[16];
                uint8_t count = 0;
                char buf_parse[64];
                char *token;
                
                // 复制字符串以避免修改原始数据
                strncpy(buf_parse, params, sizeof(buf_parse) - 1);
                buf_parse[sizeof(buf_parse) - 1] = '\0';
                
                // 使用逗号分割
                token = strtok(buf_parse, ",");
                while(token != NULL && count < 16)
                {
                    int ch = atoi(token);
                    if(ch >= 0 && ch < 16)
                    {
                        channels[count++] = (uint8_t)ch;
                    }
                    token = strtok(NULL, ",");
                }
                
                if(count > 0)
                {
                    BlackPoint_Finder_SetSimulateChannels(channels, count);
                    BlackPoint_Finder_SetSimulateMode(1);
                    
                    char resp[64];
                    sprintf(resp, "OK:BLK=%d ch\r\n", count);
                    BT_SendResponse(resp);
                }
                else
                {
                    BT_SendResponse("ERR:BLK_FMT\r\n");
                }
            }
        }
        else
        {
            // 无参数，启用默认模拟模式（中间两路7,8）
            BlackPoint_Finder_SetSimulateMode(1);
            BT_SendResponse("OK:BLK=7,8(default)\r\n");
        }
        return;
    }
    
    // 状态查询
    if(strcmp(cmd_part, "?STATUS") == 0 || strcmp(cmd_part, "STATUS") == 0 || strcmp(cmd_part, "?") == 0)
    {
        BT_HandleQueryStatus();
        return;
    }
    
    // ADC查询: ?ADC - 返回16路光电管ADC值和黑线检测情况
    if(strcmp(cmd_part, "?ADC") == 0 || strcmp(cmd_part, "ADC") == 0)
    {
        uint8_t black_flags[SENSOR_COUNT];
        char buf[256];
        int i;
        
        // 获取黑线检测状态
        BlackPoint_Finder_GetBlackFlags_Dynamic(g_mux_adc_values, black_flags);
        
        // 发送ADC值 (分两行发送避免缓冲区溢出)
        sprintf(buf, "ADC[0-7]:");
        for(i = 0; i < 8; i++)
        {
            char tmp[16];
            sprintf(tmp, "%d%c", g_mux_adc_values[i], (i < 7) ? ',' : '\n');
            strcat(buf, tmp);
        }
        BT_SendResponse(buf);
        
        sprintf(buf, "ADC[8-15]:");
        for(i = 8; i < 16; i++)
        {
            char tmp[16];
            sprintf(tmp, "%d%c", g_mux_adc_values[i], (i < 15) ? ',' : '\n');
            strcat(buf, tmp);
        }
        BT_SendResponse(buf);
        
        // 发送黑线检测状态 (1=黑, 0=白)
        sprintf(buf, "BLK:");
        for(i = 0; i < 16; i++)
        {
            char tmp[4];
            sprintf(tmp, "%d", black_flags[i]);
            strcat(buf, tmp);
        }
        strcat(buf, "\r\n");
        BT_SendResponse(buf);
        return;
    }
    
    // PID运行状态查询: ?PID - 返回左右轮目标速度、实际速度、PWM输出
    if(strcmp(cmd_part, "?PID") == 0)
    {
        float tgt_l, tgt_r, pwm_l, pwm_r;
        int16_t act_l, act_r;
        char buf[128];
        
        PID_GetRunStatus(&tgt_l, &tgt_r, &act_l, &act_r, &pwm_l, &pwm_r);
        
        sprintf(buf, "TGT:L=%.1f,R=%.1f\r\n", tgt_l, tgt_r);
        BT_SendResponse(buf);
        sprintf(buf, "ACT:L=%d,R=%d\r\n", act_l, act_r);
        BT_SendResponse(buf);
        sprintf(buf, "PWM:L=%.0f,R=%.0f\r\n", pwm_l, pwm_r);
        BT_SendResponse(buf);
        return;
    }
    
    // 里程计状态查询: ?ODOM 或 ODOM - 返回位置、坐标、航向角
    if(strcmp(cmd_part, "?ODOM") == 0 || strcmp(cmd_part, "ODOM") == 0)
    {
        char buf[128];
        Odometer_Data_t odom;
        Odometer_GetData(&odom);
        
        sprintf(buf, "LOC:%.2f mm\r\n", odom.location);
        BT_SendResponse(buf);
        sprintf(buf, "POS:X=%.2f,Y=%.2f mm\r\n", odom.x, odom.y);
        BT_SendResponse(buf);
        sprintf(buf, "HDG:%.2f deg\r\n", odom.theta);
        BT_SendResponse(buf);
        sprintf(buf, "ENC:L=%ld,R=%ld\r\n", left_ecoder_cnt, right_ecoder_cnt);
        BT_SendResponse(buf);
        return;
    }
    
    // 里程计重置: ODOM_RST - 重置坐标和里程
    if(strcmp(cmd_part, "ODOM_RST") == 0)
    {
        Odometer_Reset();
        BT_SendResponse("OK:ODOM_RST\r\n");
        return;
    }
    
    // 里程计坐标重置(保留里程): ODOM_ZERO - 仅重置坐标，保留累计里程
    if(strcmp(cmd_part, "ODOM_ZERO") == 0)
    {
        Odometer_ResetCoordinate();
        BT_SendResponse("OK:ODOM_ZERO\r\n");
        return;
    }
    
    // 编码器校准开始: CALIB_START - 开始记录脉冲数
    if(strcmp(cmd_part, "CALIB_START") == 0 || strcmp(cmd_part, "CALIB") == 0)
    {
        Odometer_StartCalibration();
        RGB_SetColor(RGB_COLOR_YELLOW);
        BT_SendResponse("OK:CALIB_START\r\n");
        BT_SendResponse("Push car straight, then send CALIB_END:distance_mm\r\n");
        return;
    }
    
    // 编码器校准结束: CALIB_END:1000 - 传入实际推动的距离(mm)
    if(strcmp(cmd_part, "CALIB_END") == 0)
    {
        RGB_SetColor(RGB_COLOR_OFF);
        
        if(!Odometer_IsCalibrating())
        {
            BT_SendResponse("ERR:NOT_CALIB\r\n");
            return;
        }
        
        int32_t pulses = Odometer_GetCalibrationPulseCount();
        
        if(colon && param > 0)
        {
            // 用户指定了实际距离
            float new_pulse_per_mm = Odometer_EndCalibration((float)param);
            char buf[128];
            sprintf(buf, "CALIB:pulses=%ld,dist=%d mm\r\n", pulses, param);
            BT_SendResponse(buf);
            sprintf(buf, "NEW_PULSE_PER_MM=%.4f\r\n", new_pulse_per_mm);
            BT_SendResponse(buf);
            BT_SendResponse("Update ODOM_ENCODER_PULSE_PER_MM in Odometer.h\r\n");
        }
        else
        {
            // 未指定距离，仅显示脉冲数
            char buf[128];
            sprintf(buf, "CALIB:pulses=%ld\r\n", pulses);
            BT_SendResponse(buf);
            sprintf(buf, "If 500mm: %.4f pulse/mm\r\n", (float)pulses / 500.0f);
            BT_SendResponse(buf);
            sprintf(buf, "If 1000mm: %.4f pulse/mm\r\n", (float)pulses / 1000.0f);
            BT_SendResponse(buf);
            BT_SendResponse("Send CALIB_END:xxx (xxx=actual mm)\r\n");
        }
        return;
    }
    
    // 校准状态查询: ?CALIB - 查看当前校准脉冲数
    if(strcmp(cmd_part, "?CALIB") == 0)
    {
        if(Odometer_IsCalibrating())
        {
            int32_t pulses = Odometer_GetCalibrationPulseCount();
            char buf[64];
            sprintf(buf, "CALIB:pulses=%ld (active)\r\n", pulses);
            BT_SendResponse(buf);
        }
        else
        {
            BT_SendResponse("CALIB:inactive\r\n");
        }
        return;
    }
    
    // 巡线速度参数设置: ISPD:base,min[,omega_max]
    if(strcmp(cmd_part, "ISPD") == 0)
    {
        float base = 0.0f, spd_min = 0.0f, omega_max = 0.0f;
        int args = 0;
        
        if(colon)
        {
            args = sscanf(colon + 1, "%f,%f,%f", &base, &spd_min, &omega_max);
        }
        
        if(args >= 2)
        {
            PID_SetLineSpeedParams(base, spd_min);
            if(args >= 3)
                PID_SetLineSpeedRateK(omega_max);
            char buf[64];
            sprintf(buf, "OK:ISPD=%.0f,%.0f,W=%.1f\r\n", base, spd_min, PID_GetLineSpeedRateK());
            BT_SendResponse(buf);
        }
        else if(args == 0 || colon == NULL)
        {
            float cur_base, cur_min, gw, dw;
            PID_GetLineSpeedParams(&cur_base, &cur_min);
            PID_GetSpeedWeights(&gw, &dw);
            char buf[80];
            sprintf(buf, "ISPD:%.0f,%.0f,W=%.1f,GW=%.1f,DW=%.1f\r\n", 
                    cur_base, cur_min, PID_GetLineSpeedRateK(), gw, dw);
            BT_SendResponse(buf);
        }
        else
        {
            BT_SendResponse("ERR:ISPD_FMT (use ISPD:base,min[,omega_max])\r\n");
        }
        return;
    }
    
    // 减速权重设置: ISPDW:gyro_w,dev_w
    if(strcmp(cmd_part, "ISPDW") == 0)
    {
        float gw = 0.0f, dw = 0.0f;
        int args = 0;
        
        if(colon)
        {
            args = sscanf(colon + 1, "%f,%f", &gw, &dw);
        }
        
        if(args == 2)
        {
            PID_SetSpeedWeights(gw, dw);
            char buf[48];
            sprintf(buf, "OK:ISPDW=%.2f,%.2f\r\n", gw, dw);
            BT_SendResponse(buf);
        }
        else
        {
            PID_GetSpeedWeights(&gw, &dw);
            char buf[48];
            sprintf(buf, "ISPDW:%.2f,%.2f\r\n", gw, dw);
            BT_SendResponse(buf);
        }
        return;
    }
    
    // 实时数据打印开关（LIVE 命令，每次接收均切换状态）
    if(strcmp(cmd_part, "LIVE") == 0)
    {
        g_live_debug = !g_live_debug;
        if(g_live_debug)
        {
            g_live_debug_last_ms = g_systick_ms;
            BT_SendResponse("OK:LIVE=ON\r\n");
        }
        else
        {
            BT_SendResponse("OK:LIVE=OFF\r\n");
        }
        return;
    }
    
    // 光电管调试打印开关（SDBG 命令，每次接收均切换状态）
    // 打印格式: "000000011000000\r\n" (16位, 1=黑线, 0=白)
    if(strcmp(cmd_part, "SDBG") == 0)
    {
        g_sensor_debug = !g_sensor_debug;
        if(g_sensor_debug)
        {
            g_sensor_debug_last_ms = g_systick_ms;
            BT_SendResponse("OK:SDBG=ON\r\n");
        }
        else
        {
            BT_SendResponse("OK:SDBG=OFF\r\n");
        }
        return;
    }

    // 键控模式开关
    if(strcmp(cmd_part, "KEY") == 0)
    {
        BT_HandleKeyControlMode((uint8_t)param);
        return;
    }
    
    // ========== 键控模式命令（需要在键控模式下） ==========
    
    // 前进
    if(strcmp(cmd_part, "FWD") == 0 || strcmp(cmd_part, "W") == 0)
    {
        BT_HandleMoveForward((uint8_t)param);
        return;
    }
    
    // 后退
    if(strcmp(cmd_part, "BWD") == 0 || strcmp(cmd_part, "S") == 0)
    {
        BT_HandleMoveBackward((uint8_t)param);
        return;
    }
    
    // 左转
    if(strcmp(cmd_part, "TL") == 0 || strcmp(cmd_part, "A") == 0)
    {
        BT_HandleTurnLeft((uint8_t)param);
        return;
    }
    
    // 右转
    if(strcmp(cmd_part, "TR") == 0 || strcmp(cmd_part, "D") == 0)
    {
        BT_HandleTurnRight((uint8_t)param);
        return;
    }
    
    // 停止移动
    if(strcmp(cmd_part, "STOP") == 0 || strcmp(cmd_part, "X") == 0)
    {
        BT_HandleStop();
        return;
    }
    
    // 帮助命令
    if(strcmp(cmd_part, "HELP") == 0 || strcmp(cmd_part, "H") == 0)
    {
        BT_SendResponse("=== BT Commands ===\r\n");
        BT_SendResponse("Global: ESTOP,CLR,VON,VOFF,VS:xx,SPD:xx,?\r\n");
        BT_SendResponse("KeyCtrl: FWD,BWD,TL,TR,STOP (or W,S,A,D,X)\r\n");
        BT_SendResponse("KEY:1=Enter KeyMode, KEY:0=Exit\r\n");
        return;
    }
    
    // 未知命令
    BT_SendResponse("ERR:UNKNOWN_CMD\r\n");
}

/**
 * @brief 处理接收的数据（字符串解析）
 *        非阻塞方式，处理缓冲区内所有已收到的数据
 */
void BT_Process(void)
{
    uint8_t valid = 0;
    int available = Uart2_BytesAvailable();
    
    // 处理缓冲区中所有可用数据，不再限制32字节
    // 这避免了缓冲区来不及消费而溢出丢字节的问题
    while(available > 0)
    {
        uint8_t byte = Uart2_ReadByte(&valid);
        if(!valid)
        {
            break; // 没有数据了，退出
        }
        available--;
        
        // 检测换行符，表示命令结束
        if(byte == '\n' || byte == '\r')
        {
            if(g_cmd_overflow)
            {
                // 之前命令溢出，丢弃这条不完整命令，恢复正常
                g_cmd_overflow = 0;
                g_cmd_index = 0;
            }
            else if(g_cmd_index > 0)
            {
                g_cmd_buffer[g_cmd_index] = '\0';
                BT_ParseCommand(g_cmd_buffer);
                g_cmd_index = 0;
            }
        }
        else if(byte >= 32 && byte < 127)  // 可打印字符
        {
            if(g_cmd_index < BT_CMD_BUFFER_SIZE - 1)
            {
                g_cmd_buffer[g_cmd_index++] = (char)byte;
            }
            else
            {
                // 缓冲区满，标记溢出，等待当前命令的换行符后丢弃
                g_cmd_overflow = 1;
                g_cmd_index = 0;
            }
        }
        // 忽略其他控制字符
    }

    /* 实时调试数据周期打印 */
    if(g_live_debug)
    {
        uint32_t now = g_systick_ms;
        if((now - g_live_debug_last_ms) >= LIVE_DEBUG_INTERVAL_MS)
        {
            g_live_debug_last_ms = now;
            float pos_err = (float)position_get / 10.0f - 7.5f;
            float wt = BlackPoint_Finder_GetLastWeightSum();
            char buf[64];
            sprintf(buf, "LIVE:L=%d,R=%d,ERR=%.2f,WT=%.0f\r\n",
                    speed_left, speed_right, pos_err, wt);
            BT_SendResponse(buf);
        }
    }

    /* 光电管调试数据高频打印 */
    if(g_sensor_debug)
    {
        uint32_t now = g_systick_ms;
        if((now - g_sensor_debug_last_ms) >= SENSOR_DEBUG_INTERVAL_MS)
        {
            g_sensor_debug_last_ms = now;
            uint8_t black_flags[SENSOR_COUNT];
            char buf[24];  /* 16 chars + \r\n + \0 */
            uint8_t i;
            
            BlackPoint_Finder_GetBlackFlags_Dynamic(g_mux_adc_values, black_flags);
            
            for(i = 0; i < SENSOR_COUNT; i++)
            {
                buf[i] = black_flags[i] ? '1' : '0';
            }
            buf[SENSOR_COUNT] = '\r';
            buf[SENSOR_COUNT + 1] = '\n';
            buf[SENSOR_COUNT + 2] = '\0';
            BT_SendResponse(buf);
        }
    }
}

/**
 * @brief 获取当前运动状态
 */
Motion_Mode_t BT_GetMotionMode(void)
{
    return g_motion_mode;
}

/**
 * @brief 获取当前运动速度
 */
uint8_t BT_GetMotionSpeed(void)
{
    return g_motion_speed;
}

/**
 * @brief 获取完整的蓝牙状态
 */
const BT_State_t* BT_GetState(void)
{
    static BT_State_t state;
    state.motion_mode = g_motion_mode;
    state.motion_speed = g_motion_speed;
    state.base_speed = g_base_speed;
    state.vacuum_enabled = g_vacuum_enabled;
    state.vacuum_speed = g_vacuum_speed;
    state.key_control_mode = g_key_control_mode;
    state.emergency_stop = g_emergency_stop;
    return &state;
}

/**
 * @brief 直接设置吸盘转速（用于硬件K1同步状态）
 */
void BT_SetVacuumSpeedDirect(uint8_t speed_percent)
{
    if(speed_percent > 100) speed_percent = 100;
    g_vacuum_speed = speed_percent;
    g_vacuum_enabled = (speed_percent > 0) ? 1 : 0;
}

/**
 * @brief 获取当前设置的负压风扇转速百分比 (0-100)
 */
uint8_t BT_GetVacuumSpeed(void)
{
    return g_vacuum_speed;
}

/**
 * @brief 发送数据包（保留用于兼容）
 */
void BT_SendPacket(const BT_Packet_t *packet)
{
    // 保留接口，但现在使用字符串方式
    char buf[32];
    sprintf(buf, "PKT:%02X,%02X\r\n", packet->cmd, packet->data);
    BT_SendResponse(buf);
}
