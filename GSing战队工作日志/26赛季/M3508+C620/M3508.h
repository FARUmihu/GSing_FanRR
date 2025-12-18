/*
 * M3508.h
 *
 *  Created on: Dec 10, 2025
 *      Author: FMI
 */

#ifndef INC_M3508_H_
#define INC_M3508_H_

#include "main.h"
#include "stm32h7xx_hal.h"

//配置参数
#define MOTOR_3508_number 8          //挂载在can1以及can2上所有电机的数量
#define MOTOR_HISTORY 20          // 选择记录存储电机反馈的次数：20次记录

//结构体鉴赏

//存所用大疆3508电机回传的数据仓库结构体，用id来进行分组
//写一个专门用于存储3508反馈值的结构体
typedef struct
{
	int number;                          //这个结构体的序号，一般与电机的id一一匹配，id为0，number就为0
    uint8_t motor_id;                    // 电机ID (0~3)因为是3508当轮毂电机嘛

    // 当前最新值（方便快速访问）
    int32_t now_Pos;                       // 当前位置 - 来自Data[0-1]的计算增量累加结果
    int16_t now_W;                         // 当前速度 (rad/min) - 来自Data[2-3]
    int16_t now_A;                         // 当前电流 (A) - 来自Data[4-5]
    uint8_t now_temp;          		   		// 温度 - 来自Data[6]
    int16_t round_cnt;                    // 圈数计数

    // 历史记录（环形缓冲区）
    float history_A[MOTOR_HISTORY];  // 力矩历史
    float history_W[MOTOR_HISTORY];  // 速度历史
    float history_Pos[MOTOR_HISTORY]; // 位置历史
    float history_Temp[MOTOR_HISTORY]; // 温度历史

    uint8_t write_index;                 // 下一次写入的位置索引 (0~19)
    uint8_t count;                       // 当前已存入的有效数据点数量（<=20）这个数量是取自于函数顶端的宏定义，可以自己更改
    uint32_t numb_updates;               // 总更新次数（用于调试或统计）
} Motor_3508_Instance;

//CAN发送或是接收的ID，一个枚举整型，注定了一条CAN最多挂载8个电机
typedef enum
{
    CAN_3508_ALL_ID     = 0x200,  // 3508电机控制ID
    CAN_3508_M1_ID      = 0x201,  // 3508电机1反馈ID
    CAN_3508_M2_ID      = 0x202,  // 3508电机2反馈ID
    CAN_3508_M3_ID      = 0x203,  // 3508电机3反馈ID
    CAN_3508_M4_ID      = 0x204,  // 3508电机4反馈ID
    CAN_3508_M5_ID      = 0x205,  // 3508电机5反馈ID
    CAN_3508_M6_ID      = 0x206,  // 3508电机6反馈ID
    CAN_3508_M7_ID      = 0x207,  // 3508电机7反馈ID
    CAN_3508_M8_ID      = 0x208,  // 3508电机8反馈ID
} can_msg_id;

//接收到的3508电机参数结构体
typedef struct
{
    /* 3508电机反馈数据（从CAN接收）*/
    uint16_t  ecd;                // 编码器值 [0,8191] - 来自Data[0-1]
    int16_t   speed_rpm;          // 转速 RPM - 来自Data[2-3]
    int16_t   real_current;       // 实际电流 - 来自Data[4-5]
    uint8_t   temperate;          // 温度 - 来自Data[6]

    /* 控制数据（发送用）*/
    int16_t   given_current;      // 给定电流（发送控制用，非反馈）

    /* 计算得到的数据 */
    uint16_t  last_ecd;           // 上次编码器值
    uint16_t  offset_ecd;         // 编码器偏移（校准用）
    int32_t   total_angle;        // 总角度（相对初始位置）
    int16_t   round_cnt;          // 圈数计数

    /* 统计计数 */
    uint32_t  msg_cnt;            // 消息计数
} motor_measure_t;

extern motor_measure_t motor_3508_can[8];  // CAN1上的3508电机

void my_can_filter_init_recv_all(void);
void get_moto_measure(motor_measure_t *ptr, uint8_t* data);
void set_moto_current_can1(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);//发送can1上控制四个电机电流的指令，无PID版本
void set_moto_current_can2(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);//发送can2上控制四个电机电流的指令，无PID版本

//用于布置回传存储的函数
void Motor_3508_Instance_Init(Motor_3508_Instance* inst, int number);
void Motor_3508_Instance_Update(Motor_3508_Instance* inst, int16_t now_A, int16_t W, int32_t Pos, uint8_t Temp);
#endif /* INC_M3508_H_ */
