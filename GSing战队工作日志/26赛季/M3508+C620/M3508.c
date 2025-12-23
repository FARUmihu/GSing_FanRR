/*
 * M3508.c
 *
 *  Created on: Dec 10, 2025
 *      Author: FMI
 */
#include "M3508.h"
#include "pid.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;


/* 全局变量定义 */
motor_measure_t motor_3508_can[8] = {0};//现在是can1，can2共用motor_3508_can[8]，不过彼此按标号分清，id=0，1，2，3是can1；id=4，5，6，7是can2；
Motor_3508_Instance motor_3508_instance[MOTOR_3508_number];//8个电机的回传数据中间存储结构体

/*pid相关参数变量*/
static PID_Controller pid_can[MOTOR_3508_number]; //MOTOR_3508_number 8个电机的 PID 控制器

/* 发送相关变量 */
static FDCAN_TxHeaderTypeDef CAN_Tx1Message;
static FDCAN_TxHeaderTypeDef CAN_Tx2Message;
static uint8_t TxData[8];
//一个简单的运算符，进行“？”前的判断，正确就输出“：”前面的值，错误就输出“：”后面的
#define ABS(x) ((x) > 0 ? (x) : -(x))

/*******************************************************************************************
  * @Func		my_can_filter_init_recv_all
  * @Brief    FDCAN1和FDCAN2滤波器配置（接收所有消息）
  *******************************************************************************************/
void my_can_filter_init_recv_all(void)
{
    FDCAN_FilterTypeDef filter;

    /* 配置过滤器：接收所有标准ID消息 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x0000;
    filter.FilterID2 = 0x0000;

    /* CAN1滤波器配置 */
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* CAN2滤波器配置 */
    filter.FilterIndex = 1;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* 启动CAN */
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);

    /* 激活接收中断 */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/*******************************************************************************************
  * @Func		HAL_FDCAN_RxFifo0Callback
  * @Brief    FDCAN接收回调函数
  *******************************************************************************************/

//我觉得肯定写在这里也是可以的，但是有待商榷。决定在此处加入一个全局可以访问的静态变量Motor_3508_Instance，专门存储回传数据，此处can1挂载4个那就是id，0~3
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        switch(rx_header.Identifier)
        {
            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID:
            case CAN_3508_M5_ID:
            case CAN_3508_M6_ID:
            case CAN_3508_M7_ID:
            case CAN_3508_M8_ID:
            {
                uint8_t i = rx_header.Identifier - CAN_3508_M1_ID;//这对吗？电机的标号到底是1~8还是0~7？好的，解决了，id是0~7

                /* 前50条消息用于校准，之后正常处理 */
                if (motor_3508_can[i].msg_cnt <= 50) {
                	if(motor_3508_can[i].msg_cnt == 50){
                		motor_3508_can[i].ecd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
                		motor_3508_can[i].offset_ecd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
                	}
                } else {
                    get_moto_measure(&motor_3508_can[i], rx_data);
                    Motor_3508_Instance_Update(&motor_3508_instance[i], motor_3508_can[i].real_current, motor_3508_can[i].speed_rpm, motor_3508_can[i].total_angle, motor_3508_can[i].temperate);
                }
                motor_3508_can[i].msg_cnt++;

                break;
            }

            default:
                break;
        }
    }

    /* 重新使能中断 */
    if (hfdcan == &hfdcan1) {
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    } else if (hfdcan == &hfdcan2) {
        HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}

/*******************************************************************************************
  * @Func		get_moto_measure
  * @Brief    解析3508电机反馈数据（修正版）
  *******************************************************************************************/
void get_moto_measure(motor_measure_t *ptr, uint8_t *data)
{

    /* 保存上次编码器值 */
    ptr->last_ecd = ptr->ecd;

    /* 修正：3508电机协议数据解析 */
    ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);        // 编码器值 (0-8191)
    ptr->speed_rpm = (int16_t)((data[2] << 8 | data[3]) < 10000 ? ((int16_t)(data[2] << 8 | data[3])) : (((int16_t)(data[2] << 8 | data[3])) - 65535));   // 转速 RPM
    ptr->real_current = (int16_t)(data[4] << 8 | data[5]);// 实际电流
    ptr->temperate = data[6];                           // 温度

    /* 智能角度计算（处理8192边界） */
    {
        int16_t res1, res2, delta;

        if (ptr->ecd < ptr->last_ecd) {
            res1 = ptr->ecd + 8192 - ptr->last_ecd;// 假设正转跨边界的变化值
            res2 = ptr->ecd - ptr->last_ecd;//假设未跨边界（实际是反转）的变化值
        } else {
            res1 = ptr->ecd - 8192 - ptr->last_ecd;// 假设反转转跨边界的变化值
            res2 = ptr->ecd - ptr->last_ecd;//假设未跨边界（实际是正转）的变化值
        }

        if (ABS(res1) < ABS(res2)) {
            delta = res1;
        } else {
            delta = res2;
        }

        ptr->total_angle += delta;
//        ptr->last_ecd = ptr->ecd;  // 更新上次编码器值；//这包有问题的吧
        ptr->round_cnt = ptr->total_angle / 8192;
    }
}

/*******************************************************************************************
  * @Func		set_moto_current_can1
  * @Brief    控制CAN1总线上的3508电机
  *******************************************************************************************/
void set_moto_current_can1(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    HAL_StatusTypeDef status;

    /* 只发一帧：控制4个电机 (ID: 0x200) */
    CAN_Tx1Message.Identifier = CAN_3508_CAN1_ID;
    CAN_Tx1Message.IdType = FDCAN_STANDARD_ID;
    CAN_Tx1Message.TxFrameType = FDCAN_DATA_FRAME;
    CAN_Tx1Message.DataLength = FDCAN_DLC_BYTES_8;

    TxData[0] = iq1 >> 8;
    TxData[1] = iq1;
    TxData[2] = iq2 >> 8;
    TxData[3] = iq2;
    TxData[4] = iq3 >> 8;
    TxData[5] = iq3;
    TxData[6] = iq4 >> 8;
    TxData[7] = iq4;
    HAL_Delay(1);

    status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_Tx1Message, TxData);
    if (status != HAL_OK) {
        // 可添加错误处理
    }
}

/*******************************************************************************************
  * @Func		set_moto_current_can2
  * @Brief    控制CAN2总线上的3508电机
  *******************************************************************************************/
void set_moto_current_can2(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    HAL_StatusTypeDef status;

    /* 只发一帧：控制4个电机 (识别符：0x1FF) */
    CAN_Tx2Message.Identifier = CAN_3508_CAN2_ID;
    CAN_Tx2Message.IdType = FDCAN_STANDARD_ID;
    CAN_Tx2Message.TxFrameType = FDCAN_DATA_FRAME;
    CAN_Tx2Message.DataLength = FDCAN_DLC_BYTES_8;

    TxData[0] = iq1 >> 8;
    TxData[1] = iq1;
    TxData[2] = iq2 >> 8;
    TxData[3] = iq2;
    TxData[4] = iq3 >> 8;
    TxData[5] = iq3;
    TxData[6] = iq4 >> 8;
    TxData[7] = iq4;

    status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &CAN_Tx2Message, TxData);
    if (status != HAL_OK) {
        // 可添加错误处理
    }
}

//在一切的校验后，将会进行记录的留存，这样就需要对应的结构体MotorInstance。接下来出场的是初始化和存数据函数
// 初始化一个电机实例
void Motor_3508_Instance_Init(Motor_3508_Instance* inst, int number) {
    inst->number = number;
	inst->motor_id = (uint8_t)number;

    inst->now_A = 0.0f;
    inst->now_W = 0.0f;
    inst->now_Pos = 0.0f;
    inst->write_index = 0;
    inst->count = 0;
    inst->numb_updates = 0;

    // 可选：清零历史数组（通常不是必须，但更干净）
    for (int i = 0; i < MOTOR_HISTORY; i++) {
        inst->history_A[i] = 0.0f;
        inst->history_W[i] = 0.0f;
        inst->history_Pos[i] = 0.0f;
        inst->history_Temp[i] = 0.0f;
    }
}


// 更新最新数据，并存入历史环形缓冲区
void Motor_3508_Instance_Update(Motor_3508_Instance* inst, int16_t A, int16_t W, int32_t Pos, uint8_t Temp) {
    // 更新当前值
    inst->now_A = A;
    inst->now_W = W;
    inst->now_Pos = Pos;
    inst->now_temp = Temp;

    //计算圈数
    inst->round_cnt = inst->now_Pos / 8192;

    // 写入历史
    inst->history_A[inst->write_index] = A;
    inst->history_W[inst->write_index] = W;
    inst->history_Pos[inst->write_index] = Pos;
    inst->history_Temp[inst->write_index] = Temp;

    // 更新索引和计数
    inst->write_index = (inst->write_index + 1) % MOTOR_HISTORY;
    if (inst->count < MOTOR_HISTORY) {
        inst->count++;
    }
}

//好了，开始进行PID控速了，为了方便且统一，这里用到PID控速是使用了许堃写的PID.c/.h
//初始化函数
void PID_M3508_CAN_Init(void) {

    // 保守安全参数（适用于大多数轮腿机器人）
    float Kp = 10.0f;   //
    float Ki = 0.3f;    //
    float Kd = 0.0f;    // 保持为 0

    // 输出限幅
    float output_max = 12000.0f;  // 初始限制电流，防过冲
    float output_min = -12000.0f;
    float integral_max = 500.0f;
    float integral_min = -500.0f;

    for (int i = 0; i < 8; i++) {
        PID_Init(&pid_can[i], Kp, Ki, Kd, output_max, output_min, integral_max, integral_min);
    }
}

//速度环can1
void PID_M3508_CAN1(int16_t target_rpm1, int16_t target_rpm2, int16_t target_rpm3, int16_t target_rpm4) {
//    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 获取实际转速（来自 CAN 反馈）
    int16_t actual_rpm1 = motor_3508_can[0].speed_rpm;
    int16_t actual_rpm2 = motor_3508_can[1].speed_rpm;
    int16_t actual_rpm3 = motor_3508_can[2].speed_rpm;
    int16_t actual_rpm4 = motor_3508_can[3].speed_rpm;

    // 使用 PID 计算应输出的电流指令（单位：int16_t）
    float iq1 = PID_Update(&pid_can[0], (float)(target_rpm1 * 3599 / 187), (float)actual_rpm1, current_time);
    float iq2 = PID_Update(&pid_can[1], (float)(target_rpm2 * 3599 / 187), (float)actual_rpm2, current_time);
    float iq3 = PID_Update(&pid_can[2], (float)(target_rpm3 * 3599 / 187), (float)actual_rpm3, current_time);
    float iq4 = PID_Update(&pid_can[3], (float)(target_rpm4 * 3599 / 187), (float)actual_rpm4, current_time);

    // 限幅（确保在 ±16384 范围内）
    iq1 = fmaxf(fminf(iq1, 16384.0f), -16384.0f);
    iq2 = fmaxf(fminf(iq2, 16384.0f), -16384.0f);
    iq3 = fmaxf(fminf(iq3, 16384.0f), -16384.0f);
    iq4 = fmaxf(fminf(iq4, 16384.0f), -16384.0f);

    // 转换为 int16_t 并发送到 CAN
    set_moto_current_can1((int16_t)iq1, (int16_t)iq2, (int16_t)iq3, (int16_t)iq4);
}

//速度环can2
void PID_M3508_CAN2(int16_t target_rpm1, int16_t target_rpm2, int16_t target_rpm3, int16_t target_rpm4) {
//    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 获取实际转速（来自 CAN 反馈）
    int16_t actual_rpm1 = motor_3508_can[0].speed_rpm;
    int16_t actual_rpm2 = motor_3508_can[1].speed_rpm;
    int16_t actual_rpm3 = motor_3508_can[2].speed_rpm;
    int16_t actual_rpm4 = motor_3508_can[3].speed_rpm;

    // 使用 PID 计算应输出的电流指令（单位：int16_t）
    float iq1 = PID_Update(&pid_can[4], (float)(target_rpm1 * 3599 / 187), (float)actual_rpm1, current_time);
    float iq2 = PID_Update(&pid_can[5], (float)(target_rpm2 * 3599 / 187), (float)actual_rpm2, current_time);
    float iq3 = PID_Update(&pid_can[6], (float)(target_rpm3 * 3599 / 187), (float)actual_rpm3, current_time);
    float iq4 = PID_Update(&pid_can[7], (float)(target_rpm4 * 3599 / 187), (float)actual_rpm4, current_time);

    // 限幅（确保在 ±16384 范围内）
    iq1 = fmaxf(fminf(iq1, 16384.0f), -16384.0f);
    iq2 = fmaxf(fminf(iq2, 16384.0f), -16384.0f);
    iq3 = fmaxf(fminf(iq3, 16384.0f), -16384.0f);
    iq4 = fmaxf(fminf(iq4, 16384.0f), -16384.0f);

    // 转换为 int16_t 并发送到 CAN
    set_moto_current_can2((int16_t)iq1, (int16_t)iq2, (int16_t)iq3, (int16_t)iq4);
}
