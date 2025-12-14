/*
 * M3508.c
 *
 *  Created on: Dec 10, 2025
 *      Author: FMI
 */
#include "M3508.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* 全局变量定义 */
motor_measure_t motor_3508_can1[8] = {0};
motor_measure_t motor_3508_can2[8] = {0};

static Motor_3508_Instance motor_3508_instance[MOTOR_3508_number];
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
                motor_measure_t  *motor_array;

                /* 确定CAN总线 */
                if (hfdcan == &hfdcan1) {
                    motor_array = motor_3508_can1;
                } else {
                    motor_array = motor_3508_can2;
                }

                /* 前50条消息用于校准，之后正常处理 */
                if (motor_array[i].msg_cnt <= 50) {
                	//一个替代函数版
                	if(motor_array[i].msg_cnt == 50){
                		motor_array[i].ecd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
                		motor_array[i].offset_ecd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
                	}
//                    get_moto_offset(&motor_array[i], (uint16_t)(rx_data[0] << 8 | rx_data[1]));//用了get_moto_offset函数的情况，可以注释掉测试一下能否去掉
                } else {
                    get_moto_measure(&motor_array[i], rx_data);
                    Motor_3508_Instance_Update(&motor_3508_instance[i], motor_array[i].real_current, motor_array[i].speed_rpm, motor_array[i].total_angle, motor_array[i].temperate);
                }
                motor_array[i].msg_cnt++;

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
	//真的废物啊，这点滤波数据一点儿没派上用场，先注释掉，测试成功还没用上就给删了
//    uint32_t i, sum = 0;
//    uint16_t current_ecd;

//    /* 编码器值滤波处理 */
//    current_ecd = (uint16_t)(data[0] << 8 | data[1]);
//    ptr->ecd_buf[ptr->buf_idx] = current_ecd;
//    ptr->buf_idx = (ptr->buf_idx + 1) % FILTER_BUF_LEN;
//
//    /* 计算滤波后的编码器值 */
//    for (i = 0; i < FILTER_BUF_LEN; i++) {
//        sum += ptr->ecd_buf[i];
//    }
//    ptr->filtered_ecd = sum / FILTER_BUF_LEN;

    /* 保存上次编码器值 */
    ptr->last_ecd = ptr->ecd;

    /* 修正：3508电机协议数据解析 */
    ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);        // 编码器值 (0-8191)
    ptr->speed_rpm = (int16_t)(data[2] << 8 | data[3]);   // 转速 RPM
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

///*******************************************************************************************
//  * @Func		get_moto_offset
//  * @Brief    电机偏移校准
//  *******************************************************************************************/
//void get_moto_offset(motor_measure_t *ptr, uint16_t current_ecd)
//{
//    ptr->ecd = current_ecd;
//    ptr->offset_ecd = current_ecd;
//    ptr->total_angle = 0;
//    ptr->round_cnt = 0;
//}

/*******************************************************************************************
  * @Func		set_moto_current_can1
  * @Brief    控制CAN1总线上的3508电机
  *******************************************************************************************/
void set_moto_current_can1(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    HAL_StatusTypeDef status;

    /* 只发一帧：控制4个电机 (ID: 0x200) */
    CAN_Tx1Message.Identifier = CAN_3508_ALL_ID;
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

    /* 只发一帧：控制4个电机 (ID: 0x200) */
    CAN_Tx2Message.Identifier = CAN_3508_ALL_ID;
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

/*******************************************************************************************
  * @Func		get_moto_3508_ptr
  * @Brief    获取指定电机指针
  *******************************************************************************************/
motor_measure_t* get_moto_3508_ptr(uint8_t can_bus, uint8_t motor_id)
{
    if (motor_id >= 8) {
        motor_id = 0;
    }

    if (can_bus == 0) {
        return &motor_3508_can1[motor_id];
    } else {
        return &motor_3508_can2[motor_id];
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
        inst->history_T[i] = 0.0f;
        inst->history_W[i] = 0.0f;
        inst->history_Pos[i] = 0.0f;
    }
}


// 更新最新数据，并存入历史环形缓冲区
void Motor_3508_Instance_Update(Motor_3508_Instance* inst, uint16_t A, uint16_t W, uint32_t Pos, uint8_t Temp) {
    // 更新当前值
    inst->now_A = A;
    inst->now_W = W;
    inst->now_Pos = Pos;
    inst->now_temp = Temp;

    //计算圈数
    inst->round_cnt = inst->now_Pos / 8192;

    // 写入历史
    inst->history_T[inst->write_index] = A;
    inst->history_W[inst->write_index] = W;
    inst->history_Pos[inst->write_index] = Pos;
    inst->history_Temp[inst->write_index] = Temp;

    // 更新索引和计数
    inst->write_index = (inst->write_index + 1) % MOTOR_HISTORY;
    if (inst->count < MOTOR_HISTORY) {
        inst->count++;
    }
}
