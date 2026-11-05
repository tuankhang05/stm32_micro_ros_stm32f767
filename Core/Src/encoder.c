/*
 * encoder.c
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 *
 * Tầng thích ứng: Chuyển hướng các hàm Encoder cũ sang module Hall Sensor.
 * Không khởi động TIM3 / TIM4 encoder hardware để giải phóng 4 chân GPIO cho EXTI.
 */

#include "encoder.h"
#include "hall_sensor.h"

void Encoder_Init(void)
{
    /* Khởi tạo module Hall Sensor thay cho Timer Encoder */
    Hall_Init();
}

int32_t Encoder_GetCount(EncoderId id)
{
    /* ENCODER_1 = Motor Phải (Right), ENCODER_2 = Motor Trái (Left) */
    if (id == ENCODER_1) {
        return Hall_GetCount(HALL_MOTOR_RIGHT);
    } else {
        return Hall_GetCount(HALL_MOTOR_LEFT);
    }
}

void Encoder_Reset(EncoderId id)
{
    if (id == ENCODER_1) {
        Hall_Reset(HALL_MOTOR_RIGHT);
    } else {
        Hall_Reset(HALL_MOTOR_LEFT);
    }
}
