/*
 * encoder_speed.c
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 *
 * Tầng thích ứng: Chuyển hướng tính vị trí và vận tốc từ Encoder sang module Hall Sensor.
 */

#include "encoder_speed.h"
#include "hall_sensor.h"
#include "robot_params.h"

void EncoderSpeed_Init(void)
{
    /* Đã được khởi tạo trong Hall_Init() / Hall_Reset() */
}

// --------------------------------------
//  Quãng đường tính từ lúc Reset (m)
// --------------------------------------
float Encoder_GetPosition_m(EncoderId id)
{
    if (id == ENCODER_1) {
        return Hall_GetPosition_m(HALL_MOTOR_RIGHT);
    } else {
        return Hall_GetPosition_m(HALL_MOTOR_LEFT);
    }
}

// --------------------------------------
//   Tốc độ m/s (gọi mỗi vòng lặp)
// --------------------------------------
float Encoder_GetSpeed_mps(EncoderId id, float dt)
{
    if (id == ENCODER_1) {
        return Hall_GetSpeed_mps(HALL_MOTOR_RIGHT, dt);
    } else {
        return Hall_GetSpeed_mps(HALL_MOTOR_LEFT, dt);
    }
}
