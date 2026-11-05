


/*
 * hall_sensor.h
 *
 *  Created on: 2026
 *      Author: Zackon
 *
 * Module đọc cảm biến Hall cho 2 động cơ BLDC Hoverboard.
 * Thay thế encoder quadrature bằng cảm biến Hall 3 pha.
 *
 * Phân bổ GPIO (Theo bảng ưu tiên Mục II.3):
 *   Motor Trái (Left)  : HA_L = PC6 (EXTI6),  HB_L = PC7 (EXTI7),  HC_L = PC8 (EXTI8)
 *   Motor Phải (Right) : HA_R = PD12 (EXTI12), HB_R = PD13 (EXTI13), HC_R = PD14 (EXTI14)
 *
 * Lưu ý phần cứng:
 *   - Tất cả 6 chân trên đều là chân FT (5V tolerant) trên STM32F767ZITx.
 *   - Cảm biến Hall hoverboard thường là Open-Collector hoặc 5V push-pull:
 *     + Nếu Open-Collector: dùng Pull-up nội GPIO_PULLUP (kéo lên 3.3V) an toàn 100%.
 *     + Nếu tín hiệu 5V chủ động: chân FT chịu được 5V khi VDD=3.3V, nhưng khuyến nghị
 *       dùng cầu phân áp (1k/2k) hoặc IC chuyển mức để bảo vệ tối ưu.
 */

#ifndef INC_HALL_SENSOR_H_
#define INC_HALL_SENSOR_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================
 * ID MOTOR
 * Tương ứng: HALL_MOTOR_LEFT = Motor 2 (Trái)
 *            HALL_MOTOR_RIGHT = Motor 1 (Phải)
 * ========================================================= */
#define HALL_MOTOR_LEFT         0   /* Motor trái */
#define HALL_MOTOR_RIGHT        1   /* Motor phải */

typedef enum {
    HALL_PHASE_A = 0,
    HALL_PHASE_B,
    HALL_PHASE_C
} HallPhase;

/* Live debugger counters: [0]=left/Motor 2, [1]=right/Motor 1; [A,B,C]. */
extern volatile uint32_t g_hall_channel_edges[2][3];

/* =========================================================
 * TIMEOUT TỐC ĐỘ (ms)
 * Nếu không có cạnh Hall trong thời gian này => Động cơ coi như đã dừng (tốc độ = 0)
 * ========================================================= */
#define HALL_SPEED_TIMEOUT_MS   150U

/* =========================================================
 * ĐỊNH NGHĨA CHÂN GPIO CHO 6 TÍN HIỆU HALL
 * ========================================================= */

/* Motor Trái (Left Motor): PC6, PC7, PC8 */
#define HALL_HA_L_PIN           GPIO_PIN_6
#define HALL_HA_L_PORT          GPIOC
#define HALL_HB_L_PIN           GPIO_PIN_7
#define HALL_HB_L_PORT          GPIOC
#define HALL_HC_L_PIN           GPIO_PIN_8
#define HALL_HC_L_PORT          GPIOC

/* Motor Phải (Right Motor): PD12, PD13, PD14 */
#define HALL_HA_R_PIN           GPIO_PIN_12
#define HALL_HA_R_PORT          GPIOD
#define HALL_HB_R_PIN           GPIO_PIN_13
#define HALL_HB_R_PORT          GPIOD
#define HALL_HC_R_PIN           GPIO_PIN_14
#define HALL_HC_R_PORT          GPIOD

/* Bitmask các chân phục vụ lọc EXTI */
#define HALL_LEFT_PINS_MASK     (HALL_HA_L_PIN | HALL_HB_L_PIN | HALL_HC_L_PIN)
#define HALL_RIGHT_PINS_MASK    (HALL_HA_R_PIN | HALL_HB_R_PIN | HALL_HC_R_PIN)

/* =========================================================
 * GIAO TIẾP HÀM (API FUNCTIONS)
 * ========================================================= */

/**
 * @brief  Khởi tạo GPIO và NVIC cho 6 chân Hall của 2 motor.
 */
void Hall_Init(void);

/**
 * @brief  Đọc số xung Hall tích lũy (signed) của motor.
 * @param  id: HALL_MOTOR_LEFT hoặc HALL_MOTOR_RIGHT
 * @return Số xung tích lũy (count)
 */
int32_t Hall_GetCount(int id);

/**
 * @brief  Đặt lại bộ đếm xung Hall về 0.
 * @param  id: HALL_MOTOR_LEFT hoặc HALL_MOTOR_RIGHT
 */
void Hall_Reset(int id);

/**
 * @brief  Callback xử lý ngắt EXTI cho các chân Hall (gọi từ HAL_GPIO_EXTI_Callback).
 *         Chỉ thực hiện đọc trạng thái, tra bảng, cập nhật bộ đếm và timestamp.
 * @param  GPIO_Pin: Chân GPIO gây ra ngắt
 */
void Hall_EXTI_Callback(uint16_t GPIO_Pin);

/**
 * @brief  Tính tốc độ dài tức thời (m/s) của bánh xe.
 *         Có bộ lọc thông thấp chống nhảy số ở tốc độ chậm và timeout khi dừng.
 * @param  id: HALL_MOTOR_LEFT hoặc HALL_MOTOR_RIGHT
 * @param  dt: Chu kỳ lấy mẫu (giây)
 * @return Vận tốc m/s (dấu dương = tiến, dấu âm = lùi)
 */
float Hall_GetSpeed_mps(int id, float dt);

/**
 * @brief  Tính quãng đường đi được từ lần reset cuối (mét).
 * @param  id: HALL_MOTOR_LEFT hoặc HALL_MOTOR_RIGHT
 * @return Quãng đường (m)
 */
float Hall_GetPosition_m(int id);

/**
 * @brief  Lấy số lỗi chuyển trạng thái bất thường (phục vụ chẩn đoán nhiễu).
 * @param  id: HALL_MOTOR_LEFT hoặc HALL_MOTOR_RIGHT
 * @return Số lần phát hiện chuyển trạng thái không hợp lệ
 */
uint32_t Hall_GetErrorCount(int id);

/** Đếm cạnh lên + cạnh xuống riêng từng kênh Hall kể từ Hall_Reset(). */
uint32_t Hall_GetChannelEdgeCount(int id, HallPhase phase);

/** Đọc mức logic hiện tại (0/1) của một kênh Hall. */
uint8_t Hall_ReadChannel(int id, HallPhase phase);

#ifdef __cplusplus
}
#endif

#endif /* INC_HALL_SENSOR_H_ */
