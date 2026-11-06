/*
 * robot_params.h
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */

#ifndef INC_ROBOT_PARAMS_H_
#define INC_ROBOT_PARAMS_H_

#include "pid.h"

/* ====================================================================
 * CẤU HÌNH BẬT / TẮT MICRO-ROS
 * 0: TẮT micro-ROS (chạy test phần cứng độc lập: Hall, PWM, PID, PS2 pad)
 * 1: BẬT micro-ROS (khi đã build thư viện libmicroros)
 * ==================================================================== */
#define ENABLE_MICROROS            1

/* ====================================================================
 * THÔNG SỐ CẢM BIẾN HALL & ĐỘNG CƠ BLDC HOVERBOARD
 * (CẤU HÌNH CHO NGƯỜI DÙNG ĐIỀN THÔNG SỐ THỰC TẾ)
 * ==================================================================== */

/**
 * Số đôi cực từ (Pole pairs) của động cơ BLDC Hoverboard:
 * - Động cơ hoverboard phổ biến (bánh 6.5" - 10") thường có 15 đôi cực (30 nam châm vĩnh cửu).
 * - Người dùng điền thông số thực tế tại đây:
 */
#define MOTOR_LEFT_POLE_PAIRS      15      /* ĐIỀN THÔNG SỐ THỰC TẾ SỐ ĐÔI CỰC MOTOR TRÁI */
#define MOTOR_RIGHT_POLE_PAIRS     15      /* ĐIỀN THÔNG SỐ THỰC TẾ SỐ ĐÔI CỰC MOTOR PHẢI */

/**
 * Tỷ số truyền (Gear ratio):
 * - Động cơ Hoverboard gắn trực tiếp vào bánh xe (Direct-Drive Hub Motor): GEAR_RATIO = 1.0f.
 * - Nếu dùng động cơ có hộp số giảm tốc, điền tỷ số truyền thực tế tại đây:
 */
#define MOTOR_LEFT_GEAR_RATIO      1.0f    /* ĐIỀN TỶ SỐ TRUYỀN MOTOR TRÁI (Hub motor = 1.0) */
#define MOTOR_RIGHT_GEAR_RATIO     1.0f    /* ĐIỀN TỶ SỐ TRUYỀN MOTOR PHẢI (Hub motor = 1.0) */

/**
 * Chiều quay đếm Hall (+1 hoặc -1):
 * - Đảm bảo CurSpeed CÙNG DẤU với SpeedPIDOut và DesiredSpeed (Phản hồi âm ổn định):
 */
#define MOTOR_LEFT_DIRECTION      (+1)    /* Đã đảo lại +1 để CurSpeed2 cùng dấu với SpeedPIDOut2/DesiredSpeed2 */
#define MOTOR_RIGHT_DIRECTION     (-1)    /* Đã đảo lại -1 để CurSpeed1 cùng dấu với SpeedPIDOut1/DesiredSpeed1 */

/**
 * Số cạnh chuyển trạng thái Hall trên 1 vòng quay bánh xe:
 *   HALL_EDGES_PER_MOTOR_REV = 6 * POLE_PAIRS
 *   HALL_TICKS_PER_WHEEL_REV = 6 * POLE_PAIRS * GEAR_RATIO
 */
#define HALL_TICKS_PER_REV_L       ((float)(6 * MOTOR_LEFT_POLE_PAIRS) * MOTOR_LEFT_GEAR_RATIO)
#define HALL_TICKS_PER_REV_R       ((float)(6 * MOTOR_RIGHT_POLE_PAIRS) * MOTOR_RIGHT_GEAR_RATIO)

/**
 * Tương thích ngược với TICKS_PER_REV cũ (thay thế giá trị 28000 của encoder cũ):
 */
#define TICKS_PER_REV              ((HALL_TICKS_PER_REV_L + HALL_TICKS_PER_REV_R) * 0.5f)

/* ====================================================================
 * THÔNG SỐ CƠ KHÍ ROBOT (GIỮ NGUYÊN)
 * ==================================================================== */
#define WHEEL_DIAMETER_M   0.1651f     // Đường kính bánh xe (10 cm)
#define WHEEL_CIRC_M       (3.1415926f * WHEEL_DIAMETER_M)
#define WHEEL_BASE_M       0.292f    // Khoảng cách 2 bánh xe (m)

// Chu kỳ lấy mẫu cho các task (ms)
#define ODOMETRY_PERIOD_MS     5
#define MOTORCONTROL_PERIOD_MS 5
#define ENCODER_PERIOD_MS      5
#define UART_PERIOD_MS         50
#define DT_TIME                0.005f

// Giới hạn tốc độ và gia tốc
#define MAXSPEED           1.00f
#define V_MAX              1.00f     // Tốc độ dài cực đại (m/s)
#define V_STEP             0.50f     // Bước tốc độ D-pad

// Hệ số bù trước Feedforward vận tốc: cung cấp công suất nền cơ bản (PWM / (m/s))
// 250.0f cho ~125 PWM tại 0.5 m/s, êm dịu, không gây vọt lố hay sập nguồn
#define MOTOR_FEEDFORWARD_KV   250.0f

#define MAXACC             2.5f

// Hằng số bộ điều khiển PS2
#define W_MAX              2.00f     // Tốc độ góc cực đại (rad/s)
#define W_STEP             0.50f     // Bước tốc độ xoay D-pad

#define PS2_TIMEOUT_MS     500
#define STICK_DEADBAND     12        // Vùng chết cần analog (0..255)

// Ngưỡng kích hoạt Heading hold
#define EPS_TURN_MPS       0.03f   // if |vR_ref - vL_ref| < this => "straight command"
#define EPS_MOVE_MPS       0.05f   // if avg speed below this => don't bother holding heading

// Hệ số bộ điều khiển hướng (Heading controller gains)
#define HEADING_KP         2.0f
#define HEADING_KD         0.1f

// Chu kỳ vòng lặp hướng
#define HEADING_DT         0.02f   // 20 ms (50 Hz)
#define HEADING_DIV        4       // 4 ticks * 5ms = 20ms

// Giới hạn hiệu chỉnh góc (rad/s)
#define OMEGA_CORR_MAX     2.0f

// Biến điều khiển PID
extern PID_TypeDef PosPID1, PosPID2;
extern PID_TypeDef SpeedPID1, SpeedPID2;

extern double CurPos1, CurSpeed1, PosPIDOut1, SpeedPIDOut1, DesiredPos1, DesiredSpeed1;
extern double CurPos2, CurSpeed2, PosPIDOut2, SpeedPIDOut2, DesiredPos2, DesiredSpeed2;

extern volatile double SetPos1, SetPos2, SetSpeed1, SetSpeed2;
extern double Spd1_cmdfromROS, Spd2_cmdfromROS;

extern double Kp1, Ki1, Kd1;
extern double Kp2, Ki2, Kd2;

// Các biến lưu thông số PID từ giao diện (bảo toàn định nghĩa để giữ tương thích)
extern double Kp_fromLCD1, Ki_fromLCD1, Kd_fromLCD1;
extern double Kp_fromLCD2, Ki_fromLCD2, Kd_fromLCD2;
extern uint8_t pid_update_pending;

extern int32_t PID_sampletime;

// Các biến encoder / hall toàn cục
extern int32_t g_encCount1;
extern int32_t g_encCount2;
extern float g_speed1_mps;
extern float g_speed2_mps;
extern float g_revCount1;
extern float g_revCount2;

#endif /* INC_ROBOT_PARAMS_H_ */
