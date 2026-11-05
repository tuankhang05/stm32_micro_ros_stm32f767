                                      /*
 * hall_sensor.c
 *
 *  Created on: 2026
 *      Author: Zackon
 *
 * Module đọc cảm biến Hall cho 2 động cơ BLDC Hoverboard.
 * Thay thế encoder quadrature bằng hệ thống đọc Hall 3 pha.
 */

#include "hall_sensor.h"
#include "robot_params.h"
#include <math.h>

/* =========================================================
 * BẢNG CHUYỂN TIẾP TRẠNG THÁI HALL BLDC 120° CHUẨN
 *
 * Mã hóa trạng thái 3 bit: State = (HC << 2) | (HB << 1) | HA
 * Các trạng thái hợp lệ: 1, 2, 3, 4, 5, 6
 * Trạng thái không hợp lệ: 0 (000b) và 7 (111b)
 *
 * Chuỗi tiến (Forward): 5 -> 4 -> 6 -> 2 -> 3 -> 1 -> 5
 * Chuỗi lùi  (Reverse): 5 -> 1 -> 3 -> 2 -> 6 -> 4 -> 5
 *
 * Bảng: hall_transition_table[from_state][to_state]
 *   +1 : Quay tiến (Forward)
 *   -1 : Quay lùi  (Reverse)
 *    0 : Trạng thái không đổi hoặc chuyển trạng thái không hợp lệ
 * ========================================================= */
static const int8_t hall_transition_table[8][8] = {
    /* To:   0   1   2   3   4   5   6   7 */
    /* 0 */ { 0,  0,  0,  0,  0,  0,  0,  0 }, /* 000: Không hợp lệ */
    /* 1 */ { 0,  0,  0, -1,  0, +1,  0,  0 }, /* 1->5 (+1), 1->3 (-1) */
    /* 2 */ { 0,  0,  0, +1,  0,  0, -1,  0 }, /* 2->3 (+1), 2->6 (-1) */
    /* 3 */ { 0, +1, -1,  0,  0,  0,  0,  0 }, /* 3->1 (+1), 3->2 (-1) */
    /* 4 */ { 0,  0,  0,  0,  0, -1, +1,  0 }, /* 4->6 (+1), 4->5 (-1) */
    /* 5 */ { 0, -1,  0,  0, +1,  0,  0,  0 }, /* 5->4 (+1), 5->1 (-1) */
    /* 6 */ { 0,  0, +1,  0, -1,  0,  0,  0 }, /* 6->2 (+1), 6->4 (-1) */
    /* 7 */ { 0,  0,  0,  0,  0,  0,  0,  0 }  /* 111: Không hợp lệ */
};

/* =========================================================
 * CÁC BIẾN NỘI BỘ (VOLATILE VÌ DÙNG TRONG ISR & TASK)
 * ========================================================= */

/* Bộ đếm xung tích lũy (tick counter) */
static volatile int32_t  g_hall_count[2] = {0, 0};

/* Trạng thái Hall trước đó */
static volatile uint8_t  g_hall_prev_state[2] = {0, 0};

/* Timestamp DWT chu kỳ lần có cạnh Hall gần nhất */
static volatile uint32_t g_hall_last_edge_dwt[2] = {0, 0};

/* Khoảng thời gian (CPU cycles) giữa 2 cạnh Hall gần nhất */
static volatile uint32_t g_hall_edge_period_cycles[2] = {0, 0};

/* Chiều quay tức thời ghi nhận ở cạnh gần nhất (+1 hoặc -1) */
static volatile int8_t   g_hall_last_dir[2] = {0, 0};

/* Bộ đếm số lỗi chuyển trạng thái bất thường */
static volatile uint32_t g_hall_err_count[2] = {0, 0};

/* Cạnh lên + cạnh xuống riêng cho HA/HB/HC, dùng kiểm tra từng kênh. */
volatile uint32_t g_hall_channel_edges[2][3] = {{0, 0, 0}, {0, 0, 0}};

/* Bộ đếm xung tại lần gọi Hall_GetSpeed_mps trước */
static int32_t           g_hall_prev_count[2] = {0, 0};

/* Vận tốc lọc thông thấp (m/s) */
static float             g_hall_speed_filtered[2] = {0.0f, 0.0f};

/* Cửa sổ trượt lưu chu kỳ các cạnh Hall gần nhất (6 cạnh = 1 chu kỳ điện 360 độ) */
#define HALL_SPEED_WINDOW_SIZE  6U
static volatile uint32_t g_hall_edge_window[2][HALL_SPEED_WINDOW_SIZE];
static volatile uint8_t  g_hall_win_head[2] = {0, 0};
static volatile uint8_t  g_hall_win_count[2] = {0, 0};

/* =========================================================
 * HÀM NỘI BỘ (HELPER FUNCTIONS)
 * ========================================================= */

/**
 * @brief Bật bộ đếm DWT CYCCNT để lấy timestamp độ phân giải cao
 */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_LAR_KEY) || defined(DWT_LAR)
    DWT->LAR = 0xC5ACCE55;
#else
    *(volatile uint32_t *)0xE0001FB0 = 0xC5ACCE55; // Mở khóa thanh ghi DWT trên Cortex-M7 (STM32F767)
#endif
    DWT->CYCCNT = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Đọc trạng thái 3 chân Hall motor trái (PC6, PC7, PC8)
 */
static inline uint8_t Hall_ReadState_Left(void)
{
    uint32_t idr_c = GPIOC->IDR;

    uint8_t ha = (idr_c & HALL_HA_L_PIN) ? 1U : 0U; /* PC6 */
    uint8_t hb = (idr_c & HALL_HB_L_PIN) ? 1U : 0U; /* PC7 */
    uint8_t hc = (idr_c & HALL_HC_L_PIN) ? 1U : 0U; /* PC8 */

    return (uint8_t)((hc << 2U) | (hb << 1U) | ha);
}

/**
 * @brief Đọc trạng thái 3 chân Hall motor phải (PD12, PD13, PD14)
 */
static inline uint8_t Hall_ReadState_Right(void)
{
    uint32_t idr_d = GPIOD->IDR;

    uint8_t ha = (idr_d & HALL_HA_R_PIN) ? 1U : 0U; /* PD12 */
    uint8_t hb = (idr_d & HALL_HB_R_PIN) ? 1U : 0U; /* PD13 */
    uint8_t hc = (idr_d & HALL_HC_R_PIN) ? 1U : 0U; /* PD14 */

    return (uint8_t)((hc << 2U) | (hb << 1U) | ha);
}

/**
 * @brief Xử lý cập nhật Hall trong ISR (cực nhanh, không float, không block)
 */
static inline void Hall_Process_ISR(int id, uint8_t curr_state)
{
    uint8_t prev = g_hall_prev_state[id];

    /* Bỏ qua nếu trạng thái không đổi (nhiễu rung cạnh) hoặc không hợp lệ */
    if (curr_state == prev) {
        return;
    }
    if (curr_state == 0U || curr_state == 7U) {
        g_hall_err_count[id]++;
        return;
    }

    int8_t step = hall_transition_table[prev][curr_state];
    if (step != 0) {
        uint32_t now = DWT->CYCCNT;
        uint32_t dt = now - g_hall_last_edge_dwt[id];
        g_hall_last_edge_dwt[id] = now;
        g_hall_edge_period_cycles[id] = dt;
        g_hall_last_dir[id] = step;

        /* Cập nhật cửa sổ trượt chu kỳ cạnh Hall */
        g_hall_edge_window[id][g_hall_win_head[id]] = dt;
        g_hall_win_head[id] = (uint8_t)((g_hall_win_head[id] + 1U) % HALL_SPEED_WINDOW_SIZE);
        if (g_hall_win_count[id] < HALL_SPEED_WINDOW_SIZE) {
            g_hall_win_count[id]++;
        }

        int8_t dir_mult = (id == HALL_MOTOR_LEFT) ? (int8_t)MOTOR_LEFT_DIRECTION : (int8_t)MOTOR_RIGHT_DIRECTION;
        g_hall_count[id] += (int32_t)(step * dir_mult);
        g_hall_prev_state[id] = curr_state;
    } else {
        /* Chuyển trạng thái bất thường (nhảy 2 bit do nhiễu hoặc mất cạnh) */
        g_hall_err_count[id]++;
        /* Tự đồng bộ lại trạng thái hiện tại để bắt các cạnh kế tiếp */
        g_hall_prev_state[id] = curr_state;
    }
}

/* =========================================================
 * KHỞI TẠO (INITIALIZATION)
 * ========================================================= */

void Hall_Init(void)
{
    DWT_Init();

    /* Bật clock cho các port GPIO liên quan */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Cấu hình 3 kênh Hall Motor Trái trên PC6, PC7, PC8 */
    GPIO_InitStruct.Pin   = HALL_LEFT_PINS_MASK;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Cấu hình chân Motor Phải: PD12 (HA_R), PD13 (HB_R), PD14 (HC_R) */
    GPIO_InitStruct.Pin   = HALL_HA_R_PIN | HALL_HB_R_PIN | HALL_HC_R_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Cấu hình NVIC:
     * EXTI9_5_IRQn   phục vụ EXTI6 (PC6), EXTI7 (PC7), EXTI8 (PC8) -> Motor Trái
     * EXTI15_10_IRQn phục vụ EXTI12 (PD12), EXTI13 (PD13), EXTI14 (PD14) -> Motor Phải
     * Priority = 5 (nằm trong vùng an toàn của FreeRTOS syscall <= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
     */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* Đọc trạng thái ban đầu */
    g_hall_prev_state[HALL_MOTOR_LEFT]  = Hall_ReadState_Left();
    g_hall_prev_state[HALL_MOTOR_RIGHT] = Hall_ReadState_Right();

    Hall_Reset(HALL_MOTOR_LEFT);
    Hall_Reset(HALL_MOTOR_RIGHT);
}

/* =========================================================
 * BỘ ĐẾM & TRẠNG THÁI (COUNT & STATE ACCESS)
 * ========================================================= */

int32_t Hall_GetCount(int id)
{
    if (id != HALL_MOTOR_LEFT && id != HALL_MOTOR_RIGHT) {
        return 0;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int32_t count = g_hall_count[id];
    if (!primask) {
        __enable_irq();
    }

    return count;
}

void Hall_Reset(int id)
{
    if (id != HALL_MOTOR_LEFT && id != HALL_MOTOR_RIGHT) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_hall_count[id]              = 0;
    g_hall_prev_count[id]         = 0;
    g_hall_speed_filtered[id]     = 0.0f;
    g_hall_edge_period_cycles[id] = 0;
    g_hall_last_edge_dwt[id]      = DWT->CYCCNT;
    g_hall_last_dir[id]           = 0;
    g_hall_channel_edges[id][HALL_PHASE_A] = 0;
    g_hall_channel_edges[id][HALL_PHASE_B] = 0;
    g_hall_channel_edges[id][HALL_PHASE_C] = 0;
    g_hall_win_head[id]           = 0;
    g_hall_win_count[id]          = 0;
    for (uint8_t w = 0; w < HALL_SPEED_WINDOW_SIZE; w++) {
        g_hall_edge_window[id][w] = 0;
    }
    if (!primask) {
        __enable_irq();
    }
}

uint32_t Hall_GetErrorCount(int id)
{
    if (id != HALL_MOTOR_LEFT && id != HALL_MOTOR_RIGHT) {
        return 0;
    }
    return g_hall_err_count[id];
}

uint32_t Hall_GetChannelEdgeCount(int id, HallPhase phase)
{
    if ((id != HALL_MOTOR_LEFT && id != HALL_MOTOR_RIGHT) ||
        phase < HALL_PHASE_A || phase > HALL_PHASE_C) {
        return 0;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t count = g_hall_channel_edges[id][phase];
    if (!primask) {
        __enable_irq();
    }
    return count;
}

uint8_t Hall_ReadChannel(int id, HallPhase phase)
{
    GPIO_TypeDef *port;
    uint16_t pin;

    if (id == HALL_MOTOR_LEFT) {
        port = GPIOC;
        if (phase == HALL_PHASE_A) pin = HALL_HA_L_PIN;
        else if (phase == HALL_PHASE_B) pin = HALL_HB_L_PIN;
        else if (phase == HALL_PHASE_C) pin = HALL_HC_L_PIN;
        else return 0;
    } else if (id == HALL_MOTOR_RIGHT) {
        port = GPIOD;
        if (phase == HALL_PHASE_A) pin = HALL_HA_R_PIN;
        else if (phase == HALL_PHASE_B) pin = HALL_HB_R_PIN;
        else if (phase == HALL_PHASE_C) pin = HALL_HC_R_PIN;
        else return 0;
    } else {
        return 0;
    }

    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

/* =========================================================
 * EXTI CALLBACK (GỌI TỪ HAL_GPIO_EXTI_Callback)
 * ========================================================= */

void Hall_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Đếm cạnh riêng từng kênh; cấu hình EXTI bắt cả cạnh lên và xuống. */
    if (GPIO_Pin == HALL_HA_L_PIN) g_hall_channel_edges[HALL_MOTOR_LEFT][HALL_PHASE_A]++;
    else if (GPIO_Pin == HALL_HB_L_PIN) g_hall_channel_edges[HALL_MOTOR_LEFT][HALL_PHASE_B]++;
    else if (GPIO_Pin == HALL_HC_L_PIN) g_hall_channel_edges[HALL_MOTOR_LEFT][HALL_PHASE_C]++;
    else if (GPIO_Pin == HALL_HA_R_PIN) g_hall_channel_edges[HALL_MOTOR_RIGHT][HALL_PHASE_A]++;
    else if (GPIO_Pin == HALL_HB_R_PIN) g_hall_channel_edges[HALL_MOTOR_RIGHT][HALL_PHASE_B]++;
    else if (GPIO_Pin == HALL_HC_R_PIN) g_hall_channel_edges[HALL_MOTOR_RIGHT][HALL_PHASE_C]++;

    /* Kiểm tra các chân thuộc Motor Trái (PC6, PC7, PC8) */
    if (GPIO_Pin & (HALL_HA_L_PIN | HALL_HB_L_PIN | HALL_HC_L_PIN)) {
        uint8_t state = Hall_ReadState_Left();
        Hall_Process_ISR(HALL_MOTOR_LEFT, state);
    }

    /* Kiểm tra các chân thuộc Motor Phải (PD12, PD13, PD14) */
    if (GPIO_Pin & (HALL_HA_R_PIN | HALL_HB_R_PIN | HALL_HC_R_PIN)) {
        uint8_t state = Hall_ReadState_Right();
        Hall_Process_ISR(HALL_MOTOR_RIGHT, state);
    }
}

/* =========================================================
 * TÍNH TOÁN VẬN TỐC & VỊ TRÍ
 * ========================================================= */

float Hall_GetSpeed_mps(int id, float dt)
{
    if (id != HALL_MOTOR_LEFT && id != HALL_MOTOR_RIGHT) {
        return 0.0f;
    }

    float ticks_per_wheel_rev = (id == HALL_MOTOR_LEFT) ? HALL_TICKS_PER_REV_L : HALL_TICKS_PER_REV_R;
    if (ticks_per_wheel_rev < 1.0f) {
        ticks_per_wheel_rev = 90.0f; /* Giá trị fallback an toàn */
    }
    float dist_per_edge = WHEEL_CIRC_M / ticks_per_wheel_rev;

    /* Đọc dữ liệu an toàn nguyên tử */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int32_t  cur_count  = g_hall_count[id];
    uint32_t last_edge  = g_hall_last_edge_dwt[id];
    int8_t   last_dir   = g_hall_last_dir[id];
    uint8_t  win_cnt    = g_hall_win_count[id];
    uint32_t sum_cycles = 0;
    for (uint8_t i = 0; i < win_cnt; i++) {
        sum_cycles += g_hall_edge_window[id][i];
    }
    if (!primask) {
        __enable_irq();
    }

    /* 1. Tính delta ticks giữa 2 lần gọi (chu kỳ dt = 5ms) */
    int32_t delta_ticks = cur_count - g_hall_prev_count[id];
    g_hall_prev_count[id] = cur_count;

    /* 2. Kiểm tra DWT CYCCNT và thời gian kể từ cạnh cuối */
    uint32_t now_cycles = DWT->CYCCNT;
    uint32_t elapsed_cycles = now_cycles - last_edge;
    uint32_t timeout_cycles = (SystemCoreClock / 1000U) * HALL_SPEED_TIMEOUT_MS;

    /* Nếu quá timeout mà không có xung mới => Xe đã dừng hẳn */
    if (elapsed_cycles >= timeout_cycles && delta_ticks == 0) {
        g_hall_win_count[id] = 0;
        g_hall_speed_filtered[id] = 0.0f;
        return 0.0f;
    }

    float raw_speed = 0.0f;

    if (now_cycles != 0 && win_cnt > 0 && sum_cycles > 0) {
        /* Tính vận tốc từ chu kỳ các cạnh DWT (độ mịn nanosecond) */
        float total_dist = (float)win_cnt * dist_per_edge;
        float v_calc = (total_dist * (float)SystemCoreClock) / (float)sum_cycles;

        /* Xử lý giảm tốc (Deceleration decay): khi xe đang dừng dần */
        uint32_t avg_edge_cycles = sum_cycles / win_cnt;
        if (elapsed_cycles > avg_edge_cycles && elapsed_cycles > 0) {
            float max_possible_speed = (dist_per_edge * (float)SystemCoreClock) / (float)elapsed_cycles;
            if (v_calc > max_possible_speed) {
                v_calc = max_possible_speed;
            }
        }

        int8_t dir_mult = (id == HALL_MOTOR_LEFT) ? (int8_t)MOTOR_LEFT_DIRECTION : (int8_t)MOTOR_RIGHT_DIRECTION;
        raw_speed = v_calc * (float)(last_dir * dir_mult);
    } else {
        /* Fallback dự phòng trực tiếp từ delta_ticks: đảm bảo không bao giờ bị kẹt 0 khi bánh quay */
        if (delta_ticks != 0 && dt > 1e-4f) {
            float rev_per_sec = ((float)delta_ticks / dt) / ticks_per_wheel_rev;
            raw_speed = rev_per_sec * WHEEL_CIRC_M;
        } else {
            raw_speed = g_hall_speed_filtered[id];
        }
    }

    /* 3. Bộ lọc thông thấp bậc 1 (alpha = 0.15) làm mượt hoàn hảo tín hiệu cho PID, triệt tiêu gai xung Hall */
    const float alpha = 0.15f;
    g_hall_speed_filtered[id] = alpha * raw_speed + (1.0f - alpha) * g_hall_speed_filtered[id];

    /* Giữ speed ở mức 0 nếu cực nhỏ và không có xung */
    if (fabsf(g_hall_speed_filtered[id]) < 0.005f && delta_ticks == 0) {
        g_hall_speed_filtered[id] = 0.0f;
    }

    return g_hall_speed_filtered[id];
}

float Hall_GetPosition_m(int id)
{
    if (id != HALL_MOTOR_LEFT && id != HALL_MOTOR_RIGHT) {
        return 0.0f;
    }

    int32_t count = Hall_GetCount(id);

    float ticks_per_wheel_rev = (id == HALL_MOTOR_LEFT) ? HALL_TICKS_PER_REV_L : HALL_TICKS_PER_REV_R;
    if (ticks_per_wheel_rev < 1.0f) {
        ticks_per_wheel_rev = 90.0f;
    }

    float rev = (float)count / ticks_per_wheel_rev;
    return (rev * WHEEL_CIRC_M);
}
