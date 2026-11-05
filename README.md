# STM32F767 Micro-ROS & Robot Control (BLDC Hoverboard)

Hệ thống điều khiển robot 2 bánh vi sai (Differential Drive Mobile Robot) sử dụng vi điều khiển **STM32F767ZI (Nucleo-144)**, tích hợp giao tiếp **micro-ROS** qua Ethernet UDP và điều khiển từ xa bằng **tay cầm PS2 không dây**.

---

## 📑 Mục lục
1. [Sơ đồ khối & Tính năng chính](#1-sơ-đồ-khối--tính-năng-chính)
2. [Sơ đồ chân kết nối (Pinout Diagram)](#2-sơ-đồ-chân-kết-nối-pinout-diagram)
   - [Động cơ & Driver (PWM & DIR)](#a-động-cơ--driver-bldc-pwm--dir)
   - [Cảm biến Hall 3 pha (Encoder)](#b-cảm-biến-hall-3-pha)
   - [Bộ thu tay cầm PS2 (SPI2)](#c-bộ-thu-tay-cầm-ps2-wireless-gamepad)
   - [Cổng mạng Ethernet (micro-ROS)](#d-giao-tiếp-mạng-ethernet-rmii)
   - [Debug UART / ST-Link](#e-debug-uart--st-link)
3. [Cấu hình mạng & Kết nối micro-ROS](#3-cấu-hình-mạng--kết-nối-micro-ros)
   - [Thông số IP & Cổng UDP](#a-thông-số-mạng-network-settings)
   - [Cấu hình card mạng trên máy tính / Mini PC](#b-cấu-hình-ip-trên-máy-tính-ubuntu--mini-pc)
   - [Cài đặt & Chạy micro-ROS Agent](#c-cài-đặt--khởi-chạy-micro-ros-agent)
   - [Các ROS 2 Topics](#d-các-ros-2-topics)
4. [Chế độ hoạt động (Chuyển đổi micro-ROS / PS2)](#4-chế-độ-hoạt-động)
5. [Hướng dẫn Build & Nạp Firmware](#5-hướng-dẫn-build--nạp-firmware)

---

## 1. Sơ đồ khối & Tính năng chính

- **Vi điều khiển:** STM32F767ZIT6 (Cortex-M7 @ 216MHz, FreeRTOS).
- **Hệ thống truyền động:** 2 động cơ Hub Motor BLDC Hoverboard (Direct-Drive, 15 pole pairs, bánh 6.5" - 10").
- **Phản hồi vị trí / vận tốc:** 6 kênh cảm biến Hall (3 kênh mỗi bánh) qua ngắt ngoài EXTI với độ phân giải cao và tính năng chống trôi odometry.
- **Bảo vệ Driver:** Tích hợp bộ tạo dead-time an toàn khi đổi chiều quay (35ms ngắt PWM trước khi lật DIR) tránh hư hỏng MOSFET.
- **Bộ điều khiển:** Vòng lặp PID tốc độ kép kết hợp bộ bù trước (Feedforward KV) và bộ giữ hướng (Heading hold PID).

---

## 2. Sơ đồ chân kết nối (Pinout Diagram)

### a. Động cơ & Driver BLDC (PWM & DIR)

| Tín hiệu | Chân STM32 | Timer / Chức năng | Chú thích |
| :--- | :--- | :--- | :--- |
| **Motor 1 PWM** (Bánh Phải) | `PE9` | `TIM1_CH1` | Tín hiệu PWM điều khiển tốc độ (0 - 1200) |
| **Motor 1 DIR** (Bánh Phải) | `PF15` | GPIO Output | Mức logic chọn chiều quay |
| **Motor 2 PWM** (Bánh Trái) | `PE11` | `TIM1_CH2` | Tín hiệu PWM điều khiển tốc độ (0 - 1200) |
| **Motor 2 DIR** (Bánh Trái) | `PE13` | GPIO Output | Mức logic chọn chiều quay |
| **GND** | `GND` | Ground chung | Nối chung mass với Driver động cơ |

> [!NOTE]
> Timer TIM1 được cấu hình phát xung PWM tần số cao (kèm Main Output Enable - MOE). Tín hiệu DIR được xử lý mềm qua dead-time để đảm bảo dòng điện triệt tiêu hoàn toàn trước khi đảo chiều.

---

### b. Cảm biến Hall 3 pha

Mỗi động cơ BLDC hoverboard sử dụng 3 cảm biến Hall lệch pha 120° để xác định vận tốc và góc quay:

| Cụm động cơ | Tín hiệu Hall | Chân STM32 | Chế độ ngắt (EXTI) |
| :--- | :--- | :--- | :--- |
| **Motor 2 (Trái / Left)** | Hall U / Phase A | `PC6` | `EXTI9_5_IRQHandler` |
| | Hall V / Phase B | `PC7` | `EXTI9_5_IRQHandler` |
| | Hall W / Phase C | `PC8` | `EXTI9_5_IRQHandler` |
| **Motor 1 (Phải / Right)** | Hall U / Phase A | `PD12` | `EXTI15_10_IRQHandler` |
| | Hall V / Phase B | `PD13` | `EXTI15_10_IRQHandler` |
| | Hall W / Phase C | `PD14` | `EXTI15_10_IRQHandler` |
| **Nguồn cấp Hall** | `+5V` hoặc `+3.3V` | 5V / 3V3 Pin | Nguồn cấp cho cảm biến Hall |
| **Mass** | `GND` | GND | Nối chung GND |

> [!IMPORTANT]
> - Các chân `PC6..PC8` và `PD12..PD14` trên STM32F767 đều là chân chịu được điện áp 5V (FT - 5V Tolerant).
> - Nếu cảm biến Hall dạng Open-Collector / Open-Drain, firmware đã bật sẵn điện trở kéo lên nội (`GPIO_PULLUP`). Nếu là dạng Push-Pull 5V, khuyến nghị mắc thêm trở phân áp hoặc đệm trở để bảo vệ an toàn.

---

### c. Bộ thu tay cầm PS2 (Wireless Gamepad)

Giao tiếp sử dụng phần cứng **SPI2**:

| Chân Module PS2 | Chân STM32 | Chức năng SPI2 | Ghi chú |
| :--- | :--- | :--- | :--- |
| **DO (Data Out / MISO)** | `PC2` | `SPI2_MISO` | **Bắt buộc kéo trở PULL-UP** (đã bật nội) |
| **DI (Data In / MOSI)** | `PC3` | `SPI2_MOSI` | Tín hiệu lệnh xuống tay cầm |
| **CLK (Clock / SCK)** | `PB10` | `SPI2_SCK` | Xung clock SPI |
| **CS / ATT (Select)** | `PG0` | GPIO Output | Chọn chip (Active LOW) |
| **VCC** | `3.3V` / `5V` | VCC | Nguồn cấp cho mạch thu không dây |
| **GND** | `GND` | GND | Nối mass chung |

---

### d. Giao tiếp mạng Ethernet (RMII)

Bo STM32F767 Nucleo-144 tích hợp sẵn PHY Ethernet **LAN8742A** và cổng RJ45:
- Tín hiệu RMII mặc định trên bo: `PA1, PA2, PA7, PC1, PC4, PC5, PG11, PG13, PB13`.
- Chỉ cần cắm cáp mạng LAN tiêu chuẩn trực tiếp từ cổng RJ45 của bo Nucleo vào máy tính / Mini PC / Router.

---

### e. Debug UART / ST-Link

- **USART3 (Virtual COM Port tích hợp qua cổng micro-USB ST-Link):**
  - **TX:** `PD8`
  - **RX:** `PD9`
  - **Baudrate:** `115200 bps`, `8N1`

---

## 3. Cấu hình mạng & Kết nối micro-ROS

Firmware sử dụng giao thức **UDP transport** qua stack LwIP (FreeRTOS) để truyền nhận dữ liệu với micro-ROS Agent.

### a. Thông số mạng (Network Settings)

| Thiết bị | Địa chỉ IP | Subnet Mask | Cổng kết nối (UDP Port) |
| :--- | :--- | :--- | :--- |
| **STM32 (Client)** | `192.168.4.50` | `255.255.255.0` | Random ephemeral port |
| **Máy tính / Mini PC (Agent)** | `192.168.4.150` | `255.255.255.0` | **`8888`** |

---

### b. Cấu hình IP trên máy tính Ubuntu / Mini PC

Khi cắm dây mạng trực tiếp từ STM32 vào cổng mạng của máy tính/Mini PC, cần gán IP tĩnh cho card mạng:

```bash
# Xem tên card mạng (ví dụ: eth0, enp3s0 hoặc enx...)
ip link

# Gán IP tĩnh bằng NetworkManager (thay eth0 bằng tên card mạng của bạn):
sudo nmcli con add type ethernet con-name STM32_Link ifname eth0 ip4 192.168.4.150/24
sudo nmcli con up STM32_Link

# Kiểm tra ping đến STM32:
ping 192.168.4.50
```

---

### c. Cài đặt & Khởi chạy micro-ROS Agent

Hỗ trợ ROS 2 (Humble / Iron / Jazzy):

#### 1. Cài đặt micro-ROS Agent:
```bash
# Cài đặt qua apt (thay $ROS_DISTRO bằng jazzy hoặc humble):
sudo apt update
sudo apt install ros-$ROS_DISTRO-micro-ros-agent
```

#### 2. Khởi chạy Agent:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Chạy micro-ROS Agent UDP cổng 8888
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

> [!TIP]
> Để Agent tự động khởi động cùng hệ thống, bạn có thể sử dụng service systemd có sẵn trong thư mục `deployment/microros_agent.service`. Xem hướng dẫn tại `deployment/README.md`.

---

### d. Các ROS 2 Topics

| Tên Topic | Kiểu dữ liệu (Message Type) | Chiều | Tần số | Mô tả |
| :--- | :--- | :--- | :--- | :--- |
| **`/cmd_vel`** | `geometry_msgs/msg/Twist` | **Subscriber** (ROS → STM32) | Tùy ý (khuyến nghị 20-50Hz) | Lệnh vận tốc dài `linear.x` (m/s) và vận tốc góc `angular.z` (rad/s) |
| **`/odomfromSTM32`** | `nav_msgs/msg/Odometry` | **Publisher** (STM32 → ROS) | **50 Hz** | Dữ liệu Odometry (vị trí `pose`, hướng `quaternion`, vận tốc `twist`) |

#### Lệnh kiểm tra nhanh:
```bash
# 1. Kiểm tra danh sách topic
ros2 topic list

# 2. Xem dữ liệu Odometry gửi từ STM32 lên
ros2 topic echo /odomfromSTM32

# 3. Gửi lệnh điều khiển robot chạy tới (0.2 m/s)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 4. Gửi lệnh dừng robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 4. Chế độ hoạt động

Trong file [`Core/Inc/robot_params.h`](Core/Inc/robot_params.h), bạn có thể cấu hình chế độ hoạt động:

```c
/* ====================================================================
 * CẤU HÌNH BẬT / TẮT MICRO-ROS
 * 0: TẮT micro-ROS (chạy test phần cứng độc lập: Hall, PWM, PID, PS2 pad)
 * 1: BẬT micro-ROS (kết nối mạng Ethernet UDP với micro-ROS Agent)
 * ==================================================================== */
#define ENABLE_MICROROS            1
```

- **Khi `ENABLE_MICROROS = 0`:** 
  Robot hoạt động ở chế độ độc lập (Standalone). Bạn có thể điều khiển trực tiếp bằng tay cầm PS2 mà không cần cắm mạng LAN hay bật ROS 2.
- **Khi `ENABLE_MICROROS = 1`:**
  Robot sẽ kết nối với micro-ROS Agent qua mạng Ethernet UDP, nhận lệnh `/cmd_vel` và xuất `/odomfromSTM32`. Khi không có lệnh ROS, bạn vẫn có thể can thiệp qua tay cầm PS2.

---

## 5. Hướng dẫn Build & Nạp Firmware

### Cách 1: Sử dụng STM32CubeIDE (Windows / Linux)
1. Mở STM32CubeIDE, chọn **File -> Open Projects from File System...** và trỏ đến thư mục này.
2. Chọn cấu hình build: **Debug** hoặc **Release**.
3. Nhấn nút **Build** (biểu tượng búa).
4. Kết nối board STM32F767 qua cáp USB và nhấn **Run / Debug**.

### Cách 2: Sử dụng Makefile trên Linux / Ubuntu (không cần CubeIDE)
Dự án đã tích hợp sẵn công cụ build qua dòng lệnh:
```bash
# Build firmware
make -f Makefile.linux -j4

# Nạp trực tiếp vào STM32 qua OpenOCD / ST-Link
bash tools/flash-linux.sh
```
*(Chi tiết xem thêm tại `tools/README.md`)*
