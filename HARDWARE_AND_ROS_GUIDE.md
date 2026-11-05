# TÀI LIỆU HƯỚNG DẪN KẾT NỐI PHẦN CỨNG & GIAO TIẾP ROS 2 (STM32F767)

Tài liệu chi tiết về sơ đồ chân, nguyên lý kết nối phần cứng và cách thiết lập hệ thống ROS 2 / micro-ROS cho dự án robot 2 bánh vi sai sử dụng vi điều khiển **STM32F767ZI**.

---

## MỤC LỤC
1. [Sơ đồ chân tổng quát (Complete Pinout Table)](#1-sơ-đồ-chân-tổng-quát)
2. [Chi tiết kết nối phần cứng](#2-chi-tiết-kết-nối-phần-cứng)
   - [2.1. Động cơ BLDC & Mạch công suất (Driver)](#21-động-cơ-bldc--mạch-công-suất-driver)
   - [2.2. Cảm biến Hall 3 pha (Feedback Encoder)](#22-cảm-biến-hall-3-pha-feedback-encoder)
   - [2.3. Mạch thu tay cầm PS2 không dây (SPI2)](#23-mạch-thu-tay-cầm-ps2-không-dây-spi2)
   - [2.4. Cổng mạng Ethernet (LwIP UDP)](#24-cổng-mạng-ethernet-lwip-udp)
3. [Hướng dẫn kết nối với ROS 2 / micro-ROS](#3-hướng-dẫn-kết-nối-với-ros-2--micro-ros)
   - [3.1. Cấu hình mạng máy tính / Mini PC](#31-cấu-hình-mạng-máy-tính--mini-pc)
   - [3.2. Cấu hình firmware](#32-cấu-hình-firmware)
   - [3.3. Cài đặt và chạy micro-ROS Agent](#33-cài-đặt-và-chạy-micro-ros-agent)
   - [3.4. Kiểm tra truyền nhận dữ liệu Topics](#34-kiểm-tra-truyền-nhận-dữ-liệu-topics)
4. [Xử lý sự cố thường gặp (Troubleshooting)](#4-xử-lý-sự-cố-thường-gặp-troubleshooting)

---

## 1. Sơ đồ chân tổng quát

| STT | Khối chức năng | Tín hiệu | Chân STM32F767 | Ghi chú kỹ thuật |
| :---: | :--- | :--- | :---: | :--- |
| **1** | **Motor 1 (Phải)** | PWM | `PE9` | `TIM1_CH1` (PWM 0 - 1200) |
| **2** | | DIR | `PF15` | Mức 0/1 (Chiều quay) |
| **3** | **Motor 2 (Trái)** | PWM | `PE11` | `TIM1_CH2` (PWM 0 - 1200) |
| **4** | | DIR | `PE13` | Mức 0/1 (Chiều quay) |
| **5** | **Hall Motor 2 (Trái)** | Phase A | `PC6` | EXTI6 (5V-tolerant) |
| **6** | | Phase B | `PC7` | EXTI7 (5V-tolerant) |
| **7** | | Phase C | `PC8` | EXTI8 (5V-tolerant) |
| **8** | **Hall Motor 1 (Phải)** | Phase A | `PD12` | EXTI12 (5V-tolerant) |
| **9** | | Phase B | `PD13` | EXTI13 (5V-tolerant) |
| **10** | | Phase C | `PD14` | EXTI14 (5V-tolerant) |
| **11** | **Tay cầm PS2 (SPI2)** | MISO / DO | `PC2` | Cần Pull-up (Data Out từ tay cầm) |
| **12** | | MOSI / DI | `PC3` | Command xuống tay cầm |
| **13** | | SCK / CLK | `PB10` | SPI Clock |
| **14** | | CS / ATT | `PG0` | Chip Select (Active LOW) |
| **15** | **Ethernet (RMII)** | RJ45 LAN | Tích hợp trên bo | Kết nối cáp mạng LAN chuẩn |
| **16** | **Debug UART** | TX / RX | `PD8` / `PD9` | USART3 (Virtual COM ST-Link 115200bps) |

---

## 2. Chi tiết kết nối phần cứng

### 2.1. Động cơ BLDC & Mạch công suất (Driver)
- **Động cơ:** Hub Motor BLDC Hoverboard (Direct-Drive, 15 cặp cực).
- **Driver:** Dùng các driver hỗ trợ đầu vào PWM + DIR (như mạch cầu H, ZS-X11D, JY01 hoặc driver Hoverboard mod firmware).
- **Kết nối:**
  - `PE9` -> Chân nhận PWM của Driver Motor Phải.
  - `PF15` -> Chân nhận DIR của Driver Motor Phải.
  - `PE11` -> Chân nhận PWM của Driver Motor Trái.
  - `PE13` -> Chân nhận DIR của Driver Motor Trái.
  - **GND chung:** Nối GND bo STM32 với GND tín hiệu của cả 2 Driver.
- **Tính năng an toàn đảo chiều:**
  - Firmware tích hợp dead-time 35ms (ngắt PWM trước 20ms, sau đó lật chân DIR, rồi mới tăng lại PWM) để chống sốc dòng và tránh chập cầu H.

### 2.2. Cảm biến Hall 3 pha (Feedback Encoder)
- Mỗi bánh xe có 5 dây từ cụm cảm biến Hall: 2 dây nguồn (`VCC`, `GND`) và 3 dây tín hiệu (`U, V, W` hoặc `A, B, C`).
- **Nguồn cấp:** Cấp 5V hoặc 3.3V cho cảm biến Hall từ bo hoặc nguồn ổn áp.
- **Tín hiệu:**
  - Bánh trái: U/A -> `PC6`, V/B -> `PC7`, W/C -> `PC8`.
  - Bánh phải: U/A -> `PD12`, V/B -> `PD13`, W/C -> `PD14`.
- **Lưu ý điện áp:** Tất cả các chân `PC6..PC8` và `PD12..PD14` đều là chân 5V-tolerant (FT). Code đã kích hoạt điện trở kéo lên nội `GPIO_PULLUP`.

### 2.3. Mạch thu tay cầm PS2 không dây (SPI2)
- Mạch thu có 6 chân kết nối chính:
  1. **DAT / DO:** Nối vào `PC2` (STM32 `SPI2_MISO`).
  2. **CMD / DI:** Nối vào `PC3` (STM32 `SPI2_MOSI`).
  3. **CLK:** Nối vào `PB10` (STM32 `SPI2_SCK`).
  4. **ATT / CS:** Nối vào `PG0` (STM32 GPIO Output).
  5. **VCC:** Cấp 3.3V hoặc 5V (khuyến nghị 3.3V hoặc 5V có chung GND).
  6. **GND:** Nối chung GND.

---

## 3. Hướng dẫn kết nối với ROS 2 / micro-ROS

Hệ thống giao tiếp thông qua **micro-ROS UDP Transport** qua đường mạng Ethernet.

### 3.1. Cấu hình mạng máy tính / Mini PC
- **Địa chỉ IP STM32:** `192.168.4.50` (Netmask: `255.255.255.0`)
- **Địa chỉ IP Mini PC / Host:** `192.168.4.150` (Netmask: `255.255.255.0`)
- **Cổng UDP Agent:** `8888`

#### Các bước thiết lập trên Ubuntu Linux:
1. Nối dây mạng LAN giữa bo Nucleo-F767ZI và cổng Ethernet máy tính.
2. Thiết lập IP tĩnh cho card mạng:
   ```bash
   # Tìm tên cổng mạng (ví dụ eth0, enp2s0,...)
   ip link

   # Tạo kết nối IP tĩnh
   sudo nmcli con add type ethernet con-name STM32_Link ifname <TÊN_CARD_MẠNG> ip4 192.168.4.150/24
   sudo nmcli con up STM32_Link

   # Kiểm tra kết nối vật lý bằng lệnh ping
   ping 192.168.4.50
   ```

### 3.2. Cấu hình firmware
Trước khi nạp hoặc khi muốn bật/tắt micro-ROS, kiểm tra file `Core/Inc/robot_params.h`:
```c
#define ENABLE_MICROROS            1   /* 1: Bật micro-ROS, 0: Tắt để test PS2 độc lập */
```

### 3.3. Cài đặt và chạy micro-ROS Agent
Trên máy tính Ubuntu đã cài ROS 2 (Humble / Jazzy):

```bash
# 1. Cài đặt package micro_ros_agent
sudo apt update
sudo apt install ros-$ROS_DISTRO-micro-ros-agent

# 2. Khởi chạy micro_ros_agent lắng nghe cổng UDP 8888
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
Khi kết nối thành công, màn hình terminal sẽ hiển thị các thông báo khởi tạo Session, Node, Publisher `/odomfromSTM32` và Subscriber `/cmd_vel`.

### 3.4. Kiểm tra truyền nhận dữ liệu Topics

#### 1. Kiểm tra Topics đang hoạt động:
```bash
ros2 topic list
```
Kết quả hiển thị:
```text
/cmd_vel
/odomfromSTM32
/parameter_events
/rosout
```

#### 2. Xem dữ liệu Odometry (tần số 50Hz):
```bash
ros2 topic echo /odomfromSTM32
# Hoặc kiểm tra tần số gửi:
ros2 topic hz /odomfromSTM32
```

#### 3. Điều khiển vận tốc robot qua `/cmd_vel`:
```bash
# Chạy tiến với vận tốc 0.2 m/s:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Xoay tại chỗ sang trái:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.5, y: 0.0, z: 0.0}}"

# Dừng hẳn robot:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 4. Xử lý sự cố thường gặp (Troubleshooting)

1. **Không ping được `192.168.4.50`:**
   - Kiểm tra đèn LED cổng RJ45 trên bo Nucleo (đèn xanh và cam có sáng/nhấp nháy không).
   - Kiểm tra IP máy tính đã đúng lớp mạng `192.168.4.150/24` hay chưa (`ip addr show`).
   - Đảm bảo firmware đã bật LwIP và hoàn tất quá trình khởi động mạng.

2. **Agent không nhận Session từ STM32:**
   - Kiểm tra xem port `8888` có bị chặn bởi tường lửa không: `sudo ufw allow 8888/udp`.
   - Kiểm tra xem file `Core/Inc/robot_params.h` đã đặt `#define ENABLE_MICROROS 1` chưa.

3. **Cảm biến Hall không đếm xung:**
   - Kiểm tra chân cấp nguồn 5V/3.3V và mass GND của cảm biến Hall.
   - Thử quay bánh xe bằng tay và đo điện áp trên các chân Hall (điện áp phải dao động giữa 0V và 3.3V/5V khi bánh xe xoay).
