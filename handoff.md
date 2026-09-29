# HANDOFF — Tracking FW v1.4 (BQ25622): phát hiện USB, sleep/wake, cấu hình sạc

Repo: https://github.com/ngoxuanloc2309/TF.git (base commit `bf858d5 "add bq25622"`)
MCU: STM32H563 (HAL, Makefile, arm-none-eabi-gcc). Framework nội bộ: `SynaptiX_FDK/`.
Trạng thái: **mới thảo luận + một patch thử nghiệm đã build được, chưa test trên board.**
Tài liệu tham chiếu: datasheet BQ25620/BQ25622 (SLUSEG2D Rev. D). Số trang bên dưới là số trang trong file PDF, có thể lệch vài trang so với số in ở chân trang.

---

## 1. Mục tiêu và ràng buộc phần cứng

v1.2 và v1.4 chỉ khác khối nguồn:
- **v1.2:** phát hiện USB bằng chân GPIO `PC1` (EXTI rising/falling). Rút USB → vào sleep. Cắm USB → EXTI đánh thức MCU.
- **v1.4:** BQ25622 quản lý nguồn. Phát hiện rút USB bằng cách đọc I2C: `VBUS_STAT == 0` nghĩa là chạy pin.

**Ràng buộc đã chốt (mạch đã gia công, không đổi được):**
- Chân **INT của BQ25622 thả nổi** (không nối MCU).
- **Không còn chân GPIO nào** để detect nguồn hay wake.
- **Không có ADC của MCU để đọc VBAT.** Chỉ có I2C tới BQ.
- Sleep sẽ rất dài.

## 2. Hiện trạng code (v1.2) cần biết

- `Core/Src/gpio.c`: PC1 là EXTI rising+falling, NOPULL (`EXTI1_IRQHandler` gọi `HAL_GPIO_EXTI_IRQHandler`).
- `SynaptiX_FDK/board/sx_board.c`: `HAL_GPIO_EXTI_Rising/Falling_Callback` xử lý cắm/rút (điều khiển `EN_CHARGE`/`EN_DISCHARGE`, gọi `app_mode_full_pw`, `app_notify_usb_connected`, `app_request_sleep`); `check_charge()` đọc `VBUS_PIN` mỗi vòng lặp.
- `SynaptiX_FDK/app/app.c`: máy trạng thái `FULL_POWER → ENTER_SLEEP → SLEEP → WAKE_PUBLISH`. Nhánh `SLEEP` gọi `sx_sleep_manager_enter()` rồi xử lý theo `wake_reason` (RTC / EXTI / unknown).
- `services/sleepmanager/sx_sleep_manager.c`: `enter()` tắt GPS/SIM, đặt RTC wakeup = `SX_TIME_IN_SLEEP` (60 s mặc định), vào STOP.
- `components/sleep/sx_sleep.c`: STOP mode + RTC wakeup timer, khôi phục clock bằng `SystemClock_Config()` sau khi wake.
- `components/bq25622/`: driver **chỉ đọc** (poll + debounce, `bq25622_init/poll/refresh/read_status`). **Chưa được gọi ở đâu** và **Makefile chưa build `bq25622.c`**.
- `services/read_bat` + `board.voltage.v_bat`: đọc VBAT bằng ADC của MCU (v1.2). **v1.4 không dùng được.**

## 3. Kiến thức từ datasheet (đã kiểm tra)

| Mục | Nội dung |
|---|---|
| I2C | Địa chỉ 7-bit `0x6B` |
| Trạng thái VBUS | `REG0x1E` bit[2:0] `VBUS_STAT`: `000` = không có VBUS, `100` = unknown adapter, `111` = đang OTG (tự cấp, không phải USB ngoài). Không cần ADC. |
| Flag/Mask | Flag `0x20–0x22` (xóa khi đọc), Mask `0x23–0x25` |
| INT | Open-drain, xung mức thấp 256 µs khi status đổi. **Đường duy nhất BQ dùng để báo MCU.** Trên I2C, BQ là slave: **không bao giờ tự tạo cạnh trên SCL/SDA.** |
| Default mode | Sau POR chip chạy mặc định. **Ghi bất kỳ thanh ghi nào ⇒ chuyển sang host mode và bật watchdog.** Watchdog hết hạn thì ICHG bị giảm một nửa và nhiều thiết lập bị reset. Tắt watchdog: `REG0x16` trường `WATCHDOG` = `00` (xác nhận lại vị trí bit trong datasheet trước khi ghi). |
| Mặc định | VREG 4.2 V (`REG0x04`), ICHG 1040 mA (`REG0x02`, bước 80 mA), IINDPM 3.2 A (`REG0x06`, bước 20 mA), VSYSMIN 3.52 V (`REG0x0E`, dải 2.56–3.84 V, bước 80 mV). Pre-charge `REG0x10`, termination `REG0x12`. |
| IINDPM | **Tự về 3.2 A (POR) mỗi lần rút adapter** ⇒ phải ghi lại mỗi lần cắm. Giới hạn thật = min(chân ILIM, IINDPM). |
| VBAT_UVLO | `REG0x19` bit5 `VBAT_UVLO` (trang 51): `0` = 2.2 V (mặc định), `1` = 1.8 V; sai số ±0.1 V (bảng điện trang 9). Chỉ có hai giá trị này. Khi chỉ có pin và VBAT dưới ngưỡng: BATFET tắt và **I2C tắt** (mục 8.3.10.1.1, trang 33). Khởi động lại khi VBAT > `VBAT_UVLOZ` (~2.4 V) hoặc cắm adapter. |
| VBAT_LOWV | 2.8 V (falling) / 3.0 V (rising): chỉ là ngưỡng pre-charge↔fast-charge. |
| Ngưỡng xả tuỳ ý | **Không có.** 2.5 V không tồn tại như một tuỳ chọn cắt xả. |
| ADC (VBAT) | Bật bằng `REG0x26.ADC_EN`; `ADC_RATE` = one-shot; kênh tắt bằng `REG0x27`; kết quả `REG0x30` (16-bit little-endian): `VBAT_mV = ((reg >> 1) & 0xFFF) × 1.99`. Mặc định ADC tắt. `ADC_EN` bị reset bởi watchdog. |
| ADC — giới hạn | Khi chỉ có pin, ADC chỉ chạy nếu VBAT > VBAT_LOWV (~2.8 V); **nếu kênh TS_ADC còn bật thì > 3.2 V** (`TS_ADC_DIS` mặc định = 0 ⇒ đang bật). |
| ADC — bẫy | Thanh ghi ADC **giữ giá trị đo cuối và không bao giờ tự xóa** ⇒ nếu ADC không chạy, đọc `REG0x30` cho giá trị cũ. Dùng one-shot và chờ `ADC_DONE_STAT` (`REG0x1D` bit6) = 1, có timeout. |
| Shutdown | `BATFET_CTRL` = `01` tắt BATFET; chỉ vào được khi không có VBUS; thoát khi cắm adapter (hoặc QON). Xác nhận thanh ghi/bit chính xác trong datasheet mục 8.3.9. |

## 4. Kết luận đã thống nhất

1. **PB6/PB7 chuyển sang GPIO lúc sleep không có tác dụng:** khi cắm USB, BQ không tạo tín hiệu nào trên SCL/SDA. Không nối INT vào SDA/SCL (xung INT lúc bus rảnh trông giống START).
2. **Wake khi cắm USB chỉ làm được bằng polling định kỳ** (RTC wakeup ngắn), không có nguồn wake tức thì.
3. **Vẫn phải ghi cấu hình sạc** (VREG, ICHG, IINDPM, ITERM) và tắt watchdog. Cơ chế "đọc lại – nếu khác thì ghi" ở boot và mỗi lần phát hiện cắm USB.
4. **Xả 2.5 V:** chip không hỗ trợ. Vì ADC của BQ ngừng ở ~2.8 V (3.2 V nếu còn TS), không thể cắt chính xác ở 2.5 V bằng firmware. Hai hướng còn lại: (a) cắt bằng firmware ở ~3.0–3.1 V sau khi tắt kênh TS (khuyến nghị, kèm UVLO 2.2 V làm chốt cuối), (b) chỉ dựa vào UVLO 2.2 V/1.8 V. **Chưa chốt**, cần thông số cell.
5. Bỏ đường đọc VBAT bằng ADC MCU ở v1.4; VBAT lấy từ ADC của BQ.
6. Đo VBAT khi SIM/GPS đang tắt (tránh sụt áp do LTE); tần suất thấp (mỗi chu kỳ thức dài hoặc mỗi N lần thức ngắn).

## 5. Thiết kế đề xuất cho sleep/wake (chưa code)

- **Hai chu kỳ:** thức ngắn (mặc định 10 s, cấu hình được trong `app_config.h`) chỉ để đọc `VBUS_STAT`; thức dài (chu kỳ publish, `SX_TIME_IN_SLEEP`) mới bật GPS/SIM.
- Trong `sx_sleep_manager_enter()`: tắt GPS/SIM **một lần**, rồi lặp `STOP → thức ngắn → đọc I2C → nếu VBUS present thì thoát để vào full power; nếu đủ chu kỳ dài thì thoát để publish`.
- Sau STOP nhớ khôi phục clock trước khi đọc I2C (hoặc tối ưu sau: chạy I2C bằng HSI/CSI, chọn kernel clock I2C1 và tính lại TIMINGR).
- Điện năng: dòng trung bình ≈ dòng STOP + Q_mỗi_lần_thức / chu kỳ. Ước tính thô Q ≈ 50 µC ⇒ ~5 µA ở chu kỳ 10 s. **Chưa đo, cần đo trên board.**
- Khoảng trống từ lúc cắm đến lúc phát hiện: chip dùng giới hạn dòng vào mặc định ⇒ **kiểm tra điện trở ILIM trên schematic.**
- Khi phát hiện cắm: ghi lại IINDPM (và kiểm tra cấu hình) rồi mới chạy tiếp.

## 6. Patch đã làm (file `v1.4-bq25622.patch`, base `bf858d5`)

Được tạo trên nhánh cục bộ `v1.4-bq25622` trong sandbox (nhánh này không còn sau khi hết phiên, **chỉ còn file patch**). Build bằng `make` thành công (arm-none-eabi-gcc, link OK). Các warning trong file đã sửa đều có từ trước. **Chưa test trên board.**

Patch được viết cho phương án dùng INT trên PC1, nên đã lỗi thời một phần:

**Giữ lại (vẫn đúng với hướng mới):**
- Thêm `bq25622.c` vào `synaptix.mk`, `-I components/bq25622` vào `Makefile`.
- `Board_t` có `bq25622_t bq`, `bq25622_init()` sau khi init I2C.
- `board_power_process()` (poll + debounce) thay cho `check_charge()`; các hàm xử lý cắm/rút chạy trong main loop thay vì trong ISR.
- Macro `BOARD_HAS_CHG_GPIO` (mặc định 0) bọc các thao tác `EN_CHARGE`/`EN_DISCHARGE` (chưa xác nhận v1.4 còn hai chân này không).
- Sửa bug có sẵn: cắm USB trong lúc `ENTER_SLEEP` làm `sleep_requested` bị kẹt ⇒ lần rút sau `app_request_sleep()` bị bỏ qua. Đã reset cờ trong `_handle_usb_connected()`.
- `APP_SLEEP_IF_NO_USB_AT_BOOT` (mặc định 1): boot bằng pin (VBUS=0) thì tự vào chuỗi sleep, chỉ khi BQ đã trả lời I2C.

**Bỏ hoặc hoàn nguyên (chỉ cần khi có INT):**
- `Core/Src/gpio.c` + `TrackingFirmWare.ioc`: PC1 đổi sang falling + pull-up. Hoàn nguyên hoặc đặt input, tuỳ PC1 đang nối gì trên v1.4.
- `HAL_GPIO_EXTI_Falling_Callback` cho BQ_INT, `s_bq_int_pending`, `board_bq_int_pending()`.
- Hook `sx_sleep_abort_requested()` + đoạn PRIMASK trong `sx_sleep.c`, chỉnh `sx_sleep_set_exti_wake()`.
- `bq25622_handle_int()` trong driver (có thể giữ, không dùng).
- Logic nhánh `SLEEP` trong `app.c` dựa trên `WAKE_REASON_EXTI` và INT pending: cần viết lại theo mô hình polling ở mục 5.

## 7. Việc cần làm tiếp

1. **Chốt thông số cell:** dung lượng (mAh), điện áp sạc đầy, điện áp xả tối thiểu ghi trong datasheet. Từ đó chọn VREG, ICHG, IINDPM, ITERM và ngưỡng cắt firmware.
2. Kiểm tra schematic: điện trở ILIM, cách cấp 3.3 V cho MCU, `EN_CHARGE`/`EN_DISCHARGE` còn hay không, pull-up I2C.
3. Mở rộng driver BQ: `reg_write8/16`, `bq25622_config_apply()` (ICHG/VREG/IINDPM/ITERM, tắt watchdog, cấu hình ADC), verify-and-apply, đọc VBAT one-shot có timeout + kiểm tra `ADC_DONE_STAT`. Xác nhận bit-level trong datasheet cho mọi thanh ghi trước khi ghi.
4. Viết lại sleep theo mục 5; áp lại phần "giữ lại" của patch và hoàn nguyên phần "bỏ".
5. Bỏ `sx_read_bat`/ADC MCU ở v1.4. **Cần grep** nơi dùng `board.voltage.v_bat` (ví dụ trong payload publish) và thay bằng VBAT từ BQ.
6. Cắt xả: hàm kiểm tra VBAT ở lúc SIM/GPS tắt; khi dưới ngưỡng thì tắt tải và shutdown qua `BATFET_CTRL`.

## 8. Kế hoạch test trên board

- Rút USB khi đang chạy ⇒ vào sleep (theo log `USB unplugged`).
- Cắm USB lúc đang ngủ ⇒ wake trong vòng một chu kỳ thức ngắn, chạy full power.
- Cắm/rút lúc đang `ENTER_SLEEP` và lúc đang tắt SIM.
- Boot chỉ bằng pin.
- I2C lỗi ⇒ giữ trạng thái cũ, không tự vào sleep nhầm.
- Đo dòng ngủ trung bình với các chu kỳ thức ngắn khác nhau.
- Cắt xả ở ngưỡng đã chọn; kiểm tra tự khởi động lại khi cắm adapter.

## 9. Lưu ý và khác biệt hành vi so với v1.2

- Nếu host USB suspend (`tud_suspend_cb`) trong khi VBUS còn ⇒ code cũ vẫn đưa vào sleep (như v1.2). Cần quyết định muốn giữ hành vi này hay chỉ sleep khi VBUS=0.
- Các con số tiêu thụ điện trong tài liệu này chỉ là ước tính, chưa đo.
- Mọi kết luận về datasheet có thể đối chiếu trong file `bq25622.pdf` (SLUSEG2D Rev. D); các bit chưa xác nhận đã được ghi rõ ở trên.