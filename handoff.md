# HANDOFF — Tracking FW v1.4 (BQ25622): sleep/wake theo nhu cầu, phát hiện USB, cấu hình sạc

Repo: https://github.com/ngoxuanloc2309/TF.git
MCU: STM32H563 (HAL, Makefile, arm-none-eabi-gcc). Framework nội bộ: `SynaptiX_FDK/`.
Tài liệu tham chiếu: datasheet BQ25620/BQ25622 (SLUSEG2D Rev. D). Số trang là số trang trong file PDF, có thể lệch vài trang so với số in ở chân trang.
Repo tham khảo (weather station, không có chân cắt nguồn nên tắt module bằng lệnh phần mềm): https://github.com/logan123synaptix/WS_v1.git

**Trạng thái (cập nhật 05/10/2026):** Phase 0 đã chạy được trên board (đọc được chip nguồn qua I2C). Phần sleep/wake (Phase 1–6) vẫn là kế hoạch, chưa có code. Cùng phiên này đã debug và sửa chuỗi SIM76xx/MQTT/publish (xem **mục 11**). **Chip nguồn trên board không phải BQ25622 như giả định (xem mục 7a).** Chưa đo nguồn modem, chưa test sleep trên board.

---

## 0. Quy ước làm việc (bắt buộc)

- **Tuyệt đối không gửi patch.** Sửa xong chỉ present các file đã sửa (đầy đủ nội dung) để người dùng paste và push.
- Chia việc thành các phase nhỏ (mục 8), mỗi phase test độc lập được trên board. Không làm một phát hết.
- Các kết luận về datasheet và về phần cứng chưa xác nhận phải ghi rõ là "chưa xác nhận". Không đoán bit/thanh ghi, phải đối chiếu datasheet trước khi ghi.

## 1. Mục tiêu và ràng buộc phần cứng

v1.2 và v1.4 chỉ khác khối nguồn:
- **v1.2:** phát hiện USB bằng chân `PC1` (`VBUS_PIN`, EXTI rising/falling). Rút USB thì vào sleep, cắm USB thì EXTI đánh thức MCU.
- **v1.4:** BQ25622 quản lý nguồn. Phát hiện rút/cắm USB bằng đọc I2C: `VBUS_STAT == 0` nghĩa là chạy pin.

**Ràng buộc đã chốt (mạch đã gia công, không đổi được):**
- Chân **INT của BQ25622 thả nổi** (không nối MCU).
- **Không còn GPIO nào** để detect nguồn hay wake.
- **Không có ADC của MCU để đọc VBAT.** Chỉ có I2C tới BQ.
- Sleep rất dài.

Hệ quả: **không có nguồn wake tức thì khi cắm USB.** Chỉ làm được bằng polling định kỳ (RTC wakeup ngắn). Đổi PB6/PB7 sang GPIO vô ích vì BQ là I2C slave, không tự tạo cạnh trên SCL/SDA. Không nối INT vào SDA/SCL.

## 2. Hiện trạng code (v1.2) cần biết

Luồng hiện tại (`SynaptiX_FDK/app/app.c`):
- **`FULL_POWER`:** có USB, chạy bình thường.
- Rút USB: `HAL_GPIO_EXTI_Falling_Callback` (`board/sx_board.c`) gọi `app_request_sleep()`. Chuyển sang **`ENTER_SLEEP`**: vẫn chạy GPS/MQTT, publish "enter sleep" (GSM + GPS). Thoát khi publish xong, hoặc MQTT mất quá 5 s, hoặc quá `ENTER_SLEEP_TIMEOUT_MS`. Nếu USB cắm lại trong lúc này thì quay về `FULL_POWER`.
- **`SLEEP`:** `sx_user_mqtt_force_disconnect()`, `queue_flush()`, rồi `sx_sleep_manager_enter()`: tắt GPS (`gps_power_off`), tắt SIM (`sim76xx_power_off_blocking`, xung PWRKEY + delay), đặt RTC wakeup, vào STOP.
- Nguồn đánh thức: **RTC** thì sang **`WAKE_PUBLISH`**; **EXTI PC1** (cắm USB) thì về `FULL_POWER`.
- **`WAKE_PUBLISH`:** bật GPS, chờ fix (tối đa `GPS_TIMEOUT_MS` = 130 s), bật SIM và chờ sẵn sàng (tối đa 90 s), kết nối MQTT, publish "wake up", ghi log GPS (`write_gps_log`), cập nhật giờ RTC ngoài từ GPS, rồi quay lại `SLEEP`. Cả chuỗi bị giới hạn bởi `SX_TIME_IN_WAKE` (160 s).
- Chu kỳ ngủ: `SX_TIME_IN_SLEEP` = 60000 ms mặc định, ghi đè bằng `time_sleeps` (giây) trong `config.json`.

Sleep hiện tại **chỉ tắt GPS và SIM**. `_enter_stop()` (`components/sleep/sx_sleep.c`) chỉ abort UART1/UART2, tắt NVIC USB, dừng SysTick rồi WFI; sau wake gọi `SystemClock_Config()`. Không DeInit ngoại vi nào, không tắt flash, IMU, RTC ngoài.

Các thành phần khác:
- `components/bq25622/`: driver **chỉ đọc** (`bq25622_init/poll/refresh/read_status`, poll + debounce). **Chưa được gọi ở đâu, và `bq25622.c` chưa có trong build** (cần thêm vào `synaptix.mk`, thêm `-I` vào `Makefile`).
- `sx_board.c`: `HAL_GPIO_EXTI_*_Callback` xử lý cắm/rút (điều khiển `EN_CHARGE`/`EN_DISCHARGE`, gọi `app_mode_full_pw`, `app_notify_usb_connected`, `app_request_sleep`); `check_charge()` đọc `VBUS_PIN` mỗi vòng lặp.
- `sx_board.c:221` gán `board.voltage.v_bat` từ ADC MCU (`services/read_bat`), và `app.c:330` dùng `v_bat` trong payload publish. **v1.4 không dùng được ADC MCU**, cần thay bằng VBAT từ BQ.
- `services/sx_ex_storage`: `sx_storage_sleep()` và `sx_storage_wake()` hiện **rỗng**.
- `components/sim76xx/sim76xx.c`: **chưa có `AT+CTZU`/`AT+CCLK?`**. Giờ RTC ngoài chỉ được cập nhật từ GPS.
- Bug có sẵn: cắm USB trong lúc `ENTER_SLEEP` làm `sleep_requested` bị kẹt, lần rút sau `app_request_sleep()` bị bỏ qua. Cần reset cờ này khi xử lý cắm USB.

## 3. Chân cắt nguồn có sẵn (khác WS_v1)

Tracking FW có GPIO cắt nguồn cho từng module (khai báo trong `Core/Inc/main.h`, `board/sx_board.h`). Mức suy ra từ code: **LOW = bật, HIGH = tắt** (`gpio.c` khởi tạo LOW nên mặc định bật).

| Module | Chân | Ghi chú |
|---|---|---|
| Exflash W25Q128 | `Flash_PWR` = PC4 (`SPI_PW_PIN`) | `sx_W25Q128_power_down/up()` đã có. SPI: CS = PA4, SCK/MISO/MOSI = PA5/6/7 |
| IMU BNO055 | `IMU_EN_PW` = PB4 | `bno055_power_off/on()` đã có. `IMU_RESET` = PB5 (cùng chân `I2C1_RESET`) |
| RTC ngoài RX8130CE | `RTC_EN_PW` = PB3 | **Không cắt** (cần giữ giờ, chưa xác nhận backup pin) |
| GPS | `GPS_PWR` = PC15 | đã dùng |
| LTE | `LTE_PWR` = PC13, `LTE_PWR_Key` = PC14 | đã dùng, **giữ nguyên phần SIM** |
| Sạc | `EN_CHARGE` = PC6, `EN_DISCHARGE` = PD12 | chưa xác nhận v1.4 còn hai chân này không |

I2C1: PB6/PB7 (BQ, RTC, IMU dùng chung). VBUS v1.2: PC1.

## 4. Thiết kế sleep/wake mới (đã thống nhất)

Nguyên tắc: **ngoại vi theo nhu cầu.** Khi sleep thì DeInit hết và cắt nguồn; mỗi lần thức chỉ bật đúng thứ tác vụ đó cần, làm xong thì DeInit/cắt lại.

**Vào sleep (một lần):**
- Wait busy flash, kéo `Flash_PWR` và `IMU_EN_PW` lên HIGH.
- `HAL_SPI_DeInit`, `HAL_I2C_DeInit`, các UART (pin về analog, như WS_v1). Đưa CS (PA4) và `IMU_RESET` (PB5) về LOW/analog để không đẩy tín hiệu vào chip đã mất nguồn.
- Tắt GPS/SIM như hiện tại (giữ nguyên chuỗi SIM). UART log DeInit **sau cùng**.
- Vào STOP.

**Ba loại thức dậy:**

| Loại | Việc làm | Bật lại gì |
|---|---|---|
| Wake-fake (mặc định ~10 s) | Đọc `VBUS_STAT` | Chỉ I2C1 (BQ). Xong thì DeInit và ngủ tiếp |
| Publish (`time_sleeps`, mặc định 60 s) | GPS → không có fix thì đọc `last_gps` từ flash → có fix thì ghi flash → SIM/MQTT publish → cập nhật giờ RTC | GPS+UART2; flash+SPI1 (bật tạm khi cần); LTE+UART1; I2C1 khi ghi RTC. Xong thì release hết |
| Wake-real (VBUS thật) | Lên full power | Tất cả: flash, IMU, USB, nạp calib IMU, ghi lại cấu hình sạc |

**Quy tắc nguồn:**
- Flash: mặc định tắt, chỉ bật khi ghi/đọc log; `sx_storage_*` nên tự bật nguồn nếu đang tắt để chỗ gọi (`write_gps_log`...) không phải biết về nguồn.
- IMU: tắt hẳn ở mọi chế độ ngủ, kể cả chu kỳ publish. Chỉ bật khi wake-real. Vì IMU mất calib khi mất nguồn, wake-real phải: bật nguồn, chờ boot, `bno055_init`, nạp calib (file `IMU_CALIB_FILE_PATH` nằm trong flash nên **bật flash trước, IMU sau**).
- Đề xuất lớp `acquire(res)`/`release(res)` cho I2C1, SPI+flash, UART GPS, UART LTE, UART log, IMU; `release_all()` gọi ở đầu chuỗi sleep.
- Đếm chu kỳ publish (cộng dồn các lần wake-fake) phải theo RTC, không lệch dần.
- UART log bị DeInit lúc ngủ nên khó debug: dùng cờ compile để giữ log UART khi debug, tắt khi đo dòng.
- Thêm trường mới `time_check_vbus` (giây) trong `config.json` (chu kỳ wake-fake). `config.json` hiện có: `apn`, `mqtt`, `time_sleeps`, `device_id`, `time_publish`.

## 5. Bài học từ WS_v1 (đã gặp lỗi thật ở đó)

- Trước khi cắt/đưa flash vào power-down phải `w25q_wait_busy()`. Làm ngay sau ghi/erase thì treo bus. Cắt nguồn giữa lúc ghi còn có thể hỏng dữ liệu.
- Chỉ cut clock (`__HAL_RCC_xxx_CLK_DISABLE`) không đủ; phải `HAL_xxx_DeInit()` (MspDeInit đưa pin về analog) thì dòng mới giảm rõ. I2C có pull-up ngoài, nếu pin để AF_OD mà bus đang bị kéo thấp thì rò suốt STOP. TIM/LPTIM phải dùng `DeInit`, không dùng CLK_DISABLE thô, nếu không sau wake `MX_*_Init()` bỏ qua MspInit và treo ở `Error_Handler()`.
- `SX_RESUME_TICS()` (`HAL_ResumeTick`) phải gọi **trước** `SystemClock_Config()`, vì `HAL_RCC_OscConfig()` chờ HSE bằng `HAL_GetTick()`; tick còn tắt thì treo vĩnh viễn. Code tracking FW hiện đang gọi `SystemClock_Config()` trước `SX_RESUME_TICS()` (`sx_sleep.c`), **cần đảo lại**.
- Sau khi tắt modem bằng PWRKEY, `mqtt->state` vẫn CONNECTED nếu không reset, nên sau wake `connect()` bị bỏ qua. Tracking FW đã gọi `sx_user_mqtt_force_disconnect()` trước khi tắt SIM; giữ nguyên thứ tự này.
- Không dùng một instance flash chưa init (từng gây HardFault); luôn đi qua instance thật trong `sx_ex_storage.c`.
- IWDG bị đóng băng lúc STOP nhờ option byte; nếu tracking FW có IWDG thì cần refresh ngay trước STOP (chưa kiểm tra tracking FW có dùng IWDG không).

## 6. Giờ mạng (NITZ) cho RTC ngoài

- WS_v1 gửi `AT+CTZU=1`, đọc `AT+CCLK?`, parse `yy/MM/dd,hh:mm:ss±zz`, chuẩn hóa về UTC (`mktime`/`gmtime`), ghi vào RX8130CE (`time_sync.c`). Làm mỗi lần wake vì RTC ngoài lệch (số đo của người dùng: ~38 s sau vài chục phút; chưa kiểm chứng độc lập). Fallback dùng giờ GPS nếu modem không có giờ.
- Bộ AT manual dòng A76XX có `AT+CCLK`, `AT+CTZU`, `AT+CTZR`, `AT+CNTP` (NTP). A7680C thuộc dòng Cat 1 cùng hãng, **nhưng chưa có tài liệu nào xác nhận thẳng A7680C nằm trong đúng bộ manual đó**: cần kiểm chứng trên board.
- Rủi ro: NITZ phụ thuộc nhà mạng (nếu không gửi giờ thì `CCLK?` trả giờ mặc định chưa đồng bộ); `AT+CNTP` cần đã có kết nối dữ liệu và một NTP server.
- Kiểm chứng trước khi code (gửi tay, lúc SIM đã đăng ký mạng): `AT+CGMM`, `AT+CTZU=1`, `AT+CCLK?`.
- Thứ tự nguồn giờ đề xuất: NITZ ưu tiên, GPS dự phòng. Ghi RTC ngoài cần `acquire` I2C1.

## 7a. Chip nguồn thực tế trên board (phát hiện khi test Phase 0)

- Scan I2C1 chỉ thấy `0x29` (BNO055), `0x32` (RX8130CE), `0x6A`. **Không có `0x6B`**, nên driver ở `0x6B` báo `not found`.
- Datasheet BQ25620/BQ25622 (SLUSEG2D): địa chỉ 7-bit `0x6B`. Tra datasheet thì **BQ25628/BQ25628E/BQ25629 dùng `0x6A`**, cùng dải thanh ghi 0x02–0x38. Nhiều khả năng đây là chip trên board, **nhưng chưa xác nhận** (cần mã in trên IC/schematic).
- Đã đổi driver sang `0x6A` (macro `BQ25622_I2C_ADDR7` trong `bq25622.h`, có thể override bằng `-D`). Log boot: `raw @0x6A: PART_INFO(0x38)=0x12 STATUS0(0x1D)=0x11 STATUS1(0x1E)=0x04`, driver báo `PN=2 (unknown) rev=2` (driver chỉ biết PN 0 = BQ25620, 1 = BQ25622). Ý nghĩa PN=2 **chưa đối chiếu datasheet**.
- `VBUS_STAT=100b` (4) khi cắm USB. Theo bảng 8-2 datasheet BQ25629 (dò D+/D-), giá trị này là "unknown 5-V adapter", IINDPM = **500 mA**. BQ25628 không có dò D+/D-, nên chưa rõ bảng này áp dụng cho chip nào. Cần xác nhận part trước.
- **Không chạy hàm ghi BQ (Phase 5/6: `config_apply`, cắt xả, `battery_disconnect`) cho đến khi đã đối chiếu register map của đúng part.** Toàn bộ bảng mục 7 là của BQ25622 và có thể sai bit/giá trị với BQ25628/29.
- `READ_BAT` (ADC MCU, `services/read_bat`) đọc ~0.73 V: pin gần như không nối hoặc chia áp sai. Board đang chạy chỉ bằng USB. `CHG_STAT` nhảy 0/1/2 khi không có pin (chưa xác nhận nguyên nhân).

## 7. Kiến thức từ datasheet BQ25622 (từ handoff cũ, chưa xác nhận lại bit-level)

| Mục | Nội dung |
|---|---|
| I2C | Địa chỉ 7-bit `0x6B` |
| Trạng thái VBUS | `REG0x1E` bit[2:0] `VBUS_STAT`: `000` = không có VBUS, `100` = unknown adapter, `111` = đang OTG (tự cấp, không phải USB ngoài). Không cần ADC |
| Flag/Mask | Flag `0x20–0x22` (xóa khi đọc), Mask `0x23–0x25` |
| INT | Open-drain, xung mức thấp 256 µs khi status đổi. Trên I2C, BQ là slave, không bao giờ tự tạo cạnh trên SCL/SDA |
| Default mode | Sau POR chip chạy mặc định. **Ghi bất kỳ thanh ghi nào ⇒ chuyển sang host mode và bật watchdog.** Watchdog hết hạn thì ICHG bị giảm một nửa và nhiều thiết lập bị reset. Tắt watchdog: `REG0x16` trường `WATCHDOG` = `00` (xác nhận vị trí bit trước khi ghi) |
| Mặc định | VREG 4.2 V (`REG0x04`), ICHG 1040 mA (`REG0x02`, bước 80 mA), IINDPM 3.2 A (`REG0x06`, bước 20 mA), VSYSMIN 3.52 V (`REG0x0E`). Pre-charge `REG0x10`, termination `REG0x12` |
| IINDPM | **Tự về 3.2 A mỗi lần rút adapter** ⇒ phải ghi lại mỗi lần cắm. Giới hạn thật = min(chân ILIM, IINDPM) |
| VBAT_UVLO | `REG0x19` bit5: `0` = 2.2 V (mặc định), `1` = 1.8 V; sai số ±0.1 V. Chỉ có hai giá trị. Khi chỉ có pin và VBAT dưới ngưỡng thì BATFET tắt và **I2C tắt**; khởi động lại khi VBAT > ~2.4 V hoặc cắm adapter |
| VBAT_LOWV | 2.8 V (falling) / 3.0 V (rising): chỉ là ngưỡng pre-charge/fast-charge |
| Ngưỡng xả tuỳ ý | **Không có.** 2.5 V không tồn tại như tuỳ chọn cắt xả |
| ADC (VBAT) | Bật bằng `REG0x26.ADC_EN`; `ADC_RATE` = one-shot; kênh tắt bằng `REG0x27`; kết quả `REG0x30` (16-bit little-endian): `VBAT_mV = ((reg >> 1) & 0xFFF) × 1.99`. Mặc định ADC tắt; `ADC_EN` bị reset bởi watchdog |
| ADC — giới hạn | Khi chỉ có pin, ADC chỉ chạy nếu VBAT > ~2.8 V; **nếu kênh TS_ADC còn bật thì > 3.2 V** (`TS_ADC_DIS` mặc định = 0) |
| ADC — bẫy | Thanh ghi ADC giữ giá trị đo cuối và không tự xóa. Dùng one-shot và chờ `ADC_DONE_STAT` (`REG0x1D` bit6) = 1, có timeout |
| Shutdown | `BATFET_CTRL` = `01` tắt BATFET; chỉ vào được khi không có VBUS; thoát khi cắm adapter (hoặc QON). Xác nhận thanh ghi/bit trong datasheet mục 8.3.9 |

## 8. Kế hoạch phase (mỗi phase test độc lập trên board)

- **Phase 0 — Đọc BQ (ĐÃ XONG phần đọc):** `bq25622.c` đã vào build, đọc được ở `0x6A`, log `BQ VBUS_STAT=100 CHG_STAT=0 -> VBUS present`. **Còn lại:** chưa test rút USB (`000`) và chưa xác nhận part (mục 7a).
- **Phase 1 — SIM76xx có giờ mạng:** thêm `AT+CTZU=1`, `AT+CCLK?`, parse UTC, hàm lấy giờ. Điều kiện: kết quả kiểm chứng trên board ở mục 6.
- **Phase 2 — Cắt/bật nguồn flash và IMU:** điền `sx_storage_sleep/wake`, hàm nguồn IMU, DeInit SPI/I2C, xử lý CS/`IMU_RESET`. Test: cắt flash, đo dòng, bật lại đọc JEDEC ID, ghi/đọc thử. **Test rủi ro số 1:** cắt IMU xong có còn đọc được BQ qua I2C không (IMU mất nguồn có thể kéo SDA/SCL qua diode bảo vệ). Nếu lỗi thì dùng SUSPEND cho IMU thay vì cắt nguồn.
- **Phase 3 — Quản lý ngoại vi theo nhu cầu + vòng wake-fake:** lớp `acquire/release`, vòng STOP → I2C1 → đọc `VBUS_STAT` → wake-real hoặc ngủ tiếp, `time_check_vbus`, sửa thứ tự `SX_RESUME_TICS()`/`SystemClock_Config()`, sửa bug `sleep_requested`.
- **Phase 4 — Chu kỳ publish theo nhu cầu:** GPS → last_gps từ flash → ghi flash → SIM publish → cập nhật giờ RTC → release hết. Đếm chu kỳ theo RTC.
- **Phase 5 — Cấu hình sạc BQ:** `reg_write8/16`, `bq25622_config_apply()` (ICHG/VREG/IINDPM/ITERM, tắt watchdog, cấu hình ADC), verify-and-apply lúc boot và mỗi lần phát hiện cắm USB. Cần thông số cell.
- **Phase 6 — VBAT và cắt xả:** VBAT one-shot từ ADC BQ có timeout + kiểm tra `ADC_DONE_STAT`, thay `board.voltage.v_bat` (`sx_board.c:221`, `app.c:330`), bỏ `services/read_bat`. Đo VBAT khi SIM/GPS tắt. Khi dưới ngưỡng thì tắt tải và shutdown qua `BATFET_CTRL`.

Phase 3 phụ thuộc Phase 2; Phase 4 phụ thuộc 1–3; Phase 5 và 6 độc lập với luồng sleep.

## 9. Quyết định còn mở / cần người dùng cung cấp

1. **Thông số cell:** dung lượng (mAh), điện áp sạc đầy, điện áp xả tối thiểu trong datasheet. Từ đó chọn VREG, ICHG, IINDPM, ITERM và ngưỡng cắt xả.
2. **Cắt xả:** chip không hỗ trợ 2.5 V. Hướng khuyến nghị: firmware cắt ~3.0–3.1 V (sau khi tắt kênh TS), UVLO 2.2 V làm chốt cuối. **Chưa chốt.**
3. **Schematic:** điện trở ILIM (khoảng trống từ lúc cắm đến lúc phát hiện chip dùng giới hạn dòng mặc định), cách cấp 3.3 V cho MCU, `EN_CHARGE`/`EN_DISCHARGE` còn hay không, pull-up I2C, cách RX8130CE giữ giờ (backup pin).
4. **Model SIM thật** (`AT+CGMM`) và kết quả `AT+CCLK?` (mục 6).
5. **Host USB suspend** (`tud_suspend_cb`) trong khi VBUS còn: code cũ vẫn đưa vào sleep (như v1.2). Quyết định giữ hay chỉ sleep khi VBUS=0.
6. Mục "log" trong danh sách bật ngoại vi ở chu kỳ publish: đang hiểu là UART log debug (chưa xác nhận).
7. **Part number thật của chip nguồn** (BQ25628/BQ25628E/BQ25629/khác) và nơi dò USB D+/D- (có nối hay để hở).
8. **Nguồn cho modem:** pin có nối không, USB dùng cổng PC hay adapter; nguyên nhân modem reset (mục 11.4).

## 10. Kế hoạch test trên board

- Rút USB khi đang chạy thì vào sleep.
- Cắm USB lúc đang ngủ thì wake trong vòng một chu kỳ wake-fake, chạy full power.
- Cắm/rút lúc đang `ENTER_SLEEP` và lúc đang tắt SIM.
- Boot chỉ bằng pin.
- I2C lỗi thì giữ trạng thái cũ, không tự vào sleep nhầm.
- Cắt IMU rồi đọc BQ; cắt flash rồi bật lại đọc/ghi (kiểm tra filesystem còn nguyên).
- Ghi log flash trong chu kỳ publish rồi tắt lại nhiều lần liên tiếp.
- Đo dòng ngủ trung bình với các `time_check_vbus` khác nhau. Con số ước tính ~5 µA ở chu kỳ 10 s **chưa đo**.
- Cắt xả ở ngưỡng đã chọn; kiểm tra tự khởi động lại khi cắm adapter.
---

## 11. Phiên debug SIM76xx / MQTT / publish (05/10/2026)

Mọi file bên dưới là bản đầy đủ đã present. Bản gốc lấy từ repo (nhánh hiện tại); nếu người dùng có sửa local khác thì cần gộp lại.

### 11.1 Các lỗi đã tìm ra và sửa

| # | Triệu chứng | Nguyên nhân | Sửa | File | Trạng thái |
|---|---|---|---|---|---|
| 1 | `AT+COPS=0` → `+CME ERROR: unknown error`, báo TIMEOUT | Gửi lệnh khi modem chưa xong init SIM (URC `*ATREADY`, `+CPIN: READY`, `SMS DONE` đến sau). `res_fail` chỉ khớp `\r\nERROR\r\n`, không khớp `+CME ERROR` | Thêm bước `AT+CPIN?` poll (1.5 s, tối đa 15 lần, rồi power-cycle) trước `COPS`. Trường `fail_on_cme` (opt-in, mặc định 0, chỉ bật cho `CPIN`/`COPS?`/`COPS=0`) để `+CME ERROR` báo lỗi ngay. `COPS=0` lỗi thì nghỉ 2 s retry, 3 lần thì power-cycle. Trì hoãn retry không chặn (field `defer_action`) | `modem.c/.h`, `sim76xx.c/.h` | Đã vào repo (commit "fail") |
| 2 | Publish lỗi, log luôn `Publish fail 1/3`, không bao giờ restart | `s_publish_retry = 0;` ở đầu `_on_publish`, trước `++` | Bỏ dòng reset; chỉ reset khi thành công. Retry mà `sx_mqtt_publish` trả `< 0` thì tính là một lần fail. `s_publishing` không còn kẹt khi gửi không đi được | `sx_user_mqtt.c` | Đã vào repo |
| 3 | Spam GSM+GPS mỗi vòng lặp | `config_json.s_time_publish` không có mặc định (static = 0), `TIME_PUBLISH_FULL_PW_MODE_MS` (60000) không được dùng; `config.json` không có `time_publish` | Mặc định `.s_time_publish = TIME_PUBLISH_FULL_PW_MODE_MS`; thêm `"time_publish": 60` (giây) vào `config.json` | `app.c`, `config.json` | Đã giao; log cho thấy hoạt động theo chu kỳ |
| 4 | Muốn publish ngay khi vừa connect MQTT rồi mới theo `time_publish` | — | Cờ `first_pub_pending` (đặt trong `_on_connected` của app). Chờ `sim76xx.base.isBusy == 0` (lệnh subscribe có thể còn chạy) rồi gửi GSM+GPS một lần; cờ chỉ xóa khi publish được nhận | `app.c` | Đã giao; log cho thấy gửi ngay sau `subscribe OK` |
| 5 | Sau `Max retry — restart modem` vẫn không publish được, chỉ reset cả mạch mới được | `sx_mqtt_connect()` từ chối (`connect: already connected or in progress`) vì `s_mqtt.state` vẫn CONNECTED sau khi modem init lại; `sim76xx_start` chỉ chạy lại init AT, không power-cycle modem | Trong `_on_modem_ready`: nếu state khác DISCONNECTED thì đặt về DISCONNECTED rồi `sx_mqtt_connect`. Nhánh `Max retry`: đặt state DISCONNECTED và gọi `s_on_disconnected` trước `sim76xx_start`. Luồng `cb_start` đã có sẵn nhánh STOP+START khi `CMQTTSTART` lỗi | `mqtt_restart_fix/sx_user_mqtt.c` | **Đã giao, người dùng CHƯA nạp** |

### 11.2 Chẩn đoán đã thêm (cần xác nhận người dùng đã nạp)

- `sim76xx.c` (`urc_diag/`): lỗi `CMQTTTOPIC`/`CMQTTPAYLOAD` in `res=` (1 FAIL, 2 TIMEOUT) và phản hồi thô (**đã nạp**: log có `res=1 ... response=[...]`). Thêm log mọi dòng URC không mong đợi lúc modem rảnh (`URC: ...`): trước đó `sim76xx_poll` gom các dòng này vào `s_urc_buf` và chỉ xóa khi gặp `+CMQTTRXEND`, không bao giờ in. **Chưa rõ người dùng đã nạp phần URC này.**

### 11.3 Điều đã hiểu về luồng

- "Restart modem" sau publish fail = `sim76xx_start()` (gửi lại `AT`, `CGSN`, `CPIN`, `COPS`, `CSQ`, `CGATT`...). **Không power-cycle.** Power-cycle (`SIM Reset — power cycle`) chỉ khi `AT` lỗi liên tiếp 3 lần, hoặc SIM không ready sau 15 lần poll, hoặc `COPS=0` lỗi 3 lần.
- `modem_send_command` trả `-1` khi modem đang bận, nên publish trùng lúc modem bận sẽ bị bỏ (`publish not sent, dropped`).
- `SX_MQTT_TIMEOUT_PUB` = 3000 ms áp dụng cho từng bước publish (QoS 1, `CMQTTPUB=0,1,60,0`). `MQTT_KEEPALIVE` = 60 s (`app_config.h`).
- `sim76xx_psm_enable/disable` có trong code nhưng **không được gọi ở đâu**.
- Module dòng A76xx (URC `*ATREADY`, `*ISIMAID`, `+CCIOTOPTI`, `+CGEV`); model chính xác vẫn chưa xác nhận (`AT+CGMM`).

### 11.4 Vấn đề còn MỞ: publish lỗi theo chu kỳ, chưa rõ nguyên nhân gốc

Quan sát:
- SIM Viettel ổn. SIM Vina (APN `m3-world`, user/pass `mms`, có IP, RSSI 24–27) publish được 1–2 lần rồi lỗi.
- **`time_publish = 10` thì chạy liên tục; `= 60` thì lỗi** (người dùng quan sát). Nghĩa là lỗi liên quan khoảng rảnh giữa các lần gửi.
- Hai kiểu lỗi đã thấy: (a) `AT+CMQTTPUB` trả `OK` nhưng không có URC `+CMQTTPUB: 0,0` (timeout), sau đó banner `*ATREADY ... +CPIN: READY` xuất hiện **mà không có `SMS DONE`** (modem có vẻ reset); (b) `AT+CMQTTTOPIC` trả `ERROR` rõ ràng, không banner (modem sống, phiên MQTT mất).
- Khi `Max retry`, `COPS=0` ở lần init lại vẫn có thể trả `+CME ERROR` (đã xử lý bằng #1).

Giả thuyết (**tất cả chưa xác nhận**):
1. Timeout 3 s quá ngắn sau khi kênh vô tuyến rảnh (gói đầu cần resume radio, chờ PUBACK).
2. PSM/tự reset của modem khi rảnh (cài đặt PSM có thể được lưu trong modem). Banner `*ATREADY` là manh mối.
3. NAT/idle timeout của nhà mạng cắt phiên TCP (keepalive 60 s = đúng bằng chu kỳ publish).
4. Nguồn: IINDPM 500 mA + không có pin (mục 7a) có thể làm modem sụt áp khi phát sóng. Khó khớp với việc `time_publish = 10` ổn, nhưng chưa loại trừ.

Bước kiểm chứng đề xuất (chưa thực hiện):
- Nạp `mqtt_restart_fix/sx_user_mqtt.c` + `urc_diag/sim76xx.c`, chạy `time_publish: 60` đến khi lỗi, đọc các dòng `URC:` trước lỗi (`*ATREADY`, `+CPSMSTATUS`, `+CMQTTCONNLOST`...).
- Qua cổng AT (USB CDC, `TEST_AT_USB`): `AT+CPSMS?` (nếu `1` thì thử `AT+CPSMS=0`), `AT+CGMM`.
- Thử `SX_MQTT_TIMEOUT_PUB` = 10000 ms; thử giảm `MQTT_KEEPALIVE` xuống 20–30 s.
- Lắp pin Li-ion đã sạc, hoặc dùng adapter ≥ 2 A; đo VBAT/VSYS của modem lúc `CMQTTPUB` nếu có oscilloscope.
- Nếu muốn, thêm nhận biết `*ATREADY` lúc đang chạy = modem reset, rồi tự reconnect MQTT ngay thay vì chờ 3 lần timeout (chưa làm).

### 11.5 Việc nên làm tiếp theo (đề xuất thứ tự)

1. Người dùng nạp `mqtt_restart_fix/sx_user_mqtt.c`, `urc_diag/sim76xx.c` và gửi log lỗi (mục 11.4).
2. Xác nhận part number chip nguồn, rồi mới đối chiếu register map của đúng part (Phase 5/6 phụ thuộc điều này).
3. Quyết định xử lý nguồn/pin cho modem.
4. Quay lại Phase 1–6 (sleep/wake theo nhu cầu). Lưu ý các thay đổi ở mục 11 liên quan `sx_user_mqtt`/`sim76xx_start` nên được giữ nguyên khi sửa luồng sleep (đặc biệt: sau khi tắt SIM bằng PWRKEY phải reset `s_mqtt.state`, giống mục 5 phần WS_v1).