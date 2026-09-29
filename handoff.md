# HANDOFF — Tracking FW v1.4 (BQ25622): sleep/wake theo nhu cầu, phát hiện USB, cấu hình sạc

Repo: https://github.com/ngoxuanloc2309/TF.git
MCU: STM32H563 (HAL, Makefile, arm-none-eabi-gcc). Framework nội bộ: `SynaptiX_FDK/`.
Tài liệu tham chiếu: datasheet BQ25620/BQ25622 (SLUSEG2D Rev. D). Số trang là số trang trong file PDF, có thể lệch vài trang so với số in ở chân trang.
Repo tham khảo (weather station, không có chân cắt nguồn nên tắt module bằng lệnh phần mềm): https://github.com/logan123synaptix/WS_v1.git

**Trạng thái: mới thiết kế, chưa có dòng code v1.4 nào. Chưa test trên board.** Không có file patch nào; mọi thứ bên dưới là kế hoạch.

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

- **Phase 0 — Đọc BQ25622:** thêm `bq25622.c` vào build, gọi `bq25622_init()` sau I2C1, in `VBUS_STAT` trong main loop. Test: cắm/rút USB thì log đổi `000` ↔ `100`.
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