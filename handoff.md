# HANDOFF — Tracking FW v1.4 (BQ25622): sleep/wake theo nhu cầu, phát hiện USB, cấu hình sạc

Repo: https://github.com/ngoxuanloc2309/TF.git
MCU: STM32H563 (HAL, Makefile, arm-none-eabi-gcc). Framework nội bộ: `SynaptiX_FDK/`.
Tài liệu tham chiếu: datasheet BQ25620/BQ25622 (SLUSEG2D Rev. D). Số trang là số trang trong file PDF, có thể lệch vài trang so với số in ở chân trang.
Repo tham khảo (weather station, không có chân cắt nguồn nên tắt module bằng lệnh phần mềm): https://github.com/logan123synaptix/WS_v1.git

**Trạng thái (cập nhật 06/10/2026):** Phase 0 chạy được trên board. **Phase 1 (giờ mạng) xong và đã kiểm chứng trên board** (mục 6, mục 12). **Phase 2 đã code và test trên board phần lớn** (mục 12): cắt/bật nguồn flash và I2C1 đạt; **cắt nguồn IMU không dùng được** (kéo sập bus I2C1), thay bằng SUSPEND. Còn 2 việc kiểm tra chưa xong (mục 12.4). Phase 3–6 vẫn là kế hoạch, chưa có code. **Chip nguồn trên board không phải BQ25622 như giả định (mục 7a).** Chưa đo dòng ngủ, chưa test sleep thật trên board.

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
- `app/user/sx_ex_storage/` (không phải `services/`): `sx_storage_sleep()`/`sx_storage_wake()` **đã được cài ở Phase 2** (mục 12.2). Mọi hàm `sx_storage_*` tự bật flash nếu đang tắt.
- `components/sim76xx/sim76xx.c`: **đã có `AT+CTZU=1`/`AT+CCLK?`** (Phase 1) và `sim76xx_get_utc_now()`. `app.c` đã đồng bộ RTC ngoài từ giờ mạng và từ GPS (mục 12.1).
- Bug có sẵn: cắm USB trong lúc `ENTER_SLEEP` làm `sleep_requested` bị kẹt, lần rút sau `app_request_sleep()` bị bỏ qua. Cần reset cờ này khi xử lý cắm USB.

## 3. Chân cắt nguồn có sẵn (khác WS_v1)

Tracking FW có GPIO cắt nguồn cho từng module (khai báo trong `Core/Inc/main.h`, `board/sx_board.h`). Mức đã **đo trên board** cho flash: `PC4` **HIGH = rail tắt (0 V), LOW = bật (3.3 V)**. Các chân khác (`IMU_EN_PW`, ...) chưa đo riêng mức này; riêng IMU xem bên dưới.

| Module | Chân | Ghi chú |
|---|---|---|
| Exflash W25Q128 | `Flash_PWR` = PC4 (`SPI_PW_PIN`) | Cắt được, đã đo. `sx_W25Q128_power_down/up()` giờ cắt/cấp rail thật (chờ WIP trước khi cắt). SPI: CS = PA4, SCK/MISO/MOSI = PA5/6/7 |
| IMU BNO055 | `IMU_EN_PW` = PB4 | **KHÔNG được cắt nguồn**: đã đo, `IMU_EN_PW` HIGH thì `SCL`/`SDA` bị kéo về 0 và BQ (`0x6A`), RTC (`0x32`) không ACK nữa (IMU mất nguồn kẹp bus dùng chung). Dùng SUSPEND (`sx_board_imu_suspend/resume`). `IMU_RESET` = PB5 (cùng chân `I2C1_RESET`) |
| RTC ngoài RX8130CE | `RTC_EN_PW` = PB3 | **Không cắt** (cần giữ giờ, chưa xác nhận backup pin) |
| GPS | `GPS_PWR` = PC15 | đã dùng |
| LTE | `LTE_PWR` = PC13, `LTE_PWR_Key` = PC14 | đã dùng, **giữ nguyên phần SIM** |
| Sạc | `EN_CHARGE` = PC6, `EN_DISCHARGE` = PD12 | chưa xác nhận v1.4 còn hai chân này không |

I2C1: PB6/PB7 (BQ, RTC, IMU dùng chung). VBUS v1.2: PC1.

## 4. Thiết kế sleep/wake mới (đã thống nhất)

Nguyên tắc: **ngoại vi theo nhu cầu.** Khi sleep thì DeInit hết và cắt nguồn; mỗi lần thức chỉ bật đúng thứ tác vụ đó cần, làm xong thì DeInit/cắt lại.

**Vào sleep (một lần):**
- Wait busy flash, kéo `Flash_PWR` lên HIGH (`sx_storage_sleep()`). **Không kéo `IMU_EN_PW`**: đưa IMU vào SUSPEND (`sx_board_imu_suspend()`) trước khi DeInit I2C1.
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
- Flash: mặc định tắt khi ngủ, chỉ bật khi ghi/đọc log; `sx_storage_*` **đã** tự bật nguồn nếu đang tắt (`_ensure_power`) nên chỗ gọi (`write_gps_log`, `read_last_gps`...) không phải biết về nguồn. Cờ `sx_storage_hold_off(true)` chặn auto-wake (chỉ dùng cho test bằng AT).
- IMU: **SUSPEND** ở mọi chế độ ngủ, kể cả chu kỳ publish (không cắt nguồn, xem mục 3). Chỉ resume khi wake-real: `sx_board_imu_resume()` (PWR_MODE NORMAL rồi OPR_MODE NDOF, không reset). **Chưa xác nhận suspend có giữ calib không** (mục 12.4); nếu mất thì gọi `imu_calib_load()` sau resume (file `IMU_CALIB_FILE_PATH` nằm trong flash, `sx_storage_*` tự bật flash).
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
- (Tracking FW, đã gặp ở Phase 2) Cắt nguồn một chip nằm chung bus I2C có thể kéo sập cả bus (mục 3, IMU). Luôn đo `SCL`/`SDA` và đọc lại các slave khác ngay sau khi cắt, trước khi viết tiếp.
- (Tracking FW) `publish_gps()` gọi `read_last_gps()` mỗi chu kỳ publish khi chưa có fix, nên bất kỳ truy cập flash nào kể cả lúc đang `FULL_POWER` đều làm `_ensure_power` bật lại flash trong tối đa `time_publish` giây. Đo nguồn flash khi test phải dùng `AT+FLASHPWR=0` (có hold).

## 6. Giờ mạng (NITZ) cho RTC ngoài

- WS_v1 gửi `AT+CTZU=1`, đọc `AT+CCLK?`, parse `yy/MM/dd,hh:mm:ss±zz`, chuẩn hóa về UTC (`mktime`/`gmtime`), ghi vào RX8130CE (`time_sync.c`). Làm mỗi lần wake vì RTC ngoài lệch (số đo của người dùng: ~38 s sau vài chục phút; chưa kiểm chứng độc lập). Fallback dùng giờ GPS nếu modem không có giờ.
- Bộ AT manual dòng A76XX có `AT+CCLK`, `AT+CTZU`, `AT+CTZR`, `AT+CNTP` (NTP). A7680C thuộc dòng Cat 1 cùng hãng, **nhưng chưa có tài liệu nào xác nhận thẳng A7680C nằm trong đúng bộ manual đó**: cần kiểm chứng trên board.
- Rủi ro: NITZ phụ thuộc nhà mạng (nếu không gửi giờ thì `CCLK?` trả giờ mặc định chưa đồng bộ); `AT+CNTP` cần đã có kết nối dữ liệu và một NTP server.
- Kiểm chứng trước khi code (gửi tay, lúc SIM đã đăng ký mạng): `AT+CGMM`, `AT+CTZU=1`, `AT+CCLK?`.
- Thứ tự nguồn giờ đề xuất: NITZ ưu tiên, GPS dự phòng. Ghi RTC ngoài cần `acquire` I2C1. **Hiện code: nguồn nào đến sau thì ghi đè** (GPS chỉ ghi khi có fix); chưa chốt thứ tự ưu tiên.
- **Kết quả kiểm chứng trên board (06/10/2026, SIM `m3-world`):** `AT+CGMM` = `A7680C-LANS`; `AT+CTZU=1` OK; `AT+CCLK?` = `+CCLK: "26/10/05,16:20:01+28"` (zone 28 quý giờ = UTC+7, NITZ có giờ); parse đúng, RTC ngoài được ghi `16:20:01 05/10/2026`, các publish sau đó có `time`/`date` đúng. Với SIM/nhà mạng khác chưa test.

## 7a. Chip nguồn thực tế trên board (phát hiện khi test Phase 0)

- Scan I2C1 chỉ thấy `0x29` (BNO055), `0x32` (RX8130CE), `0x6A`. **Không có `0x6B`**, nên driver ở `0x6B` báo `not found`.
- Datasheet BQ25620/BQ25622 (SLUSEG2D): địa chỉ 7-bit `0x6B`. Tra datasheet thì **BQ25628/BQ25628E/BQ25629 dùng `0x6A`**, cùng dải thanh ghi 0x02–0x38. Nhiều khả năng đây là chip trên board, **nhưng chưa xác nhận** (cần mã in trên IC/schematic).
- Đã đổi driver sang `0x6A` (macro `BQ25622_I2C_ADDR7` trong `bq25622.h`, có thể override bằng `-D`). Log boot: `raw @0x6A: PART_INFO(0x38)=0x12 STATUS0(0x1D)=0x11 STATUS1(0x1E)=0x04`, driver báo `PN=2 (unknown) rev=2` (driver chỉ biết PN 0 = BQ25620, 1 = BQ25622). Ý nghĩa PN=2 **chưa đối chiếu datasheet**.
- **Chạy pin đã quan sát (06/10/2026):** log `BQ VBUS_STAT=000 CHG_STAT=0 -> on battery` và `initial VBUS_STAT=0 -> USB absent`; `STATUS0(0x1D)=0x01 STATUS1(0x1E)=0x00`. Chưa ghi lại được lúc chuyển 100→000 khi đang chạy (rút USB giữa chừng).
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
- **Phase 1 — SIM76xx có giờ mạng (ĐÃ XONG, đã test trên board):** `AT+CTZU=1`, `AT+CCLK?`, parse UTC, `sim76xx_get_utc_now()`, đồng bộ RTC ngoài trong `app.c`. Xem mục 6 và 12.1.
- **Phase 2 — Nguồn flash, IMU, I2C1 (ĐÃ CODE; test trên board gần xong):** cắt/bật flash đạt (`AT+FLASHTEST` 5 và 50 vòng, 0 lỗi); DeInit/Init I2C1 đạt; **cắt nguồn IMU thất bại (kẹp bus) nên đổi sang SUSPEND, đạt** (bus vẫn khỏe, BQ đọc được, resume `rc=0`). **Còn:** reset board kiểm tra filesystem còn nguyên và publish sau khi bật lại I2C (mục 12.4). Chi tiết mục 12.
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
9. **Trường `time` trong payload GPS lệch +7 giờ so với epoch UTC thật** (là giờ local UTC+7 đưa vào `mktime` rồi coi như UTC; ví dụ publish `1791217206` = 16:20:06 trong khi UTC thật là 09:20:06 = `1791192006`). Lỗi có từ trước. Cần quyết định với backend: giữ hay trừ `7*3600`. **Chưa chốt.**
10. **Ưu tiên giữa giờ mạng và GPS** khi hai nguồn lệch nhau (mục 6).
11. **BNO055 SUSPEND có giữ calib không và dòng thực tế khi suspend** (chưa đo).
12. **Đối chiếu schematic về CS (PA4) kéo LOW lúc flash mất nguồn** (code đang làm theo nguyên tắc không đẩy tín hiệu vào chip mất nguồn; chưa đối chiếu mạch).

## 10. Kế hoạch test trên board

- Rút USB khi đang chạy thì vào sleep.
- Cắm USB lúc đang ngủ thì wake trong vòng một chu kỳ wake-fake, chạy full power.
- Cắm/rút lúc đang `ENTER_SLEEP` và lúc đang tắt SIM.
- Boot chỉ bằng pin.
- I2C lỗi thì giữ trạng thái cũ, không tự vào sleep nhầm.
- ~~Cắt IMU rồi đọc BQ~~ (đã làm: thất bại, mục 3). Thay bằng: IMU suspend rồi đọc BQ/RTC (đạt). Cắt flash rồi bật lại đọc/ghi (đạt 50 vòng); còn kiểm tra filesystem còn nguyên sau reset.
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

---

## 12. Phiên 06/10/2026: giờ mạng (Phase 1) và nguồn flash/IMU/I2C1 (Phase 2)

Mọi file dưới đây là bản đầy đủ đã present; build bằng `arm-none-eabi-gcc` trong sandbox (qua), **test trên board do người dùng chạy**.

### 12.1 Giờ: lỗi tìm ra và sửa (`app.c`, `sim76xx.c/.h`)

Triệu chứng: publish ra `00:29:22 05/01/2000`. Nguyên nhân:
1. Không có nguồn nào ghi giờ vào RTC ngoài: giờ mạng có trong `sim76xx` nhưng `app.c` không đọc, và GPS chưa có fix.
2. Tháng lệch: driver RX8130CE yêu cầu `month` 1–12, code GPS truyền `tm_mon` 0–11 (tháng 1 bị từ chối, tháng khác lệch 1); phần hiển thị cộng `+1` để che.
3. `week` là mặt nạ bit một-bit (`RX8130CE_WEEK_xxx`), code cũ gán `mday/7+1`.
4. Hai chỗ trong `WAKE_PUBLISH` ghi RTC vô điều kiện từ `gps.tim`, mà `gps.tim` còn cũ sau một RMC không hợp lệ.

Cách sửa: RTC lưu giờ **UTC+7**, tháng 1–12. Thêm `rtc_set_local_secs()` (tính ngày/tháng/năm/thứ không phụ thuộc `mktime`), `rtc_sync_from_gps()` (chỉ ghi khi có fix) và `app_sync_rtc_from_modem()` (mỗi mẫu `CCLK` áp một lần, tối đa 3 lần thử). Thêm `clk_tick` và `sim76xx_get_utc_now()` để cộng thời gian trôi từ lúc đọc `CCLK`. Phần toán ngày giờ đã test riêng trên host so với `gmtime` (qua năm, năm nhuận, thứ, `mday = 32`).

### 12.2 Phase 2: các thay đổi trong code

| File | Thay đổi |
|---|---|
| `components/external_flash/sx_W25Q128.c/.h` | `power_down()` chờ WIP rồi mới cắt `Flash_PWR`; `power_up()` cấp rail + chờ 10 ms; thêm `sx_W25Q128_probe()` (thức từ power-down + kiểm JEDEC) và `sx_W25Q128_wait_idle()` |
| `app/user/sx_ex_storage/sx_ex_storage.c/.h` | `sx_storage_sleep()` (chờ WIP, cắt rail, `HAL_SPI_DeInit`, CS = LOW), `sx_storage_wake()` (rail, `HAL_SPI_Init`, CS = HIGH, probe; lỗi thì cắt rail lại), `sx_storage_is_powered()`, `sx_storage_hold_off()`; `_ensure_power()` trong mọi hàm file. Log từng bước qua cờ `STORAGE_PWR_DEBUG` (mặc định 1, tắt khi đo dòng) |
| `board/sx_board.c/.h` | `sx_board_imu_suspend/resume/is_active`, `sx_board_i2c1_off/on/is_on`, `sx_board_i2c1_scan()` (mức SCL/SDA + danh sách ACK) |
| `app/app.c/.h` | bỏ `static` của `imu_calib_load()` để dùng lại (định dạng calib dạng text có sẵn, chưa ai gọi) |
| `app/user/at_usb/test_at.c` | lệnh AT test thủ công (bảng dưới) |

Lệnh AT test (cổng CDC, kết thúc bằng CR/LF, board không echo; phản hồi chỉ hiện khi terminal bật DTR):

| Lệnh | Việc |
|---|---|
| `AT+FLASHPWR=0` / `=1` / `?` | tắt (có hold) / bật / hỏi trạng thái flash |
| `AT+FLASHTEST=N` | N vòng: tắt, ghi, tắt, đọc lại, so sánh (tối đa 100; chặn vòng lặp chính khi chạy) |
| `AT+FLASHPIN=0|1|?` | ghi/đọc thẳng `PC4`, không qua lớp storage (đo cực tính) |
| `AT+IMUPWR=0` / `=1` / `?` | IMU suspend / resume (kèm `calib sys/gyro/acc/mag`) / trạng thái |
| `AT+I2CPWR=0` / `=1` / `?` | DeInit / Init I2C1 |
| `AT+I2CSCAN` | mức `SCL`/`SDA` và các địa chỉ ACK |
| `AT+BQREAD` | đọc `VBUS_STAT`/`CHG_STAT` từ BQ |

### 12.3 Kết quả test trên board

| Hạng mục | Kết quả |
|---|---|
| `PC4` cực tính | `AT+FLASHPIN=1`: VCC flash = 0 V; `=0`: 3.3 V. **HIGH = tắt** |
| `AT+FLASHPWR=0` ban đầu vẫn đo 3.3 V | Do auto-wake (`publish_gps` → `read_last_gps`, mục 5), không phải lỗi chân. Sau khi thêm hold thì giữ tắt được; log `wake: rail on` → `SPI1 Init` → `probe` → `Flash powered on`, không có `JEDEC ID mismatch`. **Chưa ghi nhận số đo VCC flash khi hold** (người dùng báo `FLASHPIN? = 1` sau `FLASHPWR=0`) |
| `AT+FLASHTEST=5` và `=50` | `pass` đủ, `fail=0` |
| Cắt nguồn IMU (`IMU_EN_PW` HIGH) | `SCL=0 SDA=0 ACK: none`, `AT+BQREAD` lỗi `rc=-1`; bật lại IMU lỗi `rc=-1` (bus còn kẹt). Cần reset board để hồi phục |
| IMU SUSPEND | `SCL=1 SDA=1`, vẫn ACK `0x29 0x32 0x6A`, `AT+BQREAD` đọc được (`VBUS_STAT=4`), resume `active rc=0` |
| Sau resume | `calib sys=0 gyro=0 acc=0 mag=0`: **không kết luận được** (chưa biết calib trước khi suspend, board nằm yên) |
| `AT+I2CPWR=0` rồi `=1` | `BQREAD` báo `I2C1 is off`; sau khi bật lại đọc được, scan đủ 3 địa chỉ |

### 12.4 Việc còn lại để đóng Phase 2 (CHƯA kiểm chứng)

1. Reset board, đọc log boot: cần `Storage init OK`, không có dòng `format`, `config.json` còn nguyên (kiểm chứng filesystem sau các lần cắt nguồn flash).
2. Sau khi bật lại I2C1, chờ một lần publish xem `time`/`date` vẫn đúng (RTC không bị ảnh hưởng).
3. (Tùy chọn) Lắc board cho calib lên rồi suspend/resume để biết suspend có giữ calib không; đo dòng cả board khi flash tắt + IMU suspend so với lúc bật.

### 12.5 Quan sát khác trong log (chưa xử lý)

- Boot vẫn thấy `AT+COPS=0` trả `+CME ERROR: 100` kèm log `TIMEOUT response`, nhưng mạng vẫn lên và publish OK. Chưa xác nhận bản `fail_on_cme` (mục 11.1 #1) có đang chạy trên board không.
- `RMC sentence is not valid` in liên tục khi GPS chưa fix (trong nhà), bình thường nhưng làm đầy log.
- Dòng `LittleFS formatted and mounted successfully` chỉ là thông điệp log chung; code chỉ format khi `lfs_mount` lỗi.
- `READ_BAT` (ADC MCU) vẫn ~0.74 V, không dùng được (Phase 6 thay bằng VBAT từ BQ).

### 12.6 Phase 3 phải tính đến

- IMU suspend/resume thay cho cắt nguồn; `I2C1` phải luôn truy cập được BQ/RTC khi cần.
- Lớp `acquire/release` bọc `sx_board_i2c1_off/on`, `sx_storage_sleep/wake`, `sx_board_imu_suspend/resume`.
- Gỡ/tắt `STORAGE_PWR_DEBUG` và các lệnh `AT+FLASH*` khi đo dòng ngủ (UART log cũng bị DeInit lúc ngủ).