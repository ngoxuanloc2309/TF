# HANDOFF — Tracking FW v1.4 (BQ25622): sleep/wake theo nhu cầu, phát hiện USB, cấu hình sạc

Repo: https://github.com/ngoxuanloc2309/TF.git
MCU: STM32H563 (HAL, Makefile, arm-none-eabi-gcc). Framework nội bộ: `SynaptiX_FDK/`.
Tài liệu tham chiếu: datasheet BQ25620/BQ25622 (SLUSEG2D Rev. D). Số trang là số trang trong file PDF, có thể lệch vài trang so với số in ở chân trang.
Repo tham khảo (weather station, không có chân cắt nguồn nên tắt module bằng lệnh phần mềm): https://github.com/logan123synaptix/WS_v1.git

**Trạng thái (cập nhật 06/10/2026, cuối phiên Phase 3):** Phase 0 và Phase 1 xong. Phase 2 test gần xong (mục 12.4). **Phase 3 phần lõi đã code và chạy trên board** (mục 13): vòng wake-fake (ngủ STOP, mỗi `SX_TIME_WAKE_FAKE` thức dậy chỉ bật I2C1 đọc `VBUS_STAT`), phát hiện cắm USB lúc đang ngủ rồi wake-real, và USB (CDC + MSC) nhận lại được trên máy tính nhờ re-enumerate. Người dùng báo rút/cắm USB detect mượt. **Đã bỏ hẳn đọc pin bằng ADC MCU và ngắt/đọc chân VBUS (PC1/EXTI)** khỏi board và app. **Chưa làm / chưa kiểm chứng của Phase 3:** `time_check_vbus` trong `config.json`, DeInit UART lúc ngủ, lớp `acquire/release`, đo dòng ngủ, nhiều chu kỳ publish liên tiếp, các ca biên (mục 13.5). Phase 4 một phần có sẵn trong luồng hiện tại; Phase 5–6 chưa có code. **Chip nguồn trên board không phải BQ25622 như giả định (mục 7a).** **Firmware chưa ghi bất kỳ thanh ghi cấu hình nào xuống BQ (mục 13.6).**

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

Hệ quả: **không có nguồn wake tức thì khi cắm USB.** Chỉ làm được bằng polling định kỳ (RTC wakeup ngắn). Đổi PB6/PB7 sang GPIO vô ích vì BQ là I2C slave, không tự tạo cạnh trên SCL/SDA. Không nối INT vào SDA/SCL. **Hệ quả đã gặp thật:** cắm USB lúc MCU đang STOP thì host Windows enumerate không được trả lời nên báo "unknown USB device"; đã xử lý bằng re-enumerate khi wake-real (mục 13.3).

## 2. Hiện trạng code (sau Phase 3, 06/10/2026)

Luồng (`SynaptiX_FDK/app/app.c`):
- **`FULL_POWER`:** có USB, chạy bình thường.
- Rút USB: `bq25622_poll` (debounce `VBUS_STAT`) thấy `VBUS_STAT=0` → `app_request_sleep()`. Chuyển sang **`ENTER_SLEEP`**: vẫn chạy GPS/MQTT, publish "enter sleep" (GSM + GPS). Thoát khi publish xong, hoặc MQTT mất quá 5 s, hoặc quá `ENTER_SLEEP_TIMEOUT_MS`. Nếu USB cắm lại trong lúc này thì quay về `FULL_POWER`. **`ENTER_SLEEP_TIMEOUT_MS` hiện = 1000 ms nên log thường thấy `Enter sleep — timeout`, tức bản tin "enter sleep" gần như không kịp gửi** (mục 9 #14).
- **`SLEEP`:** `sx_user_mqtt_force_disconnect()`, `queue_flush()`, rồi `sx_sleep_manager_enter()` (`services/sleepmanager/`): tắt GPS, tắt SIM (`sim76xx_power_off_blocking`), `sx_storage_sleep()` (cắt flash), `sx_board_imu_suspend()`, rồi **vòng wake-fake** (mục 13.2).
- Kết quả của `sx_sleep_manager_enter()` là `wake_reason`: **`WAKE_REASON_RTC`** (hết `time_sleeps`) → **`WAKE_PUBLISH`**; **`WAKE_REASON_VBUS`** (thấy USB trong wake-fake) → `app_notify_usb_connected()` → xử lý `usb_connect_pending` (log `=== USB connected — restarting ===`): re-enumerate USB rồi về `FULL_POWER`.
- **`WAKE_PUBLISH`:** bật GPS, chờ fix (tối đa `GPS_TIMEOUT_MS` = 130 s), bật SIM và chờ sẵn sàng (tối đa 90 s), kết nối MQTT, publish "wake up", ghi log GPS (`write_gps_log`), cập nhật giờ RTC ngoài từ GPS, rồi quay lại `SLEEP`. Cả chuỗi bị giới hạn bởi `SX_TIME_IN_WAKE` (160 s). Khi vào `WAKE_PUBLISH`, I2C1 đang **bật** (cần cho RTC ngoài), IMU vẫn SUSPEND, flash tự bật khi `sx_storage_*` được gọi.
- Chu kỳ ngủ (publish): `SX_TIME_IN_SLEEP` = 60000 ms mặc định (`app_config.h`), ghi đè bằng `time_sleeps` (giây) trong `config.json`.
- Chu kỳ wake-fake: macro **`SX_TIME_WAKE_FAKE`** (`app_config.h`, mặc định 10000 ms). Trường `check_ms` trong `sx_sleep_manager_t` (0 = dùng macro) để dành cho `time_check_vbus`; **chưa đọc từ `config.json`**.

`_enter_stop()` (`components/sleep/sx_sleep.c`) abort UART1/UART2, tắt NVIC USB, dừng SysTick, WFI; sau wake gọi `SX_RESUME_TICS()` rồi `SystemClock_Config()` (**thứ tự đã đúng**, handoff cũ ghi sai). **Chưa DeInit UART/SPI** lúc ngủ (UART log vẫn sống), nên dòng ngủ chưa phải thấp nhất.

Các thành phần khác:
- `components/bq25622/`: driver đọc (`bq25622_init/poll/refresh/read_status`) đã vào build và chạy ở `0x6A`. Có sẵn `bq25622_config_apply()` và `BQ25622_CFG_DEFAULT` (4.2 V, 480 mA, ITERM 60 mA, cutoff 2.9 V) nhưng **không ai gọi** (mục 13.6).
- `sx_board.c`: `check_charge()` chỉ theo `board.bq.present` (không còn đọc `VBUS_PIN`). **Đã xóa:** `HAL_GPIO_EXTI_*_Callback`, `VBUS_PORT/VBUS_PIN`, macro `SX_VBUS_FROM_BQ`, `sx_sleep_set_exti_wake()`, toàn bộ đọc ADC (`read_vol_pin`, `s_adc_reader`, `raw_adc`, `v_adc`, `hal_adc`, `TIME_READ_PIN`). `sx_board_init()` vẫn `HAL_NVIC_DisableIRQ(EXTI1_IRQn)` + `HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1)` vì CubeMX (`gpio.c`) còn cấu hình PC1 là EXTI, chân thả nổi sẽ wake MCU khỏi STOP.
- `board.voltage.v_bat` (float, V) còn lại và **luôn = 0.0** (payload GSM có `"vbat":0.000000`) cho tới khi làm Phase 6 (VBAT từ ADC của BQ). Còn sót (chưa xóa, không ai gọi): `services/read_bat/` (trong `synaptix.mk` và `-I` của `Makefile`), `Core/Src/adc.c` và `MX_ADC1_Init()` (CubeMX), `EXTI1_IRQHandler` và cấu hình PC1 (CubeMX).
- `app/user/sx_ex_storage/` (không phải `services/`): `sx_storage_sleep()`/`sx_storage_wake()` đã cài ở Phase 2 (mục 12.2). Mọi hàm `sx_storage_*` tự bật flash nếu đang tắt. `NO DATA FROM EXFLASH!` trong `read_last_gps()` chỉ nghĩa là file log GPS rỗng/chưa có (`sx_storage_size() <= 0`), thường do chưa từng có fix; payload khi đó là `lat:0, fix:0`.
- `components/sim76xx/sim76xx.c`: đã có `AT+CTZU=1`/`AT+CCLK?` (Phase 1) và `sim76xx_get_utc_now()`. `app.c` đã đồng bộ RTC ngoài từ giờ mạng và GPS (mục 12.1).
- Publish MQTT là hàng đợi: `sx_user_mqtt_publish()` gửi ngay tin đầu nếu rảnh, các tin sau chờ trong queue và được gửi lần lượt khi callback `publish OK` của tin trước tới (QoS 1). Vì vậy trong log `GSM:` rồi `GPS:` rồi hai dòng `MQTT publish OK` là bình thường (dòng OK đầu là của GSM).
- Bug cũ `sleep_requested` bị kẹt khi cắm USB lúc `ENTER_SLEEP`: các chỗ xử lý cắm USB trong `app.c` đã reset cờ về 0 (đọc code), **nhưng chưa test trên board**.

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

**Trạng thái cài đặt của thiết kế này (06/10/2026):**

| Hạng mục | Trạng thái |
|---|---|
| Wake-fake chỉ bật I2C1, đọc `VBUS_STAT` | **Đã cài, chạy trên board** (mục 13.2) |
| Tắt GPS/SIM, cắt flash, IMU SUSPEND trước khi ngủ | Đã cài (`sx_sleep_manager_enter`) |
| Wake-real: resume IMU, về `FULL_POWER`, re-enumerate USB | Đã cài, USB nhận lại được |
| Đếm chu kỳ publish theo RTC | Đã cài (lịch RTC trong chip, lấy max với tổng các chu kỳ danh nghĩa) |
| `time_check_vbus` trong `config.json` | **Chưa** (đang dùng macro `SX_TIME_WAKE_FAKE`) |
| DeInit SPI/I2C/UART (UART log sau cùng), PA4/PB5 về LOW/analog | **Chưa** (chỉ DeInit I2C1 trong vòng wake-fake và SPI trong `sx_storage_sleep`) |
| Lớp `acquire/release`, `release_all()` | **Chưa** |
| Cờ giữ UART log khi debug | Có `SX_WAKE_FAKE_LOG` (mặc định 1) cho log trong vòng wake-fake |
| Gọi `imu_calib_load()` sau resume | **Chưa** (chưa biết suspend có mất calib không) |

## 5. Bài học từ WS_v1 (đã gặp lỗi thật ở đó)

- Trước khi cắt/đưa flash vào power-down phải `w25q_wait_busy()`. Làm ngay sau ghi/erase thì treo bus. Cắt nguồn giữa lúc ghi còn có thể hỏng dữ liệu.
- Chỉ cut clock (`__HAL_RCC_xxx_CLK_DISABLE`) không đủ; phải `HAL_xxx_DeInit()` (MspDeInit đưa pin về analog) thì dòng mới giảm rõ. I2C có pull-up ngoài, nếu pin để AF_OD mà bus đang bị kéo thấp thì rò suốt STOP. TIM/LPTIM phải dùng `DeInit`, không dùng CLK_DISABLE thô, nếu không sau wake `MX_*_Init()` bỏ qua MspInit và treo ở `Error_Handler()`.
- `SX_RESUME_TICS()` (`HAL_ResumeTick`) phải gọi **trước** `SystemClock_Config()`, vì `HAL_RCC_OscConfig()` chờ HSE bằng `HAL_GetTick()`; tick còn tắt thì treo vĩnh viễn. Tracking FW **đã** gọi `SX_RESUME_TICS()` trước `SystemClock_Config()` (`sx_sleep.c`, đã kiểm tra lại 06/10/2026).
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
- Thứ tự nguồn giờ đề xuất: NITZ ưu tiên, GPS dự phòng. Ghi RTC ngoài cần `acquire` I2C1. ****ĐÃ CHỐT (người dùng, 06/10/2026): giờ mạng (SIM) ưu tiên hơn GPS.** Code (`app.c`): `rtc_sync_from_gps()` chỉ ghi RTC khi phiên modem hiện tại chưa áp được giờ mạng (`rtc_net_time_current()`, so `sim76xx.clk_utc` với `s_net_ok_utc`); mỗi mẫu `CCLK` mới luôn ghi đè giờ GPS đã ghi trước đó. `clk_valid` bị xóa mỗi lần modem khởi động lại nên mỗi chu kỳ publish bắt đầu lại từ "chưa áp". GPS là dự phòng (SIM không có NITZ, hoặc ghi RTC từ giờ mạng lỗi 3 lần). **Hệ quả cần biết:** giờ mạng chỉ lấy một lần mỗi phiên modem, nên khi `FULL_POWER` chạy lâu thì RTC có thể trôi dần mà GPS không còn hiệu chỉnh (số đo của người dùng ~38 s sau vài chục phút, chưa kiểm chứng độc lập). Đã build qua, **chưa test trên board**.
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
- **Phase 3 — Quản lý ngoại vi theo nhu cầu + vòng wake-fake (PHẦN LÕI ĐÃ XONG, đã chạy trên board):** vòng STOP → I2C1 → đọc `VBUS_STAT` → wake-real hoặc ngủ tiếp, đếm chu kỳ publish theo RTC, re-enumerate USB khi wake-real, bỏ ADC và EXTI. **Còn:** `time_check_vbus` từ `config.json`, DeInit UART/SPI lúc ngủ + PA4/PB5, lớp `acquire/release`, đo dòng ngủ, test nhiều chu kỳ và các ca biên (mục 13.5).
- **Phase 4 — Chu kỳ publish theo nhu cầu:** GPS → last_gps từ flash → ghi flash → SIM publish → cập nhật giờ RTC → release hết. Đếm chu kỳ theo RTC.
- **Phase 5 — Cấu hình sạc BQ:** `reg_write8/16`, `bq25622_config_apply()` (ICHG/VREG/IINDPM/ITERM, tắt watchdog, cấu hình ADC), verify-and-apply lúc boot và mỗi lần phát hiện cắm USB. Cần thông số cell.
- **Phase 6 — VBAT và cắt xả:** VBAT one-shot từ ADC BQ có timeout + kiểm tra `ADC_DONE_STAT`, điền `board.voltage.v_bat` (hiện luôn 0.0, mục 2). `services/read_bat` đã hết người gọi (xóa thư mục + dòng trong `synaptix.mk`/`Makefile` nếu muốn). Đo VBAT khi SIM/GPS tắt. Khi dưới ngưỡng thì tắt tải và shutdown qua `BATFET_CTRL`.

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
9. **Múi giờ: ĐÃ CHỐT (người dùng, 06/10/2026):** dùng giờ Việt Nam (UTC+7), RTC lưu giờ local, trường `time` trong payload là giờ local, nên múi giờ hiện tại là đúng. (Trường `time` là giờ local đưa vào `mktime` rồi coi như UTC nên không phải epoch UTC thật; đây là chủ đích.)
10. ~~Ưu tiên giữa giờ mạng và GPS~~ **ĐÃ CHỐT 06/10/2026: giờ mạng (SIM) ưu tiên, GPS dự phòng** (mục 6). Còn mở: có cần đọc lại `CCLK` định kỳ khi `FULL_POWER` chạy lâu để RTC không trôi không.
11. **BNO055 SUSPEND có giữ calib không và dòng thực tế khi suspend** (chưa đo).
12. **Đối chiếu schematic về CS (PA4) kéo LOW lúc flash mất nguồn** (code đang làm theo nguyên tắc không đẩy tín hiệu vào chip mất nguồn; chưa đối chiếu mạch).
13. **Dòng sạc BQ:** firmware chưa ghi cấu hình nào nên chip chạy mặc định (theo datasheet BQ25620/22: ICHG 1040 mA, IINDPM 3.2 A, VREG 4.2 V; **chưa đọc lại từ chip, chưa xác nhận part**). Cần thông số cell và part number trước khi chốt ICHG/IINDPM (mục 13.6).
14. **`ENTER_SLEEP_TIMEOUT_MS` = 1000 ms** có khiến bản tin "enter sleep" không bao giờ kịp gửi: cố ý bỏ qua hay cần tăng?
15. **`"vbat":0.000000` trong payload GSM:** backend có chấp nhận không, hay cần bỏ trường này khỏi payload.
16. **Topic GSM không thấy trên MQTT Explorer** (06/10/2026): log thiết bị cho thấy cả GSM và GPS đều `MQTT publish OK` (QoS 1, có PUBACK), nên nghi phía broker/Explorer (subscribe `vindynamic/tracking/#` đúng lúc, retain=0, ACL, rule backend). **Chưa xác nhận bằng `mosquitto_sub`.**
17. **IMU resume xong có cần `imu_calib_load()` không** (mục 12.4 #3, vẫn chưa kiểm chứng).

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

**Đã đạt trên board (06/10/2026):** rút USB → vào sleep; vòng wake-fake chạy; cắm USB lúc đang ngủ → wake-real sau 20–30 s kể từ lúc vào sleep; sau re-enumerate máy tính nhận lại CDC + MSC. **Chưa test:** nhiều chu kỳ publish liên tiếp, cắm USB lúc `ENTER_SLEEP`/`WAKE_PUBLISH`, boot chỉ bằng pin, rút rồi cắm ngay (bug `sleep_requested`), đo dòng ngủ.
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
- (Đã xử lý ở mục 13.1) `READ_BAT` ADC MCU đã bị xóa khỏi board và app; `v_bat` giờ luôn 0.0 cho tới Phase 6.

### 12.6 Phase 3 phải tính đến

- IMU suspend/resume thay cho cắt nguồn; `I2C1` phải luôn truy cập được BQ/RTC khi cần.
- Lớp `acquire/release` bọc `sx_board_i2c1_off/on`, `sx_storage_sleep/wake`, `sx_board_imu_suspend/resume`.
- Gỡ/tắt `STORAGE_PWR_DEBUG` và các lệnh `AT+FLASH*` khi đo dòng ngủ (UART log cũng bị DeInit lúc ngủ).

---

## 13. Phiên 06/10/2026: Phase 3 (wake-fake), bỏ ADC/EXTI, USB re-enumerate

Mọi file dưới đây là bản đầy đủ đã present. Build bằng `arm-none-eabi-gcc` trong sandbox (qua; muốn build cần `git submodule update --init SynaptiX_FDK/lib/tinyusb` và cài `gcc-arm-none-eabi`); **test trên board do người dùng chạy**. Các file đã đổi: `services/sleepmanager/sx_sleep_manager.c/.h`, `components/sleep/sx_sleep.c/.h`, `board/sx_board.c/.h`, `app/app.c`, `app/app_config.h`.

### 13.1 Thay đổi

| Việc | Chi tiết |
|---|---|
| Vòng wake-fake | `sx_sleep_manager_enter()` viết lại (mục 13.2) |
| Bỏ ADC pin | Xóa `read_vol_pin()`, `s_adc_reader`, `raw_adc/v_adc`, `hal_adc`, `HAL_ADC_Start/Calibration`, `#include adc.h/sx_read_bat.h` trong board; xóa lời gọi `read_vol_pin()` trong `app.c`; xóa `TIME_READ_PIN`. Giữ `voltage_t.v_bat` (V) = 0.0 để payload không đổi định dạng |
| Bỏ GPIO/EXTI VBUS | Xóa `HAL_GPIO_EXTI_Rising/Falling_Callback`, nhánh `HAL_GPIO_ReadPin(VBUS_PIN)`, `VBUS_PORT/VBUS_PIN`, `SX_VBUS_FROM_BQ`, `sx_sleep_set_exti_wake()`; `tud_umount_cb` không còn `app_request_sleep()` |
| Đổi tên | `WAKE_REASON_EXTI` → **`WAKE_REASON_VBUS`** (`sx_sleep.h`, `sx_sleep_manager.c`, `app.c`); log "Woke by VBUS (BQ check)" |
| Macro chu kỳ | `SX_TIME_WAKE_FAKE` = 10000 ms trong `app_config.h` (thay cho macro tạm `SX_TIME_CHECK_VBUS_MS` trong `sx_sleep_manager.h`) |
| USB | re-enumerate khi wake-real (mục 13.3) |

### 13.2 Vòng wake-fake (`sx_sleep_manager_enter`)

1. Tắt GPS, SIM; `sx_storage_sleep()`; `sx_board_imu_suspend()` (khi I2C1 còn bật).
2. Lặp: `sx_board_i2c1_off()` → đặt RTC wakeup `min(check, còn lại)` giây → STOP → hủy RTC wakeup.
3. Thời gian đã trôi = `max(lịch RTC trong chip, tổng các chu kỳ RTC hoàn tất)`. Chỉ cộng chu kỳ khi `wake_reason == WAKE_REASON_RTC` (wake lạ thì lặp lại).
4. Đủ `sleep_ms` → `i2c1_on`, `wake_reason = WAKE_REASON_RTC` (sang `WAKE_PUBLISH`).
5. Chưa đủ → `i2c1_on` (lỗi thì tiếp tục ngủ, **không bao giờ coi lỗi I2C là có USB**) → `bq25622_refresh()` đọc `VBUS_STAT` trực tiếp, không qua debounce → có USB thì `sx_board_imu_resume()`, `wake_reason = WAKE_REASON_VBUS`; không thì lặp.
6. Wake-fake chỉ bật I2C1; SIM/GPS/flash/IMU/USB chỉ bật ở wake-real hoặc chu kỳ publish.

Log mẫu: `wake-fake: VBUS_STAT=4 -> USB present, wake-real`, `<<< Woke from STOP mode (VBUS after 30 s)`, `Woke by VBUS (BQ check)`.

### 13.3 Bài học: USB phải re-enumerate sau khi thức vì cắm USB

- Triệu chứng: wake-real chạy đúng nhưng máy tính báo "unknown USB device"; chỉ nhận lại khi reset board.
- Nguyên nhân (suy luận, phù hợp log): host bắt đầu enumerate ngay lúc cắm, MCU đang STOP (NVIC USB tắt, clock dừng) nên không trả lời, host bỏ cuộc. Firmware thức dậy sau tối đa một chu kỳ wake-fake.
- **Không dùng `tud_mounted()` làm điều kiện.** Log cho thấy `tud_mounted=1 tud_connected=1` ngay cả khi host đã bỏ: stack không biết đã rút cáp (không có VBUS sensing, IRQ USB tắt trong STOP), còn trạng thái cấu hình cũ (và log `USB tiny resumed`).
- Cách sửa (đã chạy được): luôn `sx_usb_tiny_msc_disconnect()` → `sx_delay_ms(500)` → `sx_usb_tiny_msc_connect()` khi xử lý `usb_connect_pending`, kèm log `USB re-enumerate (tud_mounted=.. tud_connected=..)`. Sau đó `USB tiny connected` và `CDC line state` xuất hiện, máy tính nhận lại. Cùng kiểu với `_remount()` trong `sx_user_msc.c`.
- Độ trễ nhận USB tối đa bằng `SX_TIME_WAKE_FAKE` và mỗi lần cắm có một lần "nháy" kết nối: là giới hạn của thiết kế polling.

### 13.4 Điều đã xác nhận / chưa

- **Đã xác nhận trên board:** rút USB vào sleep; vòng wake-fake chạy và đọc được BQ sau khi bật lại I2C1; cắm USB lúc ngủ thì wake-real; USB nhận lại sau re-enumerate (người dùng báo "detect rất mượt"). Chưa ghi lại chính xác khoảng cách giữa các lần wake-fake.
- **Chưa kiểm chứng:** IMU có giữ calib sau suspend không; dòng ngủ thực tế (con số ~5 µA chỉ là ước tính); filesystem sau nhiều chu kỳ cắt nguồn flash (mục 12.4); `time`/`date` sau nhiều lần bật/tắt I2C1.

### 13.5 Việc còn lại của Phase 3 (đề xuất thứ tự)

1. Test chu kỳ publish liên tiếp (≥ 3 vòng STOP → wake-fake → RTC wake → `WAKE_PUBLISH` → sleep) không treo, chu kỳ không dài dần, payload `time`/`date` đúng.
2. Ca biên: cắm USB lúc `ENTER_SLEEP`; cắm USB lúc `WAKE_PUBLISH` (cần thấy `USB plugged during WAKE_PUBLISH - full restart`); boot chỉ bằng pin; rút rồi cắm lại ngay.
3. Đóng 2 việc của Phase 2 (mục 12.4).
4. `time_check_vbus` từ `config.json` → gán `check_ms` của sleep manager.
5. DeInit UART/SPI/I2C lúc ngủ (UART log sau cùng), PA4/PB5 về LOW/analog; đặt `SX_WAKE_FAKE_LOG=0`, `STORAGE_PWR_DEBUG=0` rồi đo dòng ngủ thật.
6. Tùy chọn: lớp `acquire/release`; đổi `imu_calib_load()` sau resume nếu mất calib.

### 13.6 Dòng sạc BQ hiện tại

- Không có chỗ nào gọi `bq25622_config_apply()` (chỉ `bq25622_init()` ở `sx_board.c` và poll/refresh để **đọc**). Chưa ghi thanh ghi nào nên chip ở default mode với giá trị mặc định theo datasheet BQ25620/22 (ICHG 1040 mA, VREG 4.2 V, IINDPM 3.2 A) — **theo datasheet, chưa đọc lại từ chip, chưa xác nhận part**.
- Giới hạn thực tế khi cắm USB: `VBUS_STAT=100b` tương ứng IINDPM 500 mA theo bảng 8-2 BQ25629 (chưa rõ áp dụng cho chip trên board).
- `BQ25622_CFG_DEFAULT` trong `bq25622.h` (4.2 V, 480 mA, ITERM 60 mA, IINDPM KEEP, cutoff 2.9 V) chỉ là giá trị dự kiến, chưa áp lên chip.
- **Không ghi thanh ghi BQ cho đến khi xác nhận part (mục 7a):** ghi bất kỳ thanh ghi nào đưa chip sang host mode + watchdog; watchdog hết thì ICHG giảm một nửa. Có thể thêm lệnh AT chỉ-đọc in `ICHG/IINDPM/VREG` để biết giá trị thật (chưa làm).

### 13.7 Quan sát khác

- `RMC sentence is not valid` in liên tục khi GPS chưa fix: bình thường nhưng làm đầy log.