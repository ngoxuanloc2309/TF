APP_FILES = ${SYNAPTIX_DIR}/app/app.c \
			${SYNAPTIX_DIR}/app/user/sx_cdc/sx_user_cdc.c \
			${SYNAPTIX_DIR}/app/user/sx_msc/sx_user_msc.c \
			${SYNAPTIX_DIR}/app/user/sx_mqtt/sx_user_mqtt.c \
			${SYNAPTIX_DIR}/app/user/sx_ex_storage/sx_ex_storage.c \
			${SYNAPTIX_DIR}/app/user/at_usb/test_at.c \
			${SYNAPTIX_DIR}/app/user/at_usb/at_command.c \
			

BOARD_FILES = ${SYNAPTIX_DIR}/board/sx_board.c

COMPONENT_FILES = ${SYNAPTIX_DIR}/components/can/sx_can.c \
				  ${SYNAPTIX_DIR}/components/flash/sx_flash.c \
				  ${SYNAPTIX_DIR}/components/gpio/sx_gpio.c \
				  ${SYNAPTIX_DIR}/components/i2c/sx_i2c.c \
				  ${SYNAPTIX_DIR}/components/spi/sx_spi.c \
				  ${SYNAPTIX_DIR}/components/uart/sx_uart.c \
				  ${SYNAPTIX_DIR}/components/timer/sx_timer.c \
				  ${SYNAPTIX_DIR}/components/gps/gps.c \
				  ${SYNAPTIX_DIR}/components/gps/minmea.c \
				  ${SYNAPTIX_DIR}/components/modem/modem.c \
				  ${SYNAPTIX_DIR}/components/sim76xx/sim76xx.c \
				  ${SYNAPTIX_DIR}/components/usb_cdc_tiny/sx_usb_tiny_cdc.c \
				  ${SYNAPTIX_DIR}/components/usb_cdc_tiny/usb_descriptor.c \
				  ${SYNAPTIX_DIR}/components/usb_msc_tiny/sx_usb_tiny_msc.c \
				  ${SYNAPTIX_DIR}/components/external_flash/sx_W25Q128.c\
				  $(SYNAPTIX_DIR)/components/sleep/sx_sleep.c \
				  $(SYNAPTIX_DIR)/components/imu/bno055.c \
				  $(SYNAPTIX_DIR)/components/rtc/sx_ex_rtc.c \

SERVICES_FILES = ${SYNAPTIX_DIR}/services/logger/logger.c \
				 ${SYNAPTIX_DIR}/services/littlefs/lfs.c \
				 ${SYNAPTIX_DIR}/services/littlefs/lfs_util.c \
				 ${SYNAPTIX_DIR}/services/cJSON/cJSON.c \
				 ${SYNAPTIX_DIR}/services/cJSON/cJSON_Utils.c \
				 ${SYNAPTIX_DIR}/services/OS/sx_os.c \
				 ${SYNAPTIX_DIR}/services/filesystem/file_io.c \
				 ${SYNAPTIX_DIR}/services/shell/cli_shell.c \
				 ${SYNAPTIX_DIR}/services/filesystem/sx_fs.c \
				 ${SYNAPTIX_DIR}/services/fatfs/ff.c \
				 ${SYNAPTIX_DIR}/services/fatfs/ffsystem.c \
				 ${SYNAPTIX_DIR}/services/fatfs/ffunicode.c \
				 ${SYNAPTIX_DIR}/services/sx_fatfs/sx_diskio.c \
				 ${SYNAPTIX_DIR}/services/sx_fatfs/sx_fatfs.c \
				 ${SYNAPTIX_DIR}/services/mqtt/sx_mqtt.c \
				 ${SYNAPTIX_DIR}/services/sleepmanager/sx_sleep_manager.c \
				 ${SYNAPTIX_DIR}/services/read_bat/sx_read_bat.c \

UTILS_FILES = ${SYNAPTIX_DIR}/utils/cqueue/cqueue.c \
			  ${SYNAPTIX_DIR}/utils/mem/sx_mem.c \
			  ${SYNAPTIX_DIR}/utils/delay/sx_delay.c \
			  ${SYNAPTIX_DIR}/utils/filter/sx_filter.c \


SYNAPTIX_SOURCES = $(APP_FILES) \
				   $(BOARD_FILES) \
				   $(SERVICES_FILES) \
				   $(UTILS_FILES) \
				   $(COMPONENT_FILES)
