PROJECT_NAME     := acconeer_demo_pca10056_s140
TARGETS          := \
                    example_assembly_test \
                    example_bring_up \
                    example_battery_info \
                    example_detector_distance \
                    example_detector_distance_recorded \
                    example_detector_presence \
                    example_get_next_by_reference \
                    example_low_power_envelope \
                    example_low_power_sparse \
                    example_multiple_service_usage \
                    example_error_handling \
                    example_service_envelope \
                    example_service_iq \
                    example_service_power_bins \
                    example_service_sparse \
                    ref_app_rf_certification_test \
                    ref_app_parking \
                    ref_app_smart_presence \
                    ref_app_tank_level \
                    ble_monitor_presence

OUTPUT_DIRECTORY := out

ifeq ($(NRF_SDK_ROOT),)
$(error "Make sure to set the NRF_SDK_ROOT. E.g. 'export NRF_SDK_ROOT=~/nRF5_SDK_15.3.0_59ac345/'")
endif

SDK_ROOT := $(NRF_SDK_ROOT)

LINKER_SCRIPT := acconeer_demo.ld

ACCONEER_ZIP_ROOT := .

example_battery_info: CFLAGS+=-DBROADCAST_BATTERY_INFO

ble_monitor_presence: CFLAGS+=-DUSE_BLE_MONITOR_ADV_FORMAT

# Source files for target example_battery_info
SRC_FILES_EXAMPLE_BATTERY_INFO := \
  $(ACCONEER_ZIP_ROOT)/source/acc_battery_info_xm122.c \
  $(ACCONEER_ZIP_ROOT)/source/acc_bluetooth_beacon_xm122.c \
  $(ACCONEER_ZIP_ROOT)/source/example_low_power_envelope.c

# Source files for target example_low_power_envelope
SRC_FILES_EXAMPLE_LOW_POWER_ENVELOPE := \
  $(ACCONEER_ZIP_ROOT)/source/example_low_power_envelope.c

# Source files for target example_low_power_sparse
SRC_FILES_EXAMPLE_LOW_POWER_SPARSE := \
  $(ACCONEER_ZIP_ROOT)/source/example_low_power_sparse.c

# Source files for target example_assembly_test
SRC_FILES_EXAMPLE_ASSEMBLY_TEST := \
  $(ACCONEER_ZIP_ROOT)/source/example_assembly_test.c

# Source files for target example_bring_up
SRC_FILES_EXAMPLE_BRING_UP := \
  $(ACCONEER_ZIP_ROOT)/source/example_bring_up.c

# Source files for target example_detector_distance
SRC_FILES_EXAMPLE_DETECTOR_DISTANCE := \
  $(ACCONEER_ZIP_ROOT)/source/example_detector_distance.c

# Source files for target example_detector_distance_recorded
SRC_FILES_EXAMPLE_DETECTOR_DISTANCE_RECORDED := \
  $(ACCONEER_ZIP_ROOT)/source/example_detector_distance_recorded.c

# Source files for target example_detector_presence
SRC_FILES_EXAMPLE_DETECTOR_PRESENCE := \
  $(ACCONEER_ZIP_ROOT)/source/example_detector_presence.c

# Source files for target example_get_next_by_reference
SRC_FILES_EXAMPLE_GET_NEXT_BY_REFERENCE := \
  $(ACCONEER_ZIP_ROOT)/source/example_get_next_by_reference.c

# Source files for target example_multiple_service_usage
SRC_FILES_EXAMPLE_MULTIPLE_SERVICE_USAGE := \
  $(ACCONEER_ZIP_ROOT)/source/example_multiple_service_usage.c

# Source files for target example_error_handling
SRC_FILES_EXAMPLE_ERROR_HANDLING := \
  $(ACCONEER_ZIP_ROOT)/source/example_error_handling.c

# Source files for target example_service_envelope
SRC_FILES_EXAMPLE_SERVICE_ENVELOPE := \
  $(ACCONEER_ZIP_ROOT)/source/example_service_envelope.c

# Source files for target example_service_iq
SRC_FILES_EXAMPLE_SERVICE_IQ := \
  $(ACCONEER_ZIP_ROOT)/source/example_service_iq.c

# Source files for target example_service_power_bins
SRC_FILES_EXAMPLE_SERVICE_POWER_BINS := \
  $(ACCONEER_ZIP_ROOT)/source/example_service_power_bins.c

# Source files for target example_service_sparse
SRC_FILES_EXAMPLE_SERVICE_SPARSE := \
  $(ACCONEER_ZIP_ROOT)/source/example_service_sparse.c

# Source files for target ref_app_parking
SRC_FILES_REF_APP_PARKING := \
  $(ACCONEER_ZIP_ROOT)/source/ref_app_parking.c \

# Source files for target ref_app_smart_presence
SRC_FILES_REF_APP_SMART_PRESENCE := \
  $(ACCONEER_ZIP_ROOT)/source/ref_app_smart_presence.c \

# Source files for target ref_app_tank_level
SRC_FILES_REF_APP_TANK_LEVEL := \
  $(ACCONEER_ZIP_ROOT)/source/ref_app_tank_level.c \

# Source files for target ref_app_rf_certification_test
SRC_FILES_REF_APP_RF_CERTIFICATION_TEST := \
  $(ACCONEER_ZIP_ROOT)/source/ref_app_rf_certification_test.c \

# Source files for target ble_monitor_presence
SRC_FILES_BLE_MONITOR_PRESENCE := \
  $(ACCONEER_ZIP_ROOT)/source/acc_battery_info_xm122.c \
  $(ACCONEER_ZIP_ROOT)/source/ble_monitor_app.c

# Source files common to all targets
_SRC_FILES += \
  $(ACCONEER_ZIP_ROOT)/source/main.c \
  $(ACCONEER_ZIP_ROOT)/source/acc_integration_log.c \
  $(ACCONEER_ZIP_ROOT)/source/acc_integration_nordic.c \
  $(ACCONEER_ZIP_ROOT)/source/acc_hal_integration_xm122.c \
  $(ACCONEER_ZIP_ROOT)/source/printf.c \
  $(ACCONEER_ZIP_ROOT)/source/acc_wrap_printf.c \
  $(ACCONEER_ZIP_ROOT)/source/acc_bluetooth_beacon_xm122.c \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_uart.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/fifo/app_fifo.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/uart/app_uart_fifo.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/hardfault/nrf52/handler/hardfault_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/atomic_flags/nrf_atflags.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52840.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_rtc.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_power.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_spi.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_uart.c \
  $(SDK_ROOT)/modules/nrfx/soc/nrfx_atomic.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_power.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_saadc.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spim.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uart.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uarte.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/external/utf_converter/utf.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \


# Include folders common to all targets
INC_FOLDERS += \
  $(ACCONEER_ZIP_ROOT)/include \
  $(SDK_ROOT)/components/nfc/ndef/generic/message \
  $(SDK_ROOT)/components/nfc/t2t_lib \
  $(SDK_ROOT)/components/nfc/t4t_parser/hl_detection_procedure \
  $(SDK_ROOT)/components/ble/ble_services/ble_ancs_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias_c \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/generic \
  $(SDK_ROOT)/components/libraries/usbd/class/msc \
  $(SDK_ROOT)/components/libraries/usbd/class/hid \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/le_oob_rec_parser \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/ble/ble_services/ble_gls \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/nfc/ndef/text \
  $(SDK_ROOT)/components/libraries/mutex \
  $(SDK_ROOT)/components/libraries/gfx \
  $(SDK_ROOT)/components/libraries/bootloader/ble_dfu \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/common \
  $(SDK_ROOT)/components/libraries/fifo \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/nfc/ndef/generic/record \
  $(SDK_ROOT)/components/nfc/t4t_parser/cc_file \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/external/utf_converter \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/components/libraries/experimental_task_manager \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c \
  $(SDK_ROOT)/components/softdevice/s140/headers/nrf52 \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/le_oob_rec \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs_c \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_lls \
  $(SDK_ROOT)/components/libraries/hardfault/nrf52 \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ac_rec \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/libraries/mpu \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/ble/ble_services/ble_ans_c \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/components/libraries/csense_drv \
  $(SDK_ROOT)/components/libraries/memobj \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus_c \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/mouse \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ble_oob_advdata_parser \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/components/libraries/svc \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/cli \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs \
  $(SDK_ROOT)/components/ble/ble_services/ble_hts \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/nfc/t4t_parser/apdu \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/ecc \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/ble/ble_services/ble_cscs \
  $(SDK_ROOT)/components/libraries/uart \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/kbd \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/softdevice/s140/headers \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/components/nfc/t4t_parser/tlv \
  $(SDK_ROOT)/components/libraries/sortlist \
  $(SDK_ROOT)/components/libraries/spi_mngr \
  $(SDK_ROOT)/components/libraries/led_softblink \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser \
  $(SDK_ROOT)/components/libraries/sdcard \
  $(SDK_ROOT)/components/nfc/ndef/parser/record \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/components/ble/ble_services/ble_cts_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/libraries/twi_mngr \
  $(SDK_ROOT)/components/ble/ble_services/ble_hids \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_oob_advdata \
  $(SDK_ROOT)/components/nfc/t2t_parser \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_msg \
  $(SDK_ROOT)/components/libraries/usbd/class/audio \
  $(SDK_ROOT)/components/nfc/t4t_lib \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/components/libraries/ringbuf \
  $(SDK_ROOT)/components/ble/ble_services/ble_tps \
  $(SDK_ROOT)/components/nfc/ndef/parser/message \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  $(SDK_ROOT)/components/nfc/ndef/uri \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/components/libraries/twi_sensor \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ep_oob_rec \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_lib \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/nfc/ndef/launchapp \
  $(SDK_ROOT)/components/libraries/atomic_flags \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/hs_rec \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ac_rec_parser \
  $(SDK_ROOT)/components/libraries/stack_guard \
  $(SDK_ROOT)/components/libraries/log/src \


# Libraries common to all targets and the target specific library libacc_rf_certification_test.a
LIB_FILES += \
    -Wl,--start-group \
  $(ACCONEER_ZIP_ROOT)/lib/libacc_detector_distance.a \
  $(ACCONEER_ZIP_ROOT)/lib/libacc_detector_presence.a \
  $(ACCONEER_ZIP_ROOT)/lib/libacc_rf_certification_test.a \
  $(ACCONEER_ZIP_ROOT)/lib/libacconeer.a \
    -Wl,--end-group \

include Makefile_softdevice

# Optimization flags
OPT = -O0 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
ifeq ($(BOARD),PCA10056)
CFLAGS += -DBOARD_PCA10056
else
CFLAGS += -DCUSTOM_BOARD_INC=board_xm122
endif
CFLAGS += -DCUSTOM_BOARD_INC=board_xm122
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52840_XXAA
CFLAGS += $(NRF_SOFTDEVICE_FLAGS)
CFLAGS += -DSWI_DISABLE0
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums
ifneq ($(SUSPEND_TIME_BETWEEN_UPDATES_MS),)
CFLAGS += -DSUSPEND_TIME_BETWEEN_UPDATES_MS=$(SUSPEND_TIME_BETWEEN_UPDATES_MS)
endif
ifneq ($(USE_BLE_ADVERTISING),)
CFLAGS += -DUSE_BLE_ADVERTISING=$(USE_BLE_ADVERTISING)
endif
ifneq ($(POWER_SAVE_MODE),)
CFLAGS += -DPOWER_SAVE_MODE=$(POWER_SAVE_MODE)
endif

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ifeq ($(BOARD),PCA10056)
ASMFLAGS += -DBOARD_PCA10056
else
ASMFLAGS += -DCUSTOM_BOARD_INC=board_xm122
endif
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += $(NRF_SOFTDEVICE_FLAGS)
ASMFLAGS += -DSWI_DISABLE0

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)

LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs
# Wrap printf and friends to a minimal but fully working implementation
LDFLAGS += -Wl,--wrap=puts -Wl,--wrap=printf -Wl,--wrap=snprintf -Wl,--wrap=vsnprintf -Wl,--wrap=vsnprintf

CFLAGS += -D__HEAP_SIZE=204800
ASMFLAGS += -D__HEAP_SIZE=204800
#ASMFLAGS += -D__STACK_SIZE=8192
#CFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm

# Windows doesn't have /dev/null but NUL
DEV_NULL := $(if $(filter Windows%,$(OS)),NUL:,/dev/null)

.PHONY: default help all

# Default target - first one defined
default: all

# Function to convert string to uppercase
# Parameter 1: string to convert to uppercase
uc = $(subst a,A,$(subst b,B,$(subst c,C,$(subst d,D,$(subst e,E,$(subst f,F,$(subst g,G,$(subst \
	h,H,$(subst i,I,$(subst j,J,$(subst k,K,$(subst l,L,$(subst m,M,$(subst n,N,$(subst o,O,$(subst \
	p,P,$(subst q,Q,$(subst r,R,$(subst s,S,$(subst t,T,$(subst u,U,$(subst v,V,$(subst w,W,$(subst \
	x,X,$(subst y,Y,$(subst z,Z,$1))))))))))))))))))))))))))

# Create a flash target based on the target name
# (1) target name
define define_flash_target
.PHONY: flash_$(strip $(1))
flash_$(strip $(1)):
	@echo Flashing: $(OUTPUT_DIRECTORY)/$(strip $(1)).hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/$(strip $(1)).hex --sectorerase
	nrfjprog -f nrf52 --reset
endef

define define_dfu_target
.PHONY: create_dfu_$(strip $(1))
all: create_dfu_$(strip $(1))
create_dfu_$(strip $(1)): $(strip $(1))
	@echo Generating: $(OUTPUT_DIRECTORY)/$(strip $(1)).zip
	@nrfutil pkg generate --hw-version 52 --application-version 0 --application $(OUTPUT_DIRECTORY)/$(strip $(1)).hex \
		--sd-req $(NRF_SOFTDEVICE_FIRMWARE_ID) $(OUTPUT_DIRECTORY)/$(strip $(1)).zip >$(DEV_NULL)
endef

define define_bl_settings_target
.PHONY: $(strip $(1))_bl_settings
all: $(strip $(1))_bl_settings
$(strip $(1))_bl_settings: $(strip $(1))
	@echo Generating: $(OUTPUT_DIRECTORY)/$(strip $(1))_bl_settings.hex
	@nrfutil settings generate --family NRF52840 --application $(OUTPUT_DIRECTORY)/$(strip $(1)).hex \
		--application-version 0 --bootloader-version 0 --bl-settings-version 2 $(OUTPUT_DIRECTORY)/$(strip $(1))_bl_settings.hex >$(DEV_NULL)
endef

define define_transfer_dfu_target
.PHONY: transfer_dfu_$(strip $(1))
transfer_dfu_$(strip $(1)): $(OUTPUT_DIRECTORY)/$(strip $(1)).zip
	@echo Transferring: $(OUTPUT_DIRECTORY)/$(strip $(1)).zip
	@SERIAL_NUMBER=$$$$(usb-devices | grep -A 1 XB122 | tail -n 1 | sed -e "s/S:  SerialNumber=\(.*\)/\1/") ; \
	nrfutil dfu serial -pkg $(OUTPUT_DIRECTORY)/$(strip $(1)).zip -snr $$$$SERIAL_NUMBER
endef

define define_size_target
.PHONY: size_$(strip $(1))
all: size_$(strip $(1))
size_$(strip $(1)): $(strip $(1))
	@echo Generating: $(OUTPUT_DIRECTORY)/$(strip $(1))_size.txt
	$(NO_ECHO)$(SIZE) -t $(OUTPUT_DIRECTORY)/$(strip $(1)).out > $(OUTPUT_DIRECTORY)/$(strip $(1))_size.txt
endef

# Print all targets that can be built
help:
	@echo following targets are available:
	@$(foreach target,$(TARGETS),echo $(target);)
	@$(foreach target,$(TARGETS),echo $(target)_bl_settings;)
	@echo		flash_softdevice
	@$(foreach target,$(TARGETS),echo flash_$(strip $(target));)
	@$(foreach target,$(TARGETS),echo flash_$(strip $(target))_bl_settings;)
	@$(foreach target,$(TARGETS),echo transfer_dfu__$(strip $(target));)
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		erase      - erase flash

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common
include Makefile_override_error

# Create build targets
# SRC_FILES is used in the define_target macro and will be different for different targets
$(foreach target, $(TARGETS), $(eval SRC_FILES := $(_SRC_FILES) $($(addsuffix $(call uc,$(target)),SRC_FILES_))) $(call define_target, $(target)))

# Create flash targets
$(foreach target, $(TARGETS), $(eval $(call define_flash_target, $(target))))
$(foreach target, $(TARGETS), $(eval $(call define_flash_target, $(target)_bl_settings)))

# Create create_dfu targets
$(foreach target, $(TARGETS), $(eval $(call define_dfu_target, $(target))))

# Create bl_settings targets
$(foreach target, $(TARGETS), $(eval $(call define_bl_settings_target, $(target))))

# Create transfer_dfu targets
$(foreach target, $(TARGETS), $(eval $(call define_transfer_dfu_target, $(target))))

# Create size targets
$(foreach target, $(TARGETS), $(eval $(call define_size_target, $(target))))

all: $(TARGETS)

.PHONY: flash_softdevice erase

# Flash softdevice
flash_softdevice:
	@echo Flashing: $(NRF_SOFTDEVICE_HEX)
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s140/hex/$(NRF_SOFTDEVICE_HEX) --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := include/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)


openocd_gdbserver:
	openocd -s /usr/local/bin/openocd/tcl -f interface/jlink-swd.cfg -f target/nrf52.cfg -c init -c targets -c "reset halt"

openocd_gdbclient:
	gdb-multiarch -ex "target remote localhost:3333" -ex "mon reset halt" $1
