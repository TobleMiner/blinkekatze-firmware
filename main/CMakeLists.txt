set(srcs
	src/bonk.c
	src/bq24295.c
	src/bq27546.c
	${BUILD_DIR}/bq27546_inr18650_df.c
	src/chacha20.c
	src/color_channel_offset.c
	src/color_override.c
	src/debounce.c
	src/default_color.c
	src/fast_hsv2rgb_32bit.c
	src/futil.c
	src/i2c_bus.c
	src/lis3dh.c
	src/ltr_303als.c
	src/main.c
	src/neighbour.c
	src/neighbour_rssi_delay_model.c
	src/neighbour_static_info.c
	src/neighbour_status.c
	src/nvs.c
	src/node_info.c
	src/ota.c
	src/platform_blinkekatze.c
	src/platform_lacklight.c
	src/platform_laempan.c
	src/platform.c
	src/power_control.c
	src/rainbow_fade.c
	src/scheduler.c
	src/settings.c
	src/shared_config.c
	src/shell.c
	src/spl06.c
	src/squish.c
	src/state_of_charge.c
	src/status_leds.c
	src/strutil.c
	src/tcp_client.c
	src/tcp_memory_server.c
	src/uid.c
	src/usb.c
	src/util.c
	src/wireless.c)

idf_component_register(SRCS ${srcs}
		       INCLUDE_DIRS "src"
		       EMBED_FILES ${BUILD_DIR}/colorcal_16x16x16_11bit.bin ${BUILD_DIR}/colorcal_16x16x16_12bit.bin ${BUILD_DIR}/colorcal_32x32x32_12bit.bin ${PROJECT_DIR}/main/assets/wireless_key)

add_custom_command(OUTPUT ${BUILD_DIR}/bq27546_inr18650_df.c
		   COMMAND ${PROJECT_DIR}/tools/flash_stream_to_c.py ${CMAKE_CURRENT_LIST_DIR}/assets/bq27546_INR18650-35E.df.fs ${BUILD_DIR}/bq27546_inr18650_df.c bq27546_inr18650_dataflash
		   DEPENDS ${CMAKE_CURRENT_LIST_DIR}/assets/bq27546_INR18650-35E.df.fs ${PROJECT_DIR}/tools/flash_stream_to_c.py
		   VERBATIM)
add_custom_target(gauge_dataflash DEPENDS ${BUILD_DIR}/bq27546_inr18650_df.c)
add_dependencies(${COMPONENT_LIB} gauge_dataflash)
set_property(DIRECTORY "${COMPONENT_DIR}" APPEND PROPERTY
	     ADDITIONAL_CLEAN_FILES ${BUILD_DIR}/bq27546_inr18650_df.c)

add_custom_command(OUTPUT ${BUILD_DIR}/colorcal_16x16x16_11bit.bin
		   COMMAND ${PROJECT_DIR}/tools/displaycal_to_bin.py 65535 ${CMAKE_CURRENT_LIST_DIR}/assets/blinkekatze_rev1.0_cal_16x16x16_16bit.png 2047 ${BUILD_DIR}/colorcal_16x16x16_11bit.bin
		   DEPENDS ${CMAKE_CURRENT_LIST_DIR}/assets/blinkekatze_rev1.0_cal_16x16x16_16bit.png ${PROJECT_DIR}/tools/displaycal_to_bin.py
		   VERBATIM)

add_custom_command(OUTPUT ${BUILD_DIR}/colorcal_16x16x16_12bit.bin
		   COMMAND ${PROJECT_DIR}/tools/displaycal_to_bin.py 65535 ${CMAKE_CURRENT_LIST_DIR}/assets/blinkekatze_rev1.0_cal_16x16x16_16bit.png 4080 ${BUILD_DIR}/colorcal_16x16x16_12bit.bin
		   DEPENDS ${CMAKE_CURRENT_LIST_DIR}/assets/blinkekatze_rev1.0_cal_16x16x16_16bit.png ${PROJECT_DIR}/tools/displaycal_to_bin.py
		   VERBATIM)

add_custom_command(OUTPUT ${BUILD_DIR}/colorcal_32x32x32_12bit.bin
		   COMMAND ${PROJECT_DIR}/tools/displaycal_to_bin.py 65535 ${CMAKE_CURRENT_LIST_DIR}/assets/blinkekatze_rev1.0_cal_32x32x32_16bit.png 4080 ${BUILD_DIR}/colorcal_32x32x32_12bit.bin
		   DEPENDS ${CMAKE_CURRENT_LIST_DIR}/assets/blinkekatze_rev1.0_cal_32x32x32_16bit.png ${PROJECT_DIR}/tools/displaycal_to_bin.py
		   VERBATIM)

add_custom_target(colorcal_bin DEPENDS ${BUILD_DIR}/colorcal_16x16x16_11bit.bin ${BUILD_DIR}/colorcal_16x16x16_12bit.bin ${BUILD_DIR}/colorcal_32x32x32_12bit.bin)
add_dependencies(${COMPONENT_LIB} colorcal_bin)
set_property(DIRECTORY "${COMPONENT_DIR}" APPEND PROPERTY
	     ADDITIONAL_CLEAN_FILES ${BUILD_DIR}/colorcal_16x16x16_11bit.bin ${BUILD_DIR}/colorcal_16x16x16_12bit.bin ${BUILD_DIR}/colorcal_32x32x32_12bit.bin)
