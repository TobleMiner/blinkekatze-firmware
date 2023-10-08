set(srcs
	src/bonk.c
	src/bq24295.c
	src/bq27546.c
	${BUILD_DIR}/bq27546_inr18650_df.c
	src/fast_hsv2rgb_32bit.c
	src/i2c_bus.c
	src/lis3dh.c
	src/main.c
	src/neighbour.c
	src/neighbour_rssi_delay_model.c
	src/strutil.c
	src/spl06.c
	src/squish.c
	src/wireless.c)

idf_component_register(SRCS ${srcs}
		       INCLUDE_DIRS "src")

add_custom_command(OUTPUT ${BUILD_DIR}/bq27546_inr18650_df.c
		   COMMAND ${PROJECT_DIR}/tools/flash_stream_to_c.py ${CMAKE_CURRENT_LIST_DIR}/assets/bq27546_INR18650-35E.df.fs ${BUILD_DIR}/bq27546_inr18650_df.c bq27546_inr18650_dataflash
		   DEPENDS ${CMAKE_CURRENT_LIST_DIR}/assets/bq27546_INR18650-35E.df.fs ${PROJECT_DIR}/tools/flash_stream_to_c.py
		   VERBATIM)
add_custom_target(gauge_dataflash DEPENDS ${BUILD_DIR}/bq27546_inr18650_df.c)
add_dependencies(${COMPONENT_LIB} gauge_dataflash)
set_property(DIRECTORY "${COMPONENT_DIR}" APPEND PROPERTY
	     ADDITIONAL_CLEAN_FILES ${BUILD_DIR}/bq27546_inr18650_df.c)
