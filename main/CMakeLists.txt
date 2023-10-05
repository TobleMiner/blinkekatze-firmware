file(GLOB srcs src/*.c)

idf_component_register(SRCS ${srcs}
		       INCLUDE_DIRS "src")
