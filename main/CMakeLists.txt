idf_component_register(SRCS "mpu6050_collect_main.c" "mpu6050_filter.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver spiffs)
