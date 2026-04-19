idf_component_register(
    SRCS
        "src/sh2.c"
        "src/shtp.c"
        "src/sh2_util.c"
        "src/sh2_SensorValue.c"
        "src/euler.c"
        "sh2_hal_esp32.c"
    INCLUDE_DIRS "include" "."
    REQUIRES esp_driver_i2c esp_driver_gpio esp_timer log
)

# Suppress warnings in third-party CEVA source without affecting main/ flags.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-parameter
    -Wno-sign-compare
    -Wno-missing-field-initializers
)
