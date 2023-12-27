cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(scioly_bot)

pico_sdk_init()

add_executable(scioly_bot
        src/main.cpp
        src/pathing.cpp
        )

pico_generate_pio_header(scioly_bot ${CMAKE_CURRENT_LIST_DIR}/quad.pio)


# pull in common dependencies
target_link_libraries(scioly_bot pico_stdlib hardware_pwm hardware_i2c hardware_pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(scioly_bot 1)
pico_enable_stdio_uart(scioly_bot 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(scioly_bot)

# add url via pico_set_program_url
# example_auto_set_url(scioly_bot)
