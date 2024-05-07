set(PICO_BOARD pico_w)

pico_sdk_init()

add_subdirectory(vendors)

add_executable(pico_dice
    pico_dice.c
    )
target_link_libraries(pico_dice
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    haw_mpu6050
    )
target_include_directories(pico_dice PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    )
pico_btstack_make_gatt_header(pico_dice PRIVATE "${CMAKE_CURRENT_LIST_DIR}/pico_dice.gatt")

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_dice 0)
pico_enable_stdio_uart(pico_dice 1)

pico_add_extra_outputs(pico_dice)
