/**
 * Author:    Martin Michalik (xmicha82)
 * Created:   5.2024
 **/

#include "stdio.h"
#include "btstack.h"
#include "btstack_event.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "haw/MPU6050.h"
#include <math.h>

#include "pico_dice.h"
#include "dice_types.h"


// part of outline for BTstack enabled app
#define APP_AD_FLAGS 0x06

static btstack_packet_callback_registration_t hci_event_callback_registration;
static hci_con_handle_t con_handle;
static int accel_notification_enabled;
static btstack_timer_source_t heartbeat;

const uint8_t adv_data[] = {
  // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x0a, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', ' ', 'D', 'i', 'c', 'e',
    0x03,
    BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x10, 0xff
};

const uint8_t adv_data_len = sizeof(adv_data);

// end outline

mpu6050_t mpu6050;
mpu6050_activity_t* activities;
mpu6050_vectorf_t *accel;

vectorf_t previous[4] = {
    {0.0,0.0,0.0},
    {0.0,0.0,0.0},
    {0.0,0.0,0.0},
    {0.0,0.0,0.0}
};

static char result_string[10];
static int result_string_len;

uint8_t dice_roll = 0;
DiceStruct current_dice = {
    D6,
    d6_sides,
    6
};

float dot_product(vectorf_t v1, vectorf_t v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

int8_t max_correlation_index(vectorf_t input) {
  int8_t max_index = -1;
  float max_correlation = 0;
  for(int i = 0; i < current_dice.size; i++) {
    float correlation = dot_product(current_dice.sides[i], input);
    printf("[%d/%d]: %f", i, current_dice.size, correlation);
    if(correlation > max_correlation) {
      max_correlation = correlation;
      max_index = i;
    }
  }

  return max_index;
}

void save_to_previous(vectorf_t v) {
    previous[3] = previous[2];
    previous[2] = previous[1];
    previous[1] = previous[0];
    previous[0] = v;
}

int is_between(float f, float lower, float higher) {
    return f > lower && f < higher;
}

void get_sensor_data() {
    mpu6050_event(&mpu6050);

    accel = mpu6050_get_accelerometer(&mpu6050);

    printf("%02.1f,%02.1f,%02.1f\n", accel->x, accel->y, accel->z);
}

void prepare_result() {
    result_string_len = snprintf(result_string, sizeof(result_string), "%d", max_correlation_index(*accel) + 1);
}

int dice_inactive(){
    static int was_active = false;

    if(
        is_between(dot_product(previous[0], *accel), 20, 30) &&
        is_between(dot_product(previous[1], *accel), 20, 30) &&
        is_between(dot_product(previous[2], *accel), 20, 30) &&
        is_between(dot_product(previous[3], *accel), 20, 30)) {
            if (was_active){
                was_active = false;
                return true;
            }

            return false;
    } else {
        was_active = true;
        return false;
    }
}

// function signatures part of BTstack example outline


static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_address;

    if(packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch(event_type) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;

            gap_local_bd_addr(local_address);
            bd_addr_to_str(local_address);

            //setup advertisements
            uint16_t adv_int_min = 800;
            uint16_t adv_int_max = 800;
            uint8_t adv_type = 0;
            bd_addr_t null_addr;
            memset(null_addr, 0, 6);
            gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
            gap_advertisements_set_data(adv_data_len, (uint8_t*)adv_data);
            gap_advertisements_enable(true);

            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            accel_notification_enabled = false;
            break;
        case ATT_EVENT_CAN_SEND_NOW:
            int stat = att_server_notify(con_handle, ATT_CHARACTERISTIC_F943D189_C2D5_4C98_85C8_8EE552B4A000_01_VALUE_HANDLE, (const uint8_t*)result_string, result_string_len);
        default:
        break;
    }
}

uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);

    if (att_handle == ATT_CHARACTERISTIC_F943D189_C2D5_4C98_85C8_8EE552B4A000_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob((const uint8_t*)result_string, result_string_len, offset, buffer, buffer_size);
    }
    return 0;
}

int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    switch(att_handle) {
        case ATT_CHARACTERISTIC_F943D189_C2D5_4C98_85C8_8EE552B4A000_01_CLIENT_CONFIGURATION_HANDLE:
            accel_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
            con_handle = connection_handle;
            return 0;
            break;
        case ATT_CHARACTERISTIC_5AFC00F8_D780_44D7_9BBD_D025941A2B23_01_VALUE_HANDLE:
            int dice = little_endian_read_16(buffer, 0);

            switch (dice)
            {
            case D6:
                current_dice = d6;
                break;
            case D8:
                current_dice = d8;
                break;
            default:
                current_dice = d6;
                break;
            }
            return 0;
            break;
        default:
            return 0;
    }
}

// end outline code


int main() {
    gpio_set_function(16, GPIO_FUNC_UART); //TX
    gpio_set_function(17, GPIO_FUNC_UART); //RX
    stdio_init_all();

    // nastavenie I2C
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    // Pullup rezistory pre I2C
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    sleep_ms(500);

    mpu6050 = mpu6050_init(i2c0, MPU6050_ADDRESS_A0_GND);

    // Check if the MPU6050 can initialize
    // code taken from
    // https://github.com/HumansAreWeak/rpi-pico-mpu6050/blob/master/example/mpu6050_accelerometer_and_gyro_read.c
    if (mpu6050_begin(&mpu6050))
    {
        // Set scale of gyroscope
        mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
        // Set range of accelerometer
        mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);

        // Enable temperature, gyroscope and accelerometer readings
        mpu6050_set_gyroscope_measuring(&mpu6050, true);
        mpu6050_set_accelerometer_measuring(&mpu6050, true);

        // Enable free fall, motion and zero motion interrupt flags
        mpu6050_set_int_free_fall(&mpu6050, false);
        mpu6050_set_int_motion(&mpu6050, false);
        mpu6050_set_int_zero_motion(&mpu6050, false);

        // Set motion detection threshold and duration
        mpu6050_set_motion_detection_threshold(&mpu6050, 2);
        mpu6050_set_motion_detection_duration(&mpu6050, 5);

        // Set zero motion detection threshold and duration
        mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);
        mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);
    }
    else
    {
        while (1)
        {
            // Endless loop
            printf("Error! MPU6050 could not be initialized. Make sure you've entered the correct address. And double check your connections.\n");
            sleep_ms(500);
        }
    }

    if(cyw43_arch_init()) {
        while (true) {
            printf("Error! CYW43 could not be initialized\n");
            sleep_ms(500);
        }
    }

    // end taken code

    // code part of BTstack outlne
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();
    sm_init();

    att_server_init(profile_data, att_read_callback, att_write_callback);

    att_server_register_packet_handler(packet_handler);

    hci_power_control(HCI_POWER_ON);

    //end outline code

    while (1)
    {
        static int led_on = true;
        led_on = !led_on;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

        get_sensor_data();

        int inactive = dice_inactive();

        if(accel_notification_enabled && inactive) {
            prepare_result();
            att_server_request_can_send_now_event(con_handle);
        }

        save_to_previous(*accel);

        sleep_ms(500);
    }
}
