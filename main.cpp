/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "InterfaceCAN.h"
#include <cstdint>
#include <ctime>
#include <cmath>
#define CAN1_RX PD_0
#define CAN1_TX PD_1
#define ID 125
#define CURRENT_ID 1
#define POSITION_ID 4

const int32_t BAUDRATE = 500000;
CAN can1(CAN1_RX, CAN1_TX, BAUDRATE); // CAN controller
const int32_t CURRENT = 10; // desired current in amps
const int32_t STOP_CURRENT = 0; // stopping current
const double delay_time = 0.2;

int32_t degrees_to_hex(double degrees) {
    double new_degrees = degrees * 1000000;
    int32_t hex_value = static_cast<int32_t>(round(new_degrees));
    return hex_value;
}

const int32_t POS_1 = degrees_to_hex(90); // desired degrees
const int32_t POS_2 = degrees_to_hex(270); // desired degrees

// Utility function to create the CAN ID with the mode and vesc ID
static constexpr uint32_t FORMULATE_ID(const uint32_t &vesc_id, const uint32_t &mode_id) {
    auto bit_size = 8;
    return (static_cast<uint32_t>(mode_id)
        << sizeof(uint8_t) * bit_size) |
        static_cast<uint32_t>(vesc_id);
}
// Creating IDs
uint32_t CURRENT_EXTENDED_ID = FORMULATE_ID(ID, CURRENT_ID);
uint32_t POS_EXTENDED_ID = FORMULATE_ID(ID, CURRENT_ID);

// converts unsigned 32-byte int to format of MSB --> LSB (ty Jesus)
void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index) {
    int bitshift_1 = 24;
    int bitshift_2 = 16;
    int bitshift_3 = 8; 
    buffer[(*index)++] = number >> bitshift_1;
    buffer[(*index)++] = number >> bitshift_2;
    buffer[(*index)++] = number >> bitshift_3;
    buffer[(*index)++] = number;
    }

double get_time_seconds(clock_t start) {
    return (clock()-start) / CLOCKS_PER_SEC;
}

void send_current(int32_t current) {
    int amp_factor = 1000;
    int32_t this_curr = current*amp_factor; // convert to amps
    uint8_t message[4] = {0, 0, 0, 0};  // Init blank message
    int32_t idx = 0;                    // Index for setting msg

    buffer_append_uint32(&message[0], this_curr, &idx);
    CANMessage msg = CANMessage(CURRENT_ID, &message[0],
                                  sizeof(message), CANData, CANExtended);
    can1.write(msg);
}

void send_to_position(int32_t pos) {
    uint8_t message[4] = {0, 0, 0, 0};  // Init blank message
    int32_t idx = 0;                    // Index for setting msg

    buffer_append_uint32(&message[0], pos, &idx);
    CANMessage msg = CANMessage(POSITION_ID, &message[0],
                                  sizeof(message), CANData, CANExtended);
    can1.write(msg);
}

void delay_seconds(double seconds) {
    clock_t start = clock();
    clock_t delay = seconds * CLOCKS_PER_SEC; // convert 200ms to clock ticks
    while (clock() < start + delay) {}
}

int main() {
    int interval = 5;
    int interval2 = 2;
    clock_t start = clock();
    // spin for a while idk
    while(get_time_seconds(start) < interval) {
        send_current(CURRENT);
        delay_seconds(delay_time);
    }
    start = clock();
    while(get_time_seconds(start) < interval2) {
        send_current(STOP_CURRENT);
        delay_seconds(delay_time);
    }
    start = clock();
    // go to position 1
    while(get_time_seconds(start) < interval) {
        send_to_position(POS_1);
        delay_seconds(delay_time);
    }
    start = clock();
    // go to position 2
    while(get_time_seconds(start) < interval) {
        send_to_position(POS_2);
        delay_seconds(delay_time);
    }
}