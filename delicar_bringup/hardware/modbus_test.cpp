#include <stdio.h>
#include <modbus/modbus.h>
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    modbus_t* ctx;
    uint16_t vel_value = 1000;
    uint16_t d1_value = 0;
    
    // Configure Modbus RTU
    ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'O', 8, 1);
    
    if (ctx == NULL) {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        return -1;
    }
    
    // Set the slave ID (device address)
    modbus_set_slave(ctx, 1);
    
    // Connect to the device
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    
    // Write single register (D1)
    // if (modbus_write_register(ctx, 1, vel_value) == -1) {
    //     fprintf(stderr, "Failed to write to D1: %s\n", modbus_strerror(errno));
    // } else {
    //     printf("Written value %d to D0\n", vel_value);
    // }
    
    // Read from D1
    if (modbus_read_registers(ctx, 1, 1, &d1_value) == -1) {
        fprintf(stderr, "Failed to read D1: %s\n", modbus_strerror(errno));
    } else {
        printf("Read value from D1: %d\n", d1_value);
    }
    
    // Clean up
    modbus_close(ctx);
    modbus_free(ctx);
    
    return 0;
}