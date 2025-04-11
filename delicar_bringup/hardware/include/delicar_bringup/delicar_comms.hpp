#ifndef DELICAR_HARDWARE_DELICAR_COMMS_H
#define DELICAR_HARDWARE_DELICAR_COMMS_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <modbus/modbus.h>
#include <cstring>
#include <string>

class DelicarComms
{
public:
    DelicarComms() 
    : modbus_ctx_(nullptr), 
    is_connected_(false),
    logger_(rclcpp::get_logger("DelicarComms"))
    {
    }
    
    ~DelicarComms()
    {
        disconnect();
    }
    
    // Initialize connection parameters
    void init(const std::string &serial_device, int baud_rate, int device_id, char parity = 'N', int data_bit = 8, int stop_bit = 1)
    {
        serial_device_ = serial_device;
        baud_rate_ = baud_rate;
        device_id_ = device_id;
        parity_ = parity;
        data_bit_ = data_bit;
        stop_bit_ = stop_bit;
        is_connected_ = false;
    }
    
    // Connect to the device
    bool connect()
    {
        // Disconnect if already connected
        if (modbus_ctx_) 
        {
            disconnect();
        }

        RCLCPP_INFO(
            logger_,
            "Connecting to Modbus RTU on %s at %d baud with device ID %d",
            serial_device_.c_str(), baud_rate_, device_id_);

        // Create a new Modbus context
        modbus_ctx_ = modbus_new_rtu(
            serial_device_.c_str(), 
            baud_rate_, 
            parity_, 
            data_bit_, 
            stop_bit_);

        if (modbus_ctx_ == nullptr) {
            RCLCPP_ERROR(logger_, "Failed to create Modbus context");
            is_connected_ = false;
            return false;
        }

        // Set the slave ID
        modbus_set_slave(modbus_ctx_, device_id_);

        // // Set response timeout
        // struct timeval timeout;
        // timeout.tv_sec = 0;
        // timeout.tv_usec = 500000; // 500ms
        // modbus_set_response_timeout(modbus_ctx_, &timeout);

        // Connect to the device
        if (modbus_connect(modbus_ctx_) == -1) {
            RCLCPP_ERROR(
                logger_, 
                "Failed to connect to Modbus device: %s", 
                modbus_strerror(errno));
            modbus_free(modbus_ctx_);
            modbus_ctx_ = nullptr;
            is_connected_ = false;
            return false;
        }

        RCLCPP_INFO(logger_, "Successfully connected to Modbus device");
        is_connected_ = true;
        return true;
    }

    // Disconnect from the device
    void disconnect()
    {
        if (modbus_ctx_) {
            modbus_close(modbus_ctx_);
            modbus_free(modbus_ctx_);
            modbus_ctx_ = nullptr;
        }
        is_connected_ = false;
        RCLCPP_INFO(logger_, "Disconnected from Modbus device");
    }
    
    // Check if connected
    bool connected() const
    {
        return is_connected_;
    }

    // Read feedback values
    void readFeedbackValues(int &feedback_value, int reg)
    {
        if (!connected()) {
            RCLCPP_WARN(logger_, "Attempted to read but not connected to Modbus device");
            return;
        }
        
        // Read feedback register
        if (!readRegister(reg, feedback_value)) {
            RCLCPP_ERROR(logger_, "Failed to read feedback from register %d", reg);
        }
    }
    
    // Set motor values
    void setMotorValues(int drive_value, int reg)
    {
        if (!connected()) {
            RCLCPP_WARN(logger_, "Attempted to write but not connected to Modbus device");
            return;
        }
        
        // Write to motor control register
        if (!writeRegister(reg, drive_value)) {
            RCLCPP_ERROR(logger_, "Failed to write value %d to register %d", drive_value, reg);
        }
    }
    

    // Write to a modbus register
    bool writeRegister(int addr, int value)
    {
        if (!connected()) {
            return false;
        }
        
        // Write a single register value
        int result = modbus_write_register(modbus_ctx_, addr, static_cast<uint16_t>(value));
        
        if (result != 1) {
            RCLCPP_ERROR(
                logger_, 
                "Modbus write error: %s (register: %d, value: %d)",
                modbus_strerror(errno), addr, value);
            return false;
        }
    
        return true; 
    }
    
    // Read from a modbus register
    bool readRegister(int addr, int &value)
    {
        if (!connected()) {
            return false;
        }
        
        uint16_t buf;
        // Read a single register value
        int result = modbus_read_registers(modbus_ctx_, addr, 1, &buf);
        
        if (result != 1) {
            RCLCPP_ERROR(
                logger_, 
                "Modbus read error: %s (register: %d)",
                modbus_strerror(errno), addr);
            return false;
        }
        
        value = static_cast<int>(buf);
        return true;
    }

private:
    modbus_t *modbus_ctx_;
    std::string serial_device_;
    int baud_rate_;
    int device_id_;
    char parity_;
    int data_bit_;
    int stop_bit_;
    bool is_connected_;
    rclcpp::Logger logger_;
};


#endif // DELICAR_HARDWARE_DELICAR_COMMS_H