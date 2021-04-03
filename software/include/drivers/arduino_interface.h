#ifndef ARDUINO_INTERFACE_H
#define ARDUINO_INTERFACE_H

// System 
#include <iostream>
#include <string>

// Third Party
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

// Constants
#include "constants.h"

// Drivers
#include "sensor_data.h"
#include "serial_port.h"

// GNC
#include "gnc/control/control_output.h"

class ArduinoInterface 
{
    public: 
        ArduinoInterface(RamseteOutput *ctrl_out,
                         SensorData *sensor_data,
                         std::string serial_channel) :
            ctrl_out_(ctrl_out),
            sensor_data_(sensor_data),
            serial_port_(serial_channel) {}

        // Send commands to the arduino and get the sensor data
        void update();

        virtual ~ArduinoInterface();
    
    private:
        // Output from the Ramsete controller
        RamseteOutput *const ctrl_out_;

        // Send sensor info back to GNC
        SensorData *sensor_data_;

        // Save serial channel info
        SerialPort serial_port_;
};

#endif