#ifndef ARDUINO_INTERFACE_H
#define ARDUINO_INTERFACE_H

// System 
#include <iostream>
#include <string>
#include <unistd.h>

// Third Party
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

// Constants
#include "constants.h"

// Drivers
#include <libserial/SerialPort.h>
#include "sensor_data.h"
// #include "serial_port.h"

// GNC
#include "gnc/control/control_output.h"

class ArduinoInterface 
{
    public: 
        ArduinoInterface(RamseteOutput *ctrl_out,
                         SensorData *sensor_data,
                         std::string serial_channel) :
            ctrl_out_(ctrl_out),
            sensor_data_(sensor_data){

            open_serial_port(serial_channel);
            it_test = 0;

            // Give the arduino time to reset
            usleep(300000); // 3 seconds is a very conservative estimate based on testing
        }

        // Send commands to the arduino and get the sensor data
        void update();

        virtual ~ArduinoInterface();
    
    private:
        // Output from the Ramsete controller
        RamseteOutput *const ctrl_out_;

        // Send sensor info back to GNC
        SensorData *sensor_data_;

        // Save serial channel info
        LibSerial::SerialPort serial_port_;


        // Do port setup
        int open_serial_port(std::string channel);

        int arduino_read(std::string *buf);
        int arduino_write(std::string data);

        int it_test;
};

#endif