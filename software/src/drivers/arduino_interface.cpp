#include "drivers/arduino_interface.h"

ArduinoInterface::~ArduinoInterface(){
    serial_port_.Close();
}


void ArduinoInterface::update()
{
    // Convert v and w to left and right power
    double base_vel = ctrl_out_->u(0);
    double wheel_vel_adj = ctrl_out_->u(1)/(Robot::wheelbase_len);
    double left = base_vel - wheel_vel_adj;
    // double right = base_vel + wheel_vel_adj;
    double right = (double)it_test;
    it_test += 1;

    // Build a command packet in JSON
    rapidjson::StringBuffer out_buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(out_buffer);

    // Packet: {"left": 0.0, "right": 0.0}
    writer.StartObject();
    writer.Key("left");
    writer.Double(left);
    writer.Key("right");
    writer.Double(right);
    writer.EndObject();

    // Write to the serial channel
    std::string out_str = out_buffer.GetString();
    out_str += "\n";
    this->arduino_write(out_str);

    // Adding in a sleep results in fewer dropped packets
    usleep(20000);

    // Pull data from the serial channel
    std::string json_str = "";
    int status = this->arduino_read(&json_str);
    const char* json = json_str.c_str();
    
    // Decode a command packet if it is a valid message
    if (status >= 0)
    {
        rapidjson::Document d;
        d.Parse(json);

        // Document must be an object and have the appropriate members
        // TODO: make this validator better
        if(d.IsObject() && d.HasMember("roll") && d.HasMember("pitch") && d.HasMember("yaw"))
        {
            // Get the values from this JSON packet
            rapidjson::Value& roll_val = d["roll"];
            rapidjson::Value& pitch_val = d["pitch"];
            rapidjson::Value& yaw_val = d["yaw"];

            // Update the sensor data from this packet
            sensor_data_->imu_roll = roll_val.GetDouble();
            sensor_data_->imu_pitch = pitch_val.GetDouble();
            sensor_data_->imu_yaw = yaw_val.GetDouble();
        }
        else
        {
            std::cout << "Incorrect format! Received: " << json << std::endl;
        }
    }
    else
    {
        std::cout << "No Serial Data.\n";
    }
}



int ArduinoInterface::open_serial_port(std::string channel)
{
    /* Set up serial port */
    // Try to open the port.
    try {
        serial_port_.Open(channel);
    }
    catch (const LibSerial::OpenFailed&)
    {
        return -1;
    }

    // Set the baud rate of the serial port.
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);

    // Set the number of data bits.
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);

    // Turn off hardware flow control.
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    // Disable parity.
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    
    // Set the number of stop bits.
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    return 0;
}


int ArduinoInterface::arduino_read(std::string *buf) 
{
    std::string buffer;
    try {
        serial_port_.ReadLine(buffer, '\n', 20);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        return -1;
    }

    *buf = buffer;
    return buffer.length();
}

int ArduinoInterface::arduino_write(std::string data) 
{
    serial_port_.Write(data);
    serial_port_.DrainWriteBuffer() ;
    
    return 0;
}


// void ArduinoInterface::update()
// {
//     // Convert v and w to left and right power
//     double base_vel = ctrl_out_->u(0);
//     double wheel_vel_adj = ctrl_out_->u(1)/(Robot::wheelbase_len);
//     double left = base_vel - wheel_vel_adj;
//     double right = base_vel + wheel_vel_adj;

//     // Build a command packet in JSON
//     rapidjson::StringBuffer out_buffer;
//     rapidjson::Writer<rapidjson::StringBuffer> writer(out_buffer);

//     // Packet: {"left": 0.0, "right": 0.0}
//     writer.StartObject();
//     writer.Key("left");
//     writer.Double(left);
//     writer.Key("right");
//     writer.Double(right);
//     writer.EndObject();

//     // Write to the serial channel
//     std::string out_str = out_buffer.GetString();
//     out_str += "\n";
//     serial_port_.write_data(out_str.c_str());

//     // usleep(100000);

//     // Pull data from the serial channel
//     std::string json_str = "";
//     int status = serial_port_.read_data(&json_str);
//     const char* json = json_str.c_str();
    
//     // Decode a command packet if it is a valid message
//     if (status >= 0)
//     {
//         rapidjson::Document d;
//         d.Parse(json);

//         // Document must be an object and have the appropriate members
//         // TODO: make this validator better
//         if(d.IsObject() && d.HasMember("roll") && d.HasMember("pitch") && d.HasMember("yaw"))
//         {
//             // Get the values from this JSON packet
//             rapidjson::Value& roll_val = d["roll"];
//             rapidjson::Value& pitch_val = d["pitch"];
//             rapidjson::Value& yaw_val = d["yaw"];

//             // Update the sensor data from this packet
//             sensor_data_->imu_roll = roll_val.GetDouble();
//             sensor_data_->imu_pitch = pitch_val.GetDouble();
//             sensor_data_->imu_yaw = yaw_val.GetDouble();
//         }
//         else
//         {
//             std::cout << "Incorrect format! Received: " << json << std::endl;
//         }
//     }
//     else
//     {
//         std::cout << "No Serial Data.\n";
//     }
// }