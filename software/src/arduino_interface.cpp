#include "arduino_interface.h"

ArduinoInterface::~ArduinoInterface()
{  
    // Set motors to 0
    this->send_motor_message(0.0, 0.0);

    // Close the serial port
    serial_port_.Close();
}


void ArduinoInterface::update()
{
    // Pull data from the serial channel
    std::string json_str = "";
    int status = this->arduino_read(&json_str);
    const char* json = json_str.c_str();
    
    // Decode a command packet if it is a valid message
    if (status >= 0)
    {
        rapidjson::Document d;
        d.Parse(json);
        // std::cout << json_str << std::endl;

        // Document must be an object and have the appropriate members
        // TODO: make this validator better
        if(d.IsObject() && d.HasMember("roll") && d.HasMember("pitch") && d.HasMember("yaw"))
        {
            // Get the values from this JSON packet
            rapidjson::Value& roll_val = d["roll"];         // deg
            rapidjson::Value& pitch_val = d["pitch"];       // deg
            rapidjson::Value& yaw_val = d["yaw"];           // deg
            rapidjson::Value& left_vel = d["left_vel"];     // m/s
            rapidjson::Value& right_vel = d["right_vel"];   // m/s

            // Calculate velocity
            double r = Robot::wheel_radius;
            double v = (left_vel.GetDouble() + right_vel.GetDouble()) / 2.0;
            double w = (right_vel.GetDouble() - left_vel.GetDouble()) / r;

            // Update the sensor data from this packet
            // sensor_data_->imu_roll = roll_val.GetDouble();
            // sensor_data_->imu_pitch = pitch_val.GetDouble();
            sensor_data_->imu_yaw = yaw_val.GetDouble() * M_PI / 180.0;
            sensor_data_->linear_velocity = v;
            sensor_data_->angular_velocity = w;
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

    // Convert v and w to left and right power
    double base_vel = ctrl_out_->u(0);
    double wheel_vel_adj = Robot::wheel_radius * ctrl_out_->u(1);
    double left = base_vel - wheel_vel_adj;
    double right = base_vel + wheel_vel_adj;

    this->send_motor_message(left, right);

    // Adding in a sleep results in fewer dropped packets
    usleep(20000);
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
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

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

void ArduinoInterface::send_motor_message(double left, double right)
{
    // Convert the motor velocities to strings
    std::string left_str = std::to_string(left);
    std::string right_str = std::to_string(right);

    // Build RapidJSON message
    rapidjson::StringBuffer out_buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(out_buffer);

    // Packet: {"left": 0.0, "right": 0.0}
    writer.StartObject();
    writer.Key("left");
    writer.String(left_str.c_str());
    writer.Key("right");
    writer.String(right_str.c_str());
    writer.EndObject();

    // Write to Arduino
    std::string out_str = out_buffer.GetString();
    out_str += "\n";
    this->arduino_write(out_str);
}