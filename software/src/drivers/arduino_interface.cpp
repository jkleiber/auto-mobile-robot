#include "drivers/arduino_interface.h"

ArduinoInterface::~ArduinoInterface(){}


void ArduinoInterface::update()
{
    // Convert v and w to left and right power
    double base_vel = ctrl_out_->u(0);
    double wheel_vel_adj = ctrl_out_->u(1)/(Robot::wheelbase_len);
    double left = base_vel - wheel_vel_adj;
    double right = base_vel + wheel_vel_adj;

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
    serial_port_.write_data(out_str.c_str());

    // Pull data from the serial channel
    std::string json_str = "";
    int status = serial_port_.read_data(&json_str);
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