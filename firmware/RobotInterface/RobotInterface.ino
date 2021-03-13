
// Libraries
#include <ArduinoJson.h>
#include <RobotLib.h>
#include <StreamUtils.h>

// Robot Devices
#include "Devices.h"
#include "Constants.h"

// Arduino JSON packets
// Read Commands
ReadBufferingStream serial_buffer(Serial, 64);
StaticJsonDocument<64> control_pkt;

// Write sensor data
StaticJsonDocument<256> sensor_pkt;

// Robot Orientation
// double roll = 0.0; // Sensor Z orientation
// double pitch = 0.0; // Sensor Y orientation
// double yaw = 0.0; // Sensor X orientation


// Robot motor commands
float left_power = 0.0;
float right_power = 0.0;


// Track control loop timing
unsigned long last_loop_time;


void setup()
{
    Serial.begin(9600);

    device_init();

    // Wait for Serial to be ready
    while(!Serial){
        delay(1);
    }
    
    // Serial.println("Orientation Sensor Test"); Serial.println("");
    
    /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    
    delay(1000);
        
    bno.setExtCrystalUse(true);

    // Start loop timing
    last_loop_time = millis();
}

void loop()
{
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    
    // Update robot orientation
    sensor_pkt["roll"] = event.orientation.z;
    sensor_pkt["pitch"] = event.orientation.y;
    sensor_pkt["yaw"] = event.orientation.x; // CW increasing

    /* Send */
    serializeJson(sensor_pkt, Serial);
    Serial.println();

    /* Display the floating point data */
    // Serial.print("X: ");
    // Serial.print(event.orientation.x, 4);
    // Serial.print("\tY: ");
    // Serial.print(event.orientation.y, 4);
    // Serial.print("\tZ: ");
    // Serial.print(event.orientation.z, 4);
    // Serial.println("");



    /* Receive */
    // Receive commands from Jetson Nano.
    if(serial_buffer.available())
    {
        // Deserialization
        DeserializationError error = deserializeJson(control_pkt, serial_buffer);

        // Notify if error
        if(error)
        {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.c_str());

            // Stop robot for safety.
            left_power = 0.0;
            right_power = 0.0;
        }
        // Otherwise update the drive controls
        else
        {
            // Update the drivetrain controls.
            left_power = control_pkt["left"].as<float>();
            right_power = control_pkt["right"].as<float>();
        }
    }

    // Tank drive
    left_motor.output(left_power);
    right_motor.output(right_power);

    // Wait for loop update time to elapse
    while((millis() - last_loop_time) < LOOP_PERIOD){
        delay(1);
    }

    // Update timing tracker
    last_loop_time = millis();
}