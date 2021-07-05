
// Libraries
#include <ArduinoJson.h>
#include <RobotLib.h>
#include <StreamUtils.h>

// Robot Devices
#include "PID.h"
#include "Devices.h"
#include "Constants.h"

// Arduino JSON packets
// Read Commands
ReadBufferingStream serial_buffer(Serial, 64);
StaticJsonDocument<64> control_pkt;

// Write sensor data
StaticJsonDocument<128> sensor_pkt;

// Robot Orientation
// double roll = 0.0; // Sensor Z orientation
// double pitch = 0.0; // Sensor Y orientation
// double yaw = 0.0; // Sensor X orientation

// Wheel velocity
float left_vel = 0.0;
float right_vel = 0.0;

// Robot motor commands
float left_vel_desired = 0.0;
float right_vel_desired = 0.0;

// PIDs for tracking velocity commands
PID left_vel_pid(0.0, 0.8, 1.5, 0);
PID right_vel_pid(0.0, 0.8, 1.5, 0);
float left_power = 0.0, right_power = 0.0;

// Deadband (set to 0 if absolute value is smaller than this)
float deadband = 0.01;

// Track control loop timing
unsigned long last_loop_time;

// Safety
bool stop_robot = false;

/* SERIAL */
const byte numChars = 64;
char receivedChars[numChars]; // an array to store the received data
char sendChars[256];
boolean newData = false;

void serialEvent() {
 static byte ndx = 0;
 char endMarker = '\n';
 char rc;
 
 while (Serial.available() > 0){// && newData == false) {
   rc = (char)Serial.read();
  
   if (rc != endMarker) {
     receivedChars[ndx] = rc;
     ndx++;
     if (ndx >= numChars) {
      ndx = numChars - 1;
     }
   }
   else {
     receivedChars[ndx] = '\0'; // terminate the string
     ndx = 0;
     newData = true;
   }
 }
}

void setup()
{
    Serial.begin(115200);

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

    /* Configure PID */
    // Left
    left_vel_pid.setOutputRange(-1, 1);
    left_vel_pid.setIntegratorBounds(-1, 1);
    // Right
    right_vel_pid.setOutputRange(-1, 1);
    right_vel_pid.setIntegratorBounds(-1, 1);

    // Start loop timing
    last_loop_time = millis();
}

void loop()
{
    /* Get a new IMU sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    
    // Update robot orientation
    sensor_pkt["roll"] = event.orientation.z;
    sensor_pkt["pitch"] = event.orientation.y;
    sensor_pkt["yaw"] = 360.0 - event.orientation.x; // The sensor is CW increasing, so invert it to be CCW increasing to be in the correct coordinate frame
    sensor_pkt["right_vel"] = right_vel;
    sensor_pkt["left_vel"] = left_vel;

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
    if(newData)
    {
        // Deserialization
        // DeserializationError error = deserializeJson(control_pkt, serial_buffer);
        DeserializationError error = deserializeJson(control_pkt, receivedChars);

        // Notify if error
        if(error)
        {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());

            // Stop robot.
            left_vel_desired = 0.0;
            right_vel_desired = 0.0;
            stop_robot = true;
        }
        // Otherwise update the drive controls
        else
        {
            // Update the drivetrain controls.
            left_vel_desired = control_pkt["left"].as<float>();
            right_vel_desired = control_pkt["right"].as<float>();
            stop_robot = false;
        }

        /* Send */
        serializeJson(sensor_pkt, sendChars);
        Serial.print(sendChars);
        Serial.print("\n");
        newData = false;
    }

    // Calculate the motor output based on PID
    left_power = -1.0 * (left_vel_pid.update(left_vel_desired, left_vel));
    right_power = -1.0 * (right_vel_pid.update(right_vel_desired, right_vel));
    // left_power = -1.0 * (left_vel_desired + left_vel_pid.update(left_vel_desired, left_vel));
    // right_power = -1.0 * (right_vel_desired + right_vel_pid.update(right_vel_desired, right_vel));

    // Clamp power output
    left_power = RLUtil::clamp(left_power, -1, 1);
    right_power = RLUtil::clamp(right_power, -1, 1);

    // Determine if the deadband needs to be applied
    if(fabs(left_vel_desired) < fabs(deadband))
    {
        left_power = 0.0;

        // Reset to avoid integral windup on startup with new speed commands
        left_vel_pid.reset();
    }
    if(fabs(right_vel_desired) < fabs(deadband))
    {
        right_power = 0.0;

        // Reset to avoid integral windup on startup with new speed commands
        right_vel_pid.reset();
    }

    // Tank drive
    if(!stop_robot)
    {
        left_motor.output(left_power);
        right_motor.output(right_power);
    }
    else
    {
        left_motor.output(0.0);
        right_motor.output(0.0);
    }
    

    // Wait for loop update time to elapse
    while((millis() - last_loop_time) < LOOP_PERIOD){
        delay(1);
    }

    /* Calculate new wheel velocities */
    float elapsed_sec = (millis() - last_loop_time) / 1000.0;
    right_vel = right_encoder.getValue() / elapsed_sec;
    left_vel  = left_encoder.getValue() / elapsed_sec;

    // Reset the encoders
    left_encoder.reset();
    right_encoder.reset();

    // Update timing tracker
    last_loop_time = millis();
}